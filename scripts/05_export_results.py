#!/usr/bin/env python3
"""Stage 5: explicitly select, export, reread, and verify the final transform."""

from __future__ import annotations
import argparse, copy, datetime as dt, json, os, platform, sys, time, traceback
from pathlib import Path
from typing import Any
import matplotlib, numpy as np, open3d as o3d, psutil, scipy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from pointcloud_registration.export import (CORE_FILES,VISUAL_FILES,ExportError,atomic_write_cloud,atomic_write_matrix,
    cloud_stats,create_visualizations,file_record,load_json,load_selected_refinement,merge_clouds,raw_mm_affine,
    transform_source_cloud,verify_cloud_roundtrip,verify_roi_correspondence,verify_transform_paths)
from pointcloud_registration.dataset_config import add_dataset_argument,ensure_report_dataset,parse_with_dataset
from pointcloud_registration.io_utils import read_header,sha256_file,validate_role_path
from pointcloud_registration.refinement import region_diagnostics
from pointcloud_registration.reporting import configure_logging,write_json
from pointcloud_registration.transforms import apply_transform,validate_rigid_transform

def parser()->argparse.ArgumentParser:
    p=argparse.ArgumentParser(description="阶段 5：导出并验证用户明确选择的 INSPIRE→FAST 变换")
    add_dataset_argument(p,ROOT)
    raw=ROOT/"data"/"raw"/"20260913"; s2=ROOT/"outputs"/"stage_02_segmentation";s3=ROOT/"outputs"/"stage_03_candidates";s4=ROOT/"outputs"/"stage_04_refinement"
    p.add_argument("--source-full",type=Path,default=raw/"20260913_195812_pc.ply");p.add_argument("--target-full",type=Path,default=raw/"09132052.pcd")
    p.add_argument("--stage1-report",type=Path,default=ROOT/"outputs"/"stage_01_inspection"/"inspection_report.json")
    p.add_argument("--stage2-report",type=Path,default=s2/"segmentation_report.json")
    p.add_argument("--stage3-report",type=Path,default=s3/"candidate_report.json");p.add_argument("--stage4-report",type=Path,default=s4/"refinement_report.json")
    p.add_argument("--refined-candidates",type=Path,default=s4/"refined_candidates.json");p.add_argument("--refinement-summary",type=Path,default=s4/"refinement_summary.csv")
    p.add_argument("--refined-matrix-dir",type=Path,default=s4/"refined_matrices");p.add_argument("--parent-candidate-id")
    p.add_argument("--selected-refined-candidate-id",help="同时核验选中的阶段 4 精配准候选 ID")
    p.add_argument("--allow-nonvalid-selection",action="store_true",
                   help="人工覆盖 refined_valid 安全门；仍导出阶段 4 的 matrix_m，并在报告中记录覆盖")
    p.add_argument("--confirm-nondefault-parent",action="store_true");p.add_argument("--config",type=Path,default=ROOT/"configs"/"registration.yaml")
    p.add_argument("--source-board",type=Path,default=s2/"source_board_points.ply");p.add_argument("--source-object",type=Path,default=s2/"source_object_points.ply")
    p.add_argument("--target-board",type=Path,default=s2/"target_board_points.pcd");p.add_argument("--target-object",type=Path,default=s2/"target_object_points.pcd")
    p.add_argument("--segmentation-preview",type=Path,default=s2/"segmentation_preview.png");p.add_argument("--candidate-comparison",type=Path,default=s3/"candidate_comparison.png")
    p.add_argument("--output-dir",type=Path,default=ROOT/"outputs"/"stage_05_export");p.add_argument("--visualization-sample-count",type=int)
    p.add_argument("--visualization-seed",type=int);p.add_argument("--verification-sample-count",type=int)
    return p

def _read_cloud(path:Path,role:str)->o3d.geometry.PointCloud:
    validate_role_path(path,role);c=o3d.io.read_point_cloud(str(path),remove_nan_points=False,remove_infinite_points=False)
    if not len(c.points):raise ExportError(f"empty point cloud: {path}")
    return c

def _stage_input(stage1:dict[str,Any],label:str)->dict[str,Any]:
    matches=[x for x in stage1.get("inputs",[]) if x.get("label")==label]
    if len(matches)!=1:raise ExportError(f"stage-1 report must contain one {label}")
    return matches[0]

def _integrity(paths:dict[str,Path])->dict[str,dict[str,Any]]:return {k:file_record(v) for k,v in sorted(paths.items())}

def _close_logger(logger)->None:
    for handler in list(logger.handlers):
        handler.flush();handler.close();logger.removeHandler(handler)

def main(argv:list[str]|None=None)->int:
    args,dataset=parse_with_dataset(parser(),argv,ROOT,5);out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    log_path=out/"stage_05_export.log";logger=configure_logging(log_path,"pointcloud_registration.stage5",file_mode="w")
    started=time.perf_counter();process=psutil.Process();peak=process.memory_info().rss
    report={"schema":"pointcloudlab.registration_report","version":"1.0","stage":5,"stage_name":"output_and_verification",
      "export_performed":False,"final_transform_selected":True,
      "selection_method":"explicit_user_selection",
      "selected_parent_candidate_id":args.parent_candidate_id,"selected_refined_candidate_id":None,
      "selected_stage4_status":None,"safety_gate_overridden":False,
      "nonvalid_selection_override_requested":args.allow_nonvalid_selection,
      "automatic_scoring_performed":False,"automatic_ranking_performed":False,
      "ambiguity_adjudication_performed":False,"transform_uniqueness_determined":False,
      "transform_direction":"INSPIRE_TO_FAST","coordinate_convention":"p_FAST,m = T_FAST_from_INSPIRE_m @ p_INSPIRE,m",
      "status":"failed","failure_reasons":[],"warnings":["No automatic scoring/ranking or ambiguity adjudication was performed.",
      "Transform direction/geometric uniqueness has not been determined."],
      "created_at":dt.datetime.now(dt.timezone.utc).isoformat(),"dataset":{"id":args.dataset_id,"manifest":str(args.dataset)}}
    try:
      configured_parent=dataset.get("selection",{}).get("parent_candidate_id")
      if not args.parent_candidate_id:raise ExportError("select a parent candidate in dataset.selection or --parent-candidate-id")
      if configured_parent and args.parent_candidate_id!=configured_parent and not args.confirm_nondefault_parent:raise ExportError("CLI parent differs from dataset selection; use --confirm-nondefault-parent")
      cfg_all=load_json(args.config.resolve());cfg=copy.deepcopy(cfg_all["export"])
      overrides={}
      if args.visualization_sample_count is not None:cfg["visualization_sample_count"]=args.visualization_sample_count;overrides["visualization_sample_count"]=args.visualization_sample_count
      if args.visualization_seed is not None:cfg["random_seeds"]["visualization"]=args.visualization_seed;overrides["visualization_seed"]=args.visualization_seed
      if args.verification_sample_count is not None:cfg["verification_sample_count"]=args.verification_sample_count;overrides["verification_sample_count"]=args.verification_sample_count
      active_stages=(1,2,3,4)
      reports={f"stage{i}":load_json(getattr(args,f"stage{i}_report").resolve()) for i in active_stages}
      for i in active_stages:
        if reports[f"stage{i}"].get("stage")!=i:raise ExportError(f"stage-{i} report stage mismatch")
        if i>=2 and reports[f"stage{i}"].get("status")!="success":raise ExportError(f"stage-{i} report is not success")
        ensure_report_dataset(reports[f"stage{i}"],args.dataset_id,i)
      for i in active_stages:
        if reports[f"stage{i}"].get("transform_direction")!="INSPIRE_TO_FAST":raise ExportError(f"stage-{i} transform direction mismatch")
      protected={"source_full":args.source_full.resolve(),"target_full":args.target_full.resolve(),"stage1_report":args.stage1_report.resolve(),"stage2_report":args.stage2_report.resolve(),
       "stage3_report":args.stage3_report.resolve(),"stage4_report":args.stage4_report.resolve(),"refined_candidates":args.refined_candidates.resolve(),
       "refinement_summary":args.refinement_summary.resolve(),"source_board":args.source_board.resolve(),"source_object":args.source_object.resolve(),"target_board":args.target_board.resolve(),
       "target_object":args.target_object.resolve(),"segmentation_preview_source":args.segmentation_preview.resolve(),"candidate_comparison_source":args.candidate_comparison.resolve()}
      if args.selected_refined_candidate_id:
        protected["selected_stage4_matrix"]=(args.refined_matrix_dir.resolve()/f"{args.selected_refined_candidate_id}.txt")
      before=_integrity(protected);logger.info("前置报告、角色、方向与输入完整性检查")
      s1s,s1t=_stage_input(reports["stage1"],"source_full"),_stage_input(reports["stage1"],"target_full")
      for label,argp,base in (("source_full",args.source_full.resolve(),s1s),("target_full",args.target_full.resolve(),s1t)):
        if str(argp)!=str(Path(base["path"]).resolve()) or before[label]["size_bytes"]!=base["file_size_bytes"] or before[label]["sha256"]!=base["sha256"]:
          raise ExportError(f"{label} path/size/SHA-256 differs from stage-1 fixed input")
      expected_ref=args.selected_refined_candidate_id
      chosen,Tm,lineage=load_selected_refinement(args.refined_candidates.resolve(),args.refinement_summary.resolve(),args.refined_matrix_dir.resolve(),args.parent_candidate_id,expected_ref,float(cfg["rigid_atol"]),args.allow_nonvalid_selection)
      report["selected_refined_candidate_id"]=chosen["refined_candidate_id"]
      report["selected_stage4_status"]=chosen.get("status")
      report["safety_gate_overridden"]=chosen.get("status")!="refined_valid"
      if report["safety_gate_overridden"]:
        report["selection_method"]="explicit_user_override"
        report["warnings"].append(f"The selected stage-4 candidate status was {chosen.get('status')}; the refined_valid safety gate was explicitly overridden.")
      if reports["stage4"].get("status")!="success":raise ExportError("stage-4 formal report is not success")
      stage4_matches=[x for x in reports["stage4"].get("refined_candidates",[]) if x.get("parent_candidate_id")==args.parent_candidate_id]
      if len(stage4_matches)!=1 or not np.array_equal(np.asarray(stage4_matches[0]["matrix_m"]),Tm):raise ExportError("stage-4 report selected matrix differs from refined JSON")
      Traw=raw_mm_affine(Tm,float(cfg["source_to_m_scale"]));scale_matrix=np.diag([.001,.001,.001,1.])
      rigid=validate_rigid_transform(Tm,float(cfg["rigid_atol"]));relation_error=float(np.max(np.abs(Traw-Tm@scale_matrix)))
      if not rigid["valid"] or relation_error>float(cfg["matrix_atol"]):raise ExportError("final matrix validation or raw-mm affine relationship failed")
      source=_read_cloud(args.source_full.resolve(),"source");target=_read_cloud(args.target_full.resolve(),"target")
      source_raw=np.asarray(source.points).copy();target_original=np.asarray(target.points).copy();peak=max(peak,process.memory_info().rss)
      registered,attribute_policy=transform_source_cloud(source,Tm,.001);merged,merge_policy=merge_clouds(target,registered)
      logger.info("完整 INSPIRE 清洗、一次 mm→m、刚体变换与 FAST 合并完成")
      paths_a=verify_transform_paths(source_raw,Tm,Traw,float(cfg["point_roundtrip_atol_m"]),int(cfg["verification_sample_count"]),int(cfg["random_seeds"]["verification"]))
      sb=np.asarray(_read_cloud(args.source_board.resolve(),"source").points);so=np.asarray(_read_cloud(args.source_object.resolve(),"source").points)
      tb=np.asarray(_read_cloud(args.target_board.resolve(),"target").points);to=np.asarray(_read_cloud(args.target_object.resolve(),"target").points)
      roi=np.vstack([sb,so]);roi_check=verify_roi_correspondence(source_raw,roi,Tm,float(cfg["roi_correspondence_atol_m"]),int(cfg["verification_sample_count"]),43)
      target_unchanged=bool(np.array_equal(target_original,np.asarray(target.points)))
      if not target_unchanged:raise ExportError("FAST target coordinates changed in memory")
      metrics=region_diagnostics(Tm,sb,so,tb,to,.9,[.0025,.005],sample_limit=3000);recorded=chosen["final_metrics"]
      metric_diff={"board_normal_angle_deg":abs(metrics["board"]["normal_angle_deg"]-recorded["board"]["normal_angle_deg"]),
       "board_plane_distance_m":abs(metrics["board"]["plane_distance_m"]-recorded["board"]["plane_distance_m"]),
       "object_trimmed_m":abs(metrics["object"]["bidirectional_trimmed_distance_m"]-recorded["object"]["bidirectional_trimmed_distance_m"]),
       "object_overlap_5mm":abs(metrics["object"]["bidirectional_overlap_ratios"]["at_5mm"]-recorded["object"]["bidirectional_overlap_ratios"]["at_5mm"])}
      if max(metric_diff.values())>float(cfg["stage4_metric_atol"]):raise ExportError(f"stage-4 diagnostic reproduction mismatch: {metric_diff}")
      atomic_write_matrix(out/CORE_FILES[0],Tm);atomic_write_matrix(out/CORE_FILES[1],Traw)
      if not np.array_equal(np.loadtxt(out/CORE_FILES[0]),Tm) or not np.array_equal(np.loadtxt(out/CORE_FILES[1]),Traw):raise ExportError("stage-5 matrix reread mismatch")
      atomic_write_cloud(out/CORE_FILES[2],registered);atomic_write_cloud(out/CORE_FILES[3],merged)
      reg_rt=verify_cloud_roundtrip(out/CORE_FILES[2],registered,float(cfg["ply_roundtrip_atol_m"]),int(cfg["verification_sample_count"]),44)
      merge_rt=verify_cloud_roundtrip(out/CORE_FILES[3],merged,float(cfg["ply_roundtrip_atol_m"]),int(cfg["verification_sample_count"]),45)
      merged_reread=np.asarray(o3d.io.read_point_cloud(str(out/CORE_FILES[3])).points)
      target_prefix_error=float(np.max(np.abs(merged_reread[:len(target_original)]-target_original),initial=0.0))
      if target_prefix_error>float(cfg["ply_roundtrip_atol_m"]):raise ExportError("saved merged cloud changed the FAST target prefix")
      if reg_rt["reread_stats"]["point_count"]!=s1s["diagnostics"]["cleaning"]["valid_clean_point_count"]:raise ExportError("registered point count differs from stage-1 valid count")
      if merge_rt["reread_stats"]["point_count"]!=s1s["diagnostics"]["cleaning"]["valid_clean_point_count"]+s1t["diagnostics"]["cleaning"]["valid_clean_point_count"]:raise ExportError("merged point count differs from stage-1 valid sum")
      vis=create_visualizations(out,source_raw*.001,np.asarray(registered.points),target_original,apply_transform(so,Tm),to,cfg,args.segmentation_preview.resolve(),args.candidate_comparison.resolve(),f"{args.parent_candidate_id} → {chosen['refined_candidate_id']}")
      after=_integrity(protected)
      if before!=after:raise ExportError("one or more protected input files changed during stage 5")
      peak=max(peak,process.memory_info().rss);elapsed=time.perf_counter()-started
      s3all=reports["stage3"].get("canonical_candidates",[]);s4all=reports["stage4"].get("refined_candidates",[])
      selection_fact=("user explicitly selected the fixed parent/refined lineage and overrode the refined_valid safety gate"
                      if report["safety_gate_overridden"] else "user explicitly selected the fixed parent/refined lineage")
      report.update({"export_performed":True,"status":"success","selection_time":report["created_at"],"selection_fact":selection_fact,
       "software":{"python":platform.python_version(),"open3d":o3d.__version__,"numpy":np.__version__,"scipy":scipy.__version__,"matplotlib":matplotlib.__version__},
       "config_path":str(args.config.resolve()),"config":cfg_all,"cli_overrides":overrides,"random_seeds":cfg["random_seeds"],
       "inputs":{"source":{"role":"INSPIRE 2","unit":"mm","working_unit":"m",**before["source_full"],"format":"ply","raw_point_count":s1s["diagnostics"]["cleaning"]["raw_point_count"],"valid_point_count":s1s["diagnostics"]["cleaning"]["valid_clean_point_count"],"attributes":s1s["open3d_attributes"],"header":read_header(args.source_full.resolve())},
        "target":{"role":"FAST-LIVO2","unit":"m","working_unit":"m",**before["target_full"],"format":"pcd","raw_point_count":s1t["diagnostics"]["cleaning"]["raw_point_count"],"valid_point_count":s1t["diagnostics"]["cleaning"]["valid_clean_point_count"],"attributes":s1t["open3d_attributes"],"header":read_header(args.target_full.resolve())}},
       "prior_stage_summaries":{"stage1":{"report":before["stage1_report"],"data_check":"success/accepted baseline"},"stage2":{"report":before["stage2_report"],"segmentation_statistics":reports["stage2"].get("outputs")},
        "stage3":{"report":before["stage3_report"],"all_canonical_candidates":[{"candidate_id":x.get("candidate_id"),"generator":x.get("generator"),"generators":x.get("generators"),"matrix_m":x.get("matrix_m")} for x in s3all]},
        "stage4":{"report":before["stage4_report"],"all_lineage":[{"parent_candidate_id":x.get("parent_candidate_id"),"refined_candidate_id":x.get("refined_candidate_id"),"status":x.get("status"),"summary":x.get("final_metrics")} for x in s4all]}},
       "selected_lineage_validation":lineage,"matrices":{"T_fast_from_inspire_m":{"definition":"metric rigid transform: p_FAST,m = R p_INSPIRE,m + t", "values":Tm.tolist(),"validation":rigid},
        "T_fast_from_inspire_raw_mm":{"definition":"full affine mapping raw INSPIRE mm directly to FAST m; not a rigid transform","values":Traw.tolist(),"linear_determinant":float(np.linalg.det(Traw[:3,:3]))},
        "relationship":"T_raw_mm = T_m @ diag(0.001,0.001,0.001,1)","maximum_relationship_error":relation_error,"tolerance":cfg["matrix_atol"]},
       "unit_and_geometry_validation":{"source_transform":attribute_policy,"merge_policy":merge_policy,"two_path_equivalence":paths_a,"stage2_roi_correspondence":roi_check,
        "target_full_coordinates_unchanged":target_unchanged,"stage4_metrics_recomputed":metrics,"stage4_recorded_metrics":recorded,"stage4_metric_absolute_differences":metric_diff},
       "saved_cloud_validation":{"registered":reg_rt,"merged":merge_rt,"merged_target_prefix":{"passed":True,"point_count":len(target_original),"maximum_absolute_xyz_error_m":target_prefix_error,"target_was_not_transformed":True}},"visualizations":vis,"protected_input_integrity":{"before":before,"after":after,"unchanged":True},
       "elapsed_s":elapsed,"peak_memory_bytes":peak,"peak_memory_mib":peak/1048576,"peak_memory_measurement":"psutil RSS sampled at major checkpoints"})
      logger.info("阶段 5 导出与全部复读验证成功，耗时 %.3f s，峰值 RSS %.1f MiB",elapsed,peak/1048576)
      for handler in logger.handlers:handler.flush()
      outputs={}
      for name in CORE_FILES[:-1]+VISUAL_FILES+["stage_05_export.log"]:
        rec=file_record(out/name)
        if name.endswith(".ply"):rec.update(cloud_stats(o3d.io.read_point_cloud(str(out/name))))
        outputs[name]=rec
      report["outputs"]=outputs
      report["outputs"]["registration_report.json"]={"path":str((out/"registration_report.json").resolve()),
        "indexed":True,"size_bytes":"not_embedded_due_to_self_reference","sha256":"not_embedded_due_to_self_reference",
        "note":"The finalized report cannot contain its own final byte size/hash; verify the file externally."}
      write_json(out/"registration_report.json",report)
      _close_logger(logger)
      return 0
    except Exception as exc:
      report["failure_reasons"].append(f"{type(exc).__name__}: {exc}");report["elapsed_s"]=time.perf_counter()-started;report["peak_memory_bytes"]=max(peak,process.memory_info().rss)
      logger.error("阶段 5 失败：%s",exc);logger.debug(traceback.format_exc())
      try:write_json(out/"registration_report.json",report)
      except Exception:pass
      _close_logger(logger)
      return 1

if __name__=="__main__":raise SystemExit(main())

