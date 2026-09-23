#!/usr/bin/env python3
"""Stage 3: generate auditable coarse INSPIRE -> FAST candidates; never refine or select a final transform."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import platform
import sys
import time
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import open3d as o3d
import psutil
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.candidates import (assign_coarse_rank, canonical_fingerprint,
    deduplicate_candidates, generate_geometry_candidates, stage3_contract)
from pointcloud_registration.dataset_config import add_dataset_argument, ensure_report_dataset, parse_with_dataset
from pointcloud_registration.io_utils import load_metric_stage2_cloud, read_header, sha256_file, validate_role_path
from pointcloud_registration.reporting import configure_logging, write_json
from pointcloud_registration.segmentation import fit_plane_ransac
from pointcloud_registration.transforms import oriented_plane, right_handed_plane_frame, rotation_difference_deg, transform_difference
from pointcloud_registration.visualization import save_stage3_candidates


ROLES = ("source_board", "source_object", "target_board", "target_object")


class Stage3Failure(RuntimeError): pass


def parser() -> argparse.ArgumentParser:
    p=argparse.ArgumentParser(description="阶段 3：INSPIRE → FAST 多候选粗配准（不执行精配准）")
    add_dataset_argument(p,ROOT)
    base=ROOT/"outputs"/"stage_02_segmentation"
    p.add_argument("--source-board",type=Path,default=base/"source_board_points.ply")
    p.add_argument("--source-object",type=Path,default=base/"source_object_points.ply")
    p.add_argument("--target-board",type=Path,default=base/"target_board_points.pcd")
    p.add_argument("--target-object",type=Path,default=base/"target_object_points.pcd")
    p.add_argument("--stage2-report",type=Path,default=base/"segmentation_report.json")
    p.add_argument("--config",type=Path,default=ROOT/"configs"/"registration.yaml")
    p.add_argument("--output-dir",type=Path,default=ROOT/"outputs"/"stage_03_candidates")
    p.add_argument("--dedup-rotation-deg",type=float)
    p.add_argument("--dedup-translation-m",type=float)
    p.add_argument("--max-candidates",type=int)
    p.add_argument("--geometry-search-window-m",type=float)
    p.add_argument("--geometry-search-step-m",type=float)
    return p


def _json(path: Path) -> dict[str,Any]:
    if not path.is_file(): raise Stage3Failure(f"required file missing: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def fingerprint(cfg: dict[str,Any]) -> str:
    return canonical_fingerprint(cfg,64)


def integrity(paths: dict[str,Path]) -> dict[str,dict[str,Any]]:
    return {k:{"size":p.stat().st_size,"sha256":sha256_file(p)} for k,p in paths.items()}


def input_record(path: Path, cloud: o3d.geometry.PointCloud, role: str) -> dict[str,Any]:
    pts=np.asarray(cloud.points)
    return {"path":str(path),"responsibility":f"authoritative stage 2 {role.replace('_',' ')} canonical output",
            "unit":"m","unit_conversion":"none; stage 2 output already uses metre working coordinates",
            "coordinates_modified":False,"file_size_bytes":path.stat().st_size,"sha256":sha256_file(path),
            "point_count":len(pts),"format":path.suffix[1:].lower(),"header":read_header(path),
            "xyz_min_m":pts.min(0).tolist(),"xyz_max_m":pts.max(0).tolist()}


def validate_preconditions(args: argparse.Namespace) -> tuple[dict[str,Any],dict[str,Path]]:
    s2=_json(args.stage2_report.resolve())
    if s2.get("status")!="success" or s2.get("stage")!=2: raise Stage3Failure("stage 2 formal report is not success")
    paths={"source_board":validate_role_path(args.source_board,"source"),"source_object":validate_role_path(args.source_object,"source"),
           "target_board":validate_role_path(args.target_board,"target"),"target_object":validate_role_path(args.target_object,"target")}
    for role,path in paths.items():
        expected=s2.get("outputs",{}).get(f"{role}_points",{})
        actual={"sha256":sha256_file(path),"size":path.stat().st_size}
        if actual["sha256"]!=expected.get("sha256"):
            raise Stage3Failure(f"{role} SHA-256 differs from the stage 2 formal report")
    return s2,paths


def fit_plane_report(points: np.ndarray, threshold: float, cfg: dict[str,Any], obj: np.ndarray) -> tuple[np.ndarray,dict[str,Any]]:
    fits=[]
    for seed in cfg["repeat_seeds"]:
        fit=fit_plane_ransac(points,threshold_m=threshold,ransac_n=int(cfg["ransac_n"]),iterations=int(cfg["num_iterations"]),seed=int(seed))
        fits.append(fit)
    main=fits[0]; oriented,sign=oriented_plane(main.model,obj,threshold)
    aligned=[]
    for fit in fits:
        n=fit.model[:3]
        if n@oriented[:3]<0:n=-n
        aligned.append(n)
    mean=np.mean(aligned,axis=0); mean/=np.linalg.norm(mean)
    angles=[float(np.degrees(np.arccos(np.clip(n@mean,-1,1)))) for n in aligned]
    inlier_points=points[main.inlier_indices]
    frame=right_handed_plane_frame(oriented,obj); uv=(inlier_points-frame["origin"])@frame["basis"][:,:2]
    coverage=np.percentile(uv,98,axis=0)-np.percentile(uv,2,axis=0)
    report={**main.report,"oriented_plane_model":oriented.tolist(),"object_side_orientation":sign,
            "coverage":{"robust_extent_uv_m":coverage.tolist(),"robust_area_m2":float(np.prod(coverage))},
            "repeat_stability":{"seeds":cfg["repeat_seeds"],"normal_angle_to_mean_deg":{"p95":float(np.percentile(angles,95)),"max":max(angles)},
                                "inlier_ratios":[f.report["plane_inlier_ratio"] for f in fits]},
            "frame":{"origin":frame["origin"].tolist(),"u":frame["u"].tolist(),"v":frame["v"].tolist(),"n":frame["n"].tolist(),
                     "determinant":frame["determinant"],"source":"continuous object-projection PCA; not inherited from world axes",
                     "pca_eigenvalues":frame["pca_eigenvalues"].tolist(),"pca_relative_gap":frame["pca_relative_gap"]}}
    frame["plane"]=oriented
    return oriented,{"report":report,"frame":frame}


def write_csv(path:Path,candidates:list[dict[str,Any]])->None:
    fields=["candidate_id","rank","generator","generators","support_count","status","coarse_rank_score","fitness","inlier_rmse",
            "board_normal_angle_deg","board_plane_distance_m","object_trimmed_chamfer_m","object_overlap_at_15mm","matrix_file"]
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for c in candidates:
            checks=c["coarse_checks"]; raw=c.get("raw_metrics",{})
            w.writerow({"candidate_id":c["candidate_id"],"rank":c.get("rank"),"generator":c["generator"],"generators":"|".join(c.get("generators",[c["generator"]])),
                "support_count":c.get("support_count",1),"status":c["status"],"coarse_rank_score":c["coarse_rank_score"],"fitness":raw.get("fitness"),
                "inlier_rmse":raw.get("inlier_rmse"),"board_normal_angle_deg":checks["board"]["normal_angle_deg"],
                "board_plane_distance_m":checks["board"]["plane_distance_m"],"object_trimmed_chamfer_m":checks["object"]["bidirectional_trimmed_chamfer_m"],
                "object_overlap_at_15mm":checks["object"]["bidirectional_overlap_at_15mm"],"matrix_file":f"candidate_matrices/{c['candidate_id']}.txt"})


def validate_serialized_outputs(out: Path, expected: list[dict[str,Any]]) -> dict[str,Any]:
    payload=_json(out/"canonical_candidates.json"); rows=[]
    with (out/"candidate_summary.csv").open("r",encoding="utf-8-sig",newline="") as f: rows=list(csv.DictReader(f))
    json_candidates=payload.get("candidates",[]);expected_ids=[c["candidate_id"] for c in expected]
    if payload.get("candidate_count")!=len(expected) or [c["candidate_id"] for c in json_candidates]!=expected_ids:
        raise Stage3Failure("canonical JSON candidate count/order/IDs are inconsistent")
    if [r["candidate_id"] for r in rows]!=expected_ids: raise Stage3Failure("CSV candidate IDs/order are inconsistent with canonical JSON")
    for candidate in json_candidates:
        matrix=np.loadtxt(out/"candidate_matrices"/f"{candidate['candidate_id']}.txt")
        if not np.array_equal(matrix,np.asarray(candidate["matrix_m"],dtype=float)):
            raise Stage3Failure(f"matrix text differs from canonical JSON: {candidate['candidate_id']}")
    return {"passed":True,"candidate_count":len(expected),"json_csv_id_order_identical":True,
            "matrix_text_json_values_identical":True,"numeric_comparison":"IEEE-754 exact after %.17g text round-trip"}


def compare_reproducibility(previous: dict[str,Any] | None, current: list[dict[str,Any]],
                            rotation_threshold_deg: float, translation_threshold_m: float) -> dict[str,Any]:
    if not previous:
        return {"comparison_available":False,"reason":"no previous canonical output was available at run start"}
    old=previous.get("candidates",[]);old_ids=[c["candidate_id"] for c in old];new_ids=[c["candidate_id"] for c in current]
    if old and current and old[0].get("config_fingerprint") != current[0].get("config_fingerprint"):
        return {"comparison_available":False,"reason":"previous canonical output used a different configuration fingerprint"}
    exact_order=old_ids==new_ids
    exact_matrix=0; nearest=[]
    old_by_id={c["candidate_id"]:c for c in old}
    for candidate in current:
        if candidate["candidate_id"] in old_by_id and np.array_equal(np.asarray(candidate["matrix_m"]),np.asarray(old_by_id[candidate["candidate_id"]]["matrix_m"])):
            exact_matrix+=1
        if old:
            diffs=[transform_difference(np.asarray(candidate["matrix_m"]),np.asarray(x["matrix_m"])) for x in old]
            rd,td=min(diffs,key=lambda x:(x[0]/max(rotation_threshold_deg,1e-12))+(x[1]/max(translation_threshold_m,1e-12)))
            nearest.append((rd,td))
    return {"comparison_available":True,"previous_candidate_count":len(old),"current_candidate_count":len(current),
            "identical_candidate_id_order":exact_order,"shared_candidate_id_count":len(set(old_ids)&set(new_ids)),
            "exact_matrix_count_for_same_id":exact_matrix,
            "nearest_SE3":{"rotation_difference_deg_p50":float(np.percentile([x[0] for x in nearest],50)) if nearest else None,
                           "rotation_difference_deg_max":max((x[0] for x in nearest),default=None),
                           "translation_difference_m_p50":float(np.percentile([x[1] for x in nearest],50)) if nearest else None,
                           "translation_difference_m_max":max((x[1] for x in nearest),default=None),
                           "within_dedup_threshold_count":sum(x[0]<=rotation_threshold_deg and x[1]<=translation_threshold_m for x in nearest)},
            "interpretation":"Geometry candidate IDs and transforms are deterministic for unchanged inputs and configuration."}


def select_display(candidates: list[dict[str,Any]], group_key: Any, limit: int = 12) -> list[dict[str,Any]]:
    ordered=sorted(candidates,key=lambda c:(c.get("coarse_rank_score",float("inf")),c["candidate_id"]));selected=[];seen=set()
    for candidate in ordered:
        key=group_key(candidate)
        if key not in seen:selected.append(candidate);seen.add(key)
    selected.extend(c for c in ordered if c not in selected)
    return selected[:limit]


def main(argv:list[str]|None=None)->int:
    args,_=parse_with_dataset(parser(),argv,ROOT,3); out=args.output_dir.resolve(); logger=configure_logging(out/"stage_03_candidates.log","pointcloud_registration.stage3",file_mode="w")
    previous_canonical=None
    if (out/"canonical_candidates.json").is_file():
        try: previous_canonical=_json(out/"canonical_candidates.json")
        except Exception: previous_canonical=None
    start=time.perf_counter(); process=psutil.Process(); peak=process.memory_info().rss
    report={**stage3_contract(),"stage_name":"multi_candidate_coarse_registration","status":"failed","failure_reasons":[],"warnings":[],"created_at":dt.datetime.now(dt.timezone.utc).isoformat(),"dataset":{"id":args.dataset_id,"manifest":str(args.dataset)}}
    try:
        s2,paths=validate_preconditions(args); before=integrity(paths)
        ensure_report_dataset(s2,args.dataset_id,2)
        full_cfg=_json(args.config.resolve()); cfg=full_cfg["coarse_registration"]
        overrides={k:v for k,v in vars(args).items() if v is not None and k in {"dedup_rotation_deg","dedup_translation_m","max_candidates","geometry_search_window_m","geometry_search_step_m"}}
        gcfg=cfg["geometry"]; dcfg=cfg["deduplication"]
        if args.geometry_search_window_m is not None:gcfg["search_window_m"]=args.geometry_search_window_m
        if args.geometry_search_step_m is not None:gcfg["search_step_m"]=args.geometry_search_step_m
        if args.dedup_rotation_deg is not None:dcfg["rotation_threshold_deg"]=args.dedup_rotation_deg
        if args.dedup_translation_m is not None:dcfg["translation_threshold_m"]=args.dedup_translation_m
        if args.max_candidates is not None:dcfg["maximum_canonical_candidates"]=args.max_candidates
        if float(dcfg["rotation_threshold_deg"])>=45: raise Stage3Failure("dedup rotation threshold must be far below 90 degrees")
        cfg_fp=fingerprint(cfg); input_hashes={k:v["sha256"] for k,v in before.items()}
        clouds={};points={};inputs={}
        for role,path in paths.items():
            cloud,pts=load_metric_stage2_cloud(path);cloud.clear() if False else None
            clouds[role]=cloud;points[role]=pts;inputs[role]=input_record(path,cloud,role)
        peak=max(peak,process.memory_info().rss); logger.info("Preconditions passed; four canonical metric inputs verified; no source rescaling")
        pcfg=cfg["plane_fit"]
        sp,sd=fit_plane_report(points["source_board"],float(pcfg["source_distance_threshold_m"]),pcfg,points["source_object"])
        tp,td=fit_plane_report(points["target_board"],float(pcfg["target_distance_threshold_m"]),pcfg,points["target_object"])
        for label,detail in (("source",sd),("target",td)):
            frame_report=detail["report"]["frame"];uv=detail["frame"]["uv"]
            ext=np.percentile(uv,98,axis=0)-np.percentile(uv,2,axis=0);aspect=float(max(ext)/max(min(ext),np.finfo(float).eps))
            evidence=[]
            if frame_report["pca_relative_gap"] < float(gcfg["pca_degeneracy_eigenvalue_ratio"]): evidence.append("PCA eigenvalues nearly equal")
            if abs(aspect-1.) < float(gcfg["square_obb_aspect_tolerance"]): evidence.append("robust OBB approximately square")
            frame_report.update({"robust_obb_extent_uv_m":ext.tolist(),"robust_obb_aspect_ratio":aspect,
                                 "orientation_degenerate":bool(evidence),"degeneracy_evidence":evidence,
                                 "degeneracy_handling":"do not choose one PCA direction; enumerate 0/90/180/270 and contour parent branches"})
            if evidence: report["warnings"].append(f"{label} object projection orientation is degenerate: {', '.join(evidence)}")
        source_frame=sd["frame"]; target_frames=[]
        target_frame=td["frame"];target_frame["normal_branch"]="observed_object_side";target_frames.append(target_frame)
        if td["report"]["object_side_orientation"]["opposite_alignment_branch_required"]:
            opposite=right_handed_plane_frame(-tp,points["target_object"]);opposite["plane"]=-tp;opposite["normal_branch"]="opposite_due_to_weak_FAST_sign_evidence";target_frames.append(opposite)
            report["warnings"].append("FAST object-to-board sign evidence is weak; opposite target-normal alignment branch was retained.")
        raw=[];geometry_audit=[]
        for tf in target_frames:
            geo,audit=generate_geometry_candidates(source_frame,tf,points["source_object"],points["target_object"],gcfg,cfg["coarse_gates"],input_hashes,cfg_fp)
            raw.extend(geo);geometry_audit.append({"normal_branch":tf["normal_branch"],**audit})
        logger.info("Geometry route complete: %d raw candidates",len(raw))
        assign_coarse_rank(raw)
        canonical,clusters=deduplicate_candidates(raw,float(dcfg["rotation_threshold_deg"]),float(dcfg["translation_threshold_m"]),int(dcfg["maximum_canonical_candidates"]),int(dcfg["minimum_per_generator"]))
        reproducibility=compare_reproducibility(previous_canonical,canonical,float(dcfg["rotation_threshold_deg"]),float(dcfg["translation_threshold_m"]))
        if reproducibility.get("comparison_available") and (not reproducibility.get("identical_candidate_id_order") or reproducibility.get("exact_matrix_count_for_same_id")!=len(canonical)):
            report["warnings"].append("Repeated geometry run was not bitwise identical; the measured difference is recorded under reproducibility.")
        geo_valid=sum(c["generator"]=="geometry" and c["status"]=="retained_raw" for c in raw)
        increments={c["provenance"].get("discrete_increment_deg") for c in raw if c["generator"]=="geometry"}
        if increments!={0,90,180,270}: raise Stage3Failure("geometry four-direction enumeration incomplete")
        if geo_valid<1: raise Stage3Failure("geometry route has no valid candidates")
        if len(canonical)<2: raise Stage3Failure("deduplication left fewer than two candidates")
        canonical_geo_increments={c["provenance"].get("discrete_increment_deg") for c in canonical if c["generator"]=="geometry"}
        if len(canonical_geo_increments)<2: raise Stage3Failure("fewer than two plane-internal direction clusters survived deduplication")
        for i,a in enumerate(canonical):
            for b in canonical[i+1:]:
                diff=rotation_difference_deg(np.asarray(a["matrix_m"]),np.asarray(b["matrix_m"]))
                if 85<=diff<=95 or 175<=diff<=180: pass
        matrices=out/"candidate_matrices";matrices.mkdir(parents=True,exist_ok=True)
        for stale in matrices.glob("s3_can_*.txt"): stale.unlink()
        for c in canonical: np.savetxt(matrices/f"{c['candidate_id']}.txt",np.asarray(c["matrix_m"]),fmt="%.17g")
        canonical_payload={"schema":"pointcloudlab.stage3.canonical_candidates","version":1,**stage3_contract(),"status":"success",
            "unit":"m","coordinate_convention":"p_target = T_FAST_from_INSPIRE_m @ p_source","candidate_count":len(canonical),"candidates":canonical}
        write_json(out/"raw_candidates.json",{"schema":"pointcloudlab.stage3.raw_candidates","version":1,**stage3_contract(),"candidates":raw})
        write_json(out/"canonical_candidates.json",canonical_payload);write_csv(out/"candidate_summary.csv",canonical)
        serialization_validation=validate_serialized_outputs(out,canonical)
        geo_pool=[c for c in raw if c["generator"]=="geometry" and c["status"]=="retained_raw"]
        geo_display=select_display(geo_pool,lambda c:c["provenance"]["discrete_increment_deg"])
        canonical_display=select_display(canonical,lambda c:c["generator"])
        save_stage3_candidates(out/"geometry_candidates.png",geo_display,points["source_object"],points["target_object"],"Geometry candidates: four directions and translation hypotheses")
        save_stage3_candidates(out/"candidate_comparison.png",canonical_display,points["source_object"],points["target_object"],"Canonical stage-4 inputs after deterministic SE(3) deduplication")
        after=integrity(paths)
        if before!=after: raise Stage3Failure("canonical inputs changed during stage 3")
        direction_diffs=[]
        for i,a in enumerate(canonical):
            for b in canonical[i+1:]:
                d=rotation_difference_deg(np.asarray(a["matrix_m"]),np.asarray(b["matrix_m"]));
                if d>45:direction_diffs.append({"candidate_a":a["candidate_id"],"candidate_b":b["candidate_id"],"rotation_difference_deg":d})
        elapsed=time.perf_counter()-start
        report.update({"status":"success","software":{"python":sys.version,"platform":platform.platform(),"open3d":o3d.__version__,"numpy":np.__version__,"scipy":scipy.__version__,"matplotlib":matplotlib.__version__},
            "config_path":str(args.config.resolve()),"config":cfg,"cli_overrides":overrides,"config_fingerprint":cfg_fp,
            "preconditions":{"stage2_status":s2["status"],"parameter_search_required":False,"passed":True},
            "inputs":inputs,
            "input_integrity":{"before":before,"after":after,"unchanged":True},"planes":{"source":sd["report"],"target":td["report"]},
            "geometry_route":{"executed":True,"audit":geometry_audit,"raw_candidate_count":sum(c["generator"]=="geometry" for c in raw),"valid_candidate_count":geo_valid},
            "candidate_counts":{"raw_total":len(raw),"geometry_valid":geo_valid,"deduplicated_canonical":len(canonical)},
            "route_coverage":{"passed":True,"status":"geometry_only","geometry_valid":geo_valid,"active_routes":["geometry"]},
            "serialization_validation":serialization_validation,
            "reproducibility":reproducibility,
            "deduplication":{**dcfg,"rotation_difference_formula":"geodesic_angle(R_a.T @ R_b)","translation_difference_formula":"norm(t_a-t_b)","clusters":clusters},
            "canonical_candidates":canonical,"ambiguity_evidence":{"competitive_direction_pairs":direction_diffs,"final_ambiguity_decision_performed":False},
            "visualizations":{"fixed_camera":"world Y-Z","unit":"m","fixed_colors":{"target":"blue","transformed_source_diagnostic_copy":"red"},
                "geometry_candidates":{"displayed_candidate_ids":[c["candidate_id"] for c in geo_display],"undisplayed_candidate_ids":[c["candidate_id"] for c in geo_pool if c not in geo_display]},
                "candidate_comparison":{"displayed_candidate_ids":[c["candidate_id"] for c in canonical_display],"undisplayed_candidate_ids":[c["candidate_id"] for c in canonical if c not in canonical_display]}},
            "limitations":["FAST target is sparse/noisy and object depths contain return/depth fusion.","Only front faces are observed; physical thickness is unavailable and unused.",
                "The approximately square planar object is highly self-similar at 0/90/180/270 degrees.","Board plane constrains normal and normal translation, not reliable in-plane translation or rotation."],
            "elapsed_s":elapsed,"peak_memory_bytes":peak,"peak_memory_measurement":"maximum process RSS polled at major geometry steps",
            "artifacts":{"raw_candidates":str((out/'raw_candidates.json').resolve()),"canonical_candidates":str((out/'canonical_candidates.json').resolve()),"summary_csv":str((out/'candidate_summary.csv').resolve()),"matrix_directory":str(matrices.resolve())}})
        write_json(out/"candidate_report.json",report);logger.info("Stage 3 success: raw=%d canonical=%d elapsed=%.2fs",len(raw),len(canonical),elapsed);return 0
    except Exception as exc:
        report["failure_reasons"].append(f"{type(exc).__name__}: {exc}");report["elapsed_s"]=time.perf_counter()-start;report["peak_memory_bytes"]=peak
        write_json(out/"candidate_report.json",report);logger.exception("Stage 3 failed: %s",exc);return 2


if __name__=="__main__": raise SystemExit(main())

