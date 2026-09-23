#!/usr/bin/env python3
"""Stage 4: refine every canonical stage-3 candidate; never rank or select a final transform."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import platform
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import open3d as o3d
import psutil
import scipy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))

from pointcloud_registration.dataset_config import add_dataset_argument, ensure_report_dataset, parse_with_dataset
from pointcloud_registration.io_utils import sha256_file, validate_role_path
from pointcloud_registration.refinement import (RefinementError, final_sanity, prepare_balanced_level,
    refined_candidate_id, region_diagnostics, run_level, stage4_contract, validate_level_configuration)
from pointcloud_registration.reporting import configure_logging, write_json
from pointcloud_registration.transforms import rotation_difference_deg, validate_rigid_transform
from pointcloud_registration.visualization import save_stage4_comparison


class Stage4Failure(RuntimeError): pass


VISUALIZATION_FIELDS=("visualization_matrix_m","visualization_matrix_role","visualization_source_algorithm",
    "visualization_source_level","visualization_source_iteration","visualization_proposal_accepted",
    "visualization_rejection_reasons","visualization_from_parent_rotation_deg",
    "visualization_from_parent_translation_m")


def _legal_visualization_matrix(value:Any)->np.ndarray|None:
    """Return a finite rigid 4x4 diagnostic matrix, never a merely parseable matrix."""
    try:matrix=np.asarray(value,dtype=float)
    except (TypeError,ValueError):return None
    return matrix if validate_rigid_transform(matrix)["valid"] else None


def select_visualization_record(candidate:dict[str,Any])->dict[str,Any]:
    """Select diagnostic matrix without changing the candidate's formal matrix_m."""
    status=candidate["status"];parent=np.asarray(candidate["parent_matrix_m"],dtype=float);levels=candidate.get("levels",[])
    selected=None;source_algorithm=None;source_level=None;source_iteration=None;proposal_accepted=None
    rejection_reasons:list[str]=[];source_kind="formal_matrix"
    if status=="refined_valid":
        selected=_legal_visualization_matrix(candidate.get("matrix_m"));role="accepted_refinement"
        for level in reversed(levels):
            for row in reversed(level.get("trajectory",[])):
                proposed=_legal_visualization_matrix(row.get("proposed_matrix_m"))
                if row.get("accepted") and proposed is not None and selected is not None and np.array_equal(proposed,selected):
                    source_algorithm=level.get("algorithm");source_level=level.get("level_index")
                    source_iteration=row.get("iteration");proposal_accepted=True;source_kind="accepted_proposal";break
            if source_kind=="accepted_proposal":break
        if selected is None:raise Stage4Failure(f"refined_valid candidate has illegal matrix_m: {candidate.get('refined_candidate_id')}")
    else:
        role="rejected_diagnostic_proposal" if status=="no_safe_refinement" else "failed_diagnostic_proposal"
        for level in reversed(levels):
            for row in reversed(level.get("trajectory",[])):
                proposed=_legal_visualization_matrix(row.get("proposed_matrix_m"))
                if proposed is not None:
                    selected=proposed;source_algorithm=level.get("algorithm");source_level=level.get("level_index")
                    source_iteration=row.get("iteration");proposal_accepted=bool(row.get("accepted"));source_kind="proposed_matrix_m"
                    rejection_reasons=list(row.get("rejection_reasons",[])) or list(candidate.get("failure_reasons",[]));break
            if selected is not None:break
        if selected is None:
            source_kind="last_legal_current_matrix"
            role="rejected_diagnostic_current" if status=="no_safe_refinement" else "failed_diagnostic_current"
            for level in reversed(levels):
                for row in reversed(level.get("trajectory",[])):
                    current=_legal_visualization_matrix(row.get("matrix_m"))
                    if current is not None:
                        selected=current;source_algorithm=level.get("algorithm");source_level=level.get("level_index")
                        source_iteration=row.get("iteration");proposal_accepted=None;break
                if selected is not None:break
            rejection_reasons=list(candidate.get("failure_reasons",[]))+["no_legal_proposed_matrix_available_for_visualization"]
        if selected is None:
            selected=_legal_visualization_matrix(parent);source_kind="parent_matrix_m"
            role="rejected_diagnostic_parent" if status=="no_safe_refinement" else "failed_diagnostic_parent"
            rejection_reasons=list(candidate.get("failure_reasons",[]))+["no_legal_proposed_or_current_matrix_available_for_visualization"]
        if selected is None:raise Stage4Failure(f"no legal diagnostic matrix: {candidate.get('refined_candidate_id')}")
    rd=rotation_difference_deg(parent,selected);td=float(np.linalg.norm(selected[:3,3]-parent[:3,3]))
    return {"visualization_matrix_m":selected.tolist(),"visualization_matrix_role":role,
        "visualization_source_kind":source_kind,"visualization_source_algorithm":source_algorithm,
        "visualization_source_level":source_level,"visualization_source_iteration":source_iteration,
        "visualization_proposal_accepted":proposal_accepted,
        "visualization_rejection_reasons":list(dict.fromkeys(rejection_reasons)),
        "visualization_from_parent_rotation_deg":rd,"visualization_from_parent_translation_m":td}


def parser()->argparse.ArgumentParser:
    p=argparse.ArgumentParser(description="阶段 4：逐一执行多尺度 GICP → robust point-to-plane ICP")
    add_dataset_argument(p,ROOT)
    s2=ROOT/"outputs"/"stage_02_segmentation";s3=ROOT/"outputs"/"stage_03_candidates"
    p.add_argument("--source-board",type=Path,default=s2/"source_board_points.ply")
    p.add_argument("--source-object",type=Path,default=s2/"source_object_points.ply")
    p.add_argument("--target-board",type=Path,default=s2/"target_board_points.pcd")
    p.add_argument("--target-object",type=Path,default=s2/"target_object_points.pcd")
    p.add_argument("--stage2-report",type=Path,default=s2/"segmentation_report.json")
    p.add_argument("--stage3-report",type=Path,default=s3/"candidate_report.json")
    p.add_argument("--canonical-candidates",type=Path,default=s3/"canonical_candidates.json")
    p.add_argument("--candidate-matrices",type=Path,default=s3/"candidate_matrices")
    p.add_argument("--plan",type=Path,default=ROOT/"配准实验总规划.md")
    p.add_argument("--config",type=Path,default=ROOT/"configs"/"registration.yaml")
    p.add_argument("--output-dir",type=Path,default=ROOT/"outputs"/"stage_04_refinement")
    p.add_argument("--random-seed",type=int)
    p.add_argument("--max-iterations",type=int,help="override every level iteration cap")
    p.add_argument("--max-points-per-region",type=int)
    return p


def _json(path:Path)->dict[str,Any]:
    if not path.is_file():raise Stage4Failure(f"required file missing: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def file_record(path:Path)->dict[str,Any]:
    return {"path":str(path.resolve()),"size":path.stat().st_size,"sha256":sha256_file(path.resolve())}


def integrity(paths:dict[str,Path])->dict[str,dict[str,Any]]:
    return {k:file_record(v) for k,v in sorted(paths.items())}


def load_metric(path:Path)->tuple[o3d.geometry.PointCloud,np.ndarray]:
    cloud=o3d.io.read_point_cloud(str(path))
    pts=np.asarray(cloud.points)
    if len(pts)<3 or not np.isfinite(pts).all():raise Stage4Failure(f"invalid/empty canonical metric cloud: {path}")
    return cloud,pts.copy()


def validate_preconditions(args:argparse.Namespace)->tuple[list[dict[str,Any]],dict[str,Path],dict[str,Any]]:
    s2=_json(args.stage2_report.resolve());s3=_json(args.stage3_report.resolve());can=_json(args.canonical_candidates.resolve())
    for expected,report in ((2,s2),(3,s3)):
        if report.get("stage")!=expected or report.get("status")!="success":raise Stage4Failure(f"stage {expected} formal report is not success")
        ensure_report_dataset(report,args.dataset_id,expected)
    if s3.get("refinement_performed") is not False or s3.get("final_transform_selected") is not False:
        raise Stage4Failure("stage 3 boundary contract is invalid")
    plan=args.plan.resolve().read_text(encoding="utf-8")
    if "阶段 3“多候选粗配准”：**已完成正式运行并通过人工验收**" not in plan:
        raise Stage4Failure("stage 3 human approval is not recorded in the authoritative plan")
    if not args.dataset_approvals.get("stage3_reviewed",False):
        raise Stage4Failure("current dataset has not been approved: set approvals.stage3_reviewed=true after reviewing stage 3")
    if can.get("status")!="success" or can.get("transform_direction")!="INSPIRE_TO_FAST" or can.get("unit")!="m":
        raise Stage4Failure("canonical candidate direction/unit/status contract failed")
    candidates=can.get("candidates",[])
    if len(candidates)!=16 or can.get("candidate_count")!=16 or len({c.get("candidate_id") for c in candidates})!=16:
        raise Stage4Failure("canonical_candidates.json must contain exactly 16 unique candidates")
    report_candidates=s3.get("canonical_candidates",[])
    if [c.get("candidate_id") for c in report_candidates] != [c.get("candidate_id") for c in candidates]:
        raise Stage4Failure("stage 3 report and canonical JSON candidate ID/order differ")
    for a,b in zip(report_candidates,candidates):
        if not np.array_equal(np.asarray(a.get("matrix_m")),np.asarray(b.get("matrix_m"))):
            raise Stage4Failure(f"stage 3 report and canonical matrix differ: {a.get('candidate_id')}")
        matrix=np.asarray(b.get("matrix_m"),float); rigid=validate_rigid_transform(matrix)
        if not rigid["valid"]:raise Stage4Failure(f"illegal canonical matrix {b.get('candidate_id')}: {rigid['reasons']}")
        text=np.loadtxt(args.candidate_matrices.resolve()/f"{b['candidate_id']}.txt")
        if not np.array_equal(text,matrix):raise Stage4Failure(f"canonical JSON and matrix text differ: {b['candidate_id']}")
    matrix_files=list(args.candidate_matrices.resolve().glob("*.txt"))
    if len(matrix_files)!=16:raise Stage4Failure(f"candidate_matrices must contain exactly 16 text matrices, got {len(matrix_files)}")
    paths={"source_board":validate_role_path(args.source_board,"source"),"source_object":validate_role_path(args.source_object,"source"),
           "target_board":validate_role_path(args.target_board,"target"),"target_object":validate_role_path(args.target_object,"target"),
           "stage3_report":args.stage3_report.resolve(),"canonical_candidates":args.canonical_candidates.resolve()}
    for c in candidates:paths[f"matrix_{c['candidate_id']}"]=args.candidate_matrices.resolve()/f"{c['candidate_id']}.txt"
    for role in ("source_board","source_object","target_board","target_object"):
        expected=s2.get("outputs",{}).get(f"{role}_points",{}); actual=file_record(paths[role])
        if actual["sha256"]!=expected.get("sha256"):raise Stage4Failure(f"{role} differs from stage 2 formal output hash")
    audit={"stage2_status":"success","stage3_status":"success","stage3_human_approval_recorded":True,
           "dataset_stage3_reviewed":True,
           "candidate_count":16,"candidate_id_report_json_consistent":True,"matrix_report_json_text_consistent":True,"passed":True}
    return candidates,paths,audit


def write_summary(path:Path,candidates:list[dict[str,Any]])->None:
    fields=["refined_candidate_id","parent_candidate_id","status","failure_reasons","algorithms_attempted","levels_attempted",
            "parent_board_angle_deg","final_board_angle_deg","parent_board_distance_m","final_board_distance_m",
            "parent_object_trimmed_m","final_object_trimmed_m","parent_object_overlap_2_5mm","final_object_overlap_2_5mm",
            "parent_object_overlap_5mm","final_object_overlap_5mm","from_parent_rotation_deg","from_parent_translation_m",
            "rejected_update_count","matrix_file"]
    with path.open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for c in candidates:
            a,b=c["parent_metrics"],c["final_metrics"]
            w.writerow({"refined_candidate_id":c["refined_candidate_id"],"parent_candidate_id":c["parent_candidate_id"],"status":c["status"],
                "failure_reasons":"|".join(c["failure_reasons"]),"algorithms_attempted":"|".join(c["algorithms_attempted"]),"levels_attempted":len(c["levels"]),
                "parent_board_angle_deg":a["board"]["normal_angle_deg"],"final_board_angle_deg":b["board"]["normal_angle_deg"],
                "parent_board_distance_m":a["board"]["plane_distance_m"],"final_board_distance_m":b["board"]["plane_distance_m"],
                "parent_object_trimmed_m":a["object"]["bidirectional_trimmed_distance_m"],"final_object_trimmed_m":b["object"]["bidirectional_trimmed_distance_m"],
                "parent_object_overlap_2_5mm":a["object"]["bidirectional_overlap_ratios"]["at_2.5mm"],"final_object_overlap_2_5mm":b["object"]["bidirectional_overlap_ratios"]["at_2.5mm"],
                "parent_object_overlap_5mm":a["object"]["bidirectional_overlap_ratios"]["at_5mm"],"final_object_overlap_5mm":b["object"]["bidirectional_overlap_ratios"]["at_5mm"],
                "from_parent_rotation_deg":c["from_parent_rotation_deg"],"from_parent_translation_m":c["from_parent_translation_m"],
                "rejected_update_count":c["rejected_update_count"],
                "matrix_file":f"refined_matrices/{c['refined_candidate_id']}.txt"})


def write_iterations(path:Path,candidates:list[dict[str,Any]])->None:
    fields=["refined_candidate_id","parent_candidate_id","algorithm","level_index","voxel_size_m","threshold_m","iteration","accepted",
            "board_correspondence_count","board_correspondence_ratio","object_correspondence_count","object_correspondence_ratio","delta_rotation_deg","delta_translation_m","from_parent_rotation_deg",
            "from_parent_translation_m","raw_delta_rotation_deg","raw_delta_translation_m","step_translation_limit_m","step_scale_ratio",
            "backtracking_performed","backtracking_attempt_count","backtracking_results","final_decision_reason","raw_proposal_rejection_reasons",
            "stop_reason","rejection_reasons"]
    with path.open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for c in candidates:
            for level in c["levels"]:
                for row in level["trajectory"]:
                    w.writerow({"refined_candidate_id":c["refined_candidate_id"],"parent_candidate_id":c["parent_candidate_id"],
                        "algorithm":level["algorithm"],"level_index":level["level_index"],"voxel_size_m":level["parameters"]["voxel_size_m"],
                        "threshold_m":level["parameters"]["max_correspondence_distance_m"],"iteration":row["iteration"],"accepted":row["accepted"],
                        "board_correspondence_count":row.get("regions",{}).get("board",{}).get("correspondence_count"),
                        "board_correspondence_ratio":row.get("regions",{}).get("board",{}).get("correspondence_ratio"),
                        "object_correspondence_count":row.get("regions",{}).get("object",{}).get("correspondence_count"),
                        "object_correspondence_ratio":row.get("regions",{}).get("object",{}).get("correspondence_ratio"),
                        "delta_rotation_deg":row["delta_rotation_deg"],"delta_translation_m":row["delta_translation_m"],
                        "from_parent_rotation_deg":row["from_parent_rotation_deg"],"from_parent_translation_m":row["from_parent_translation_m"],
                        "raw_delta_rotation_deg":row.get("raw_delta_rotation_deg"),"raw_delta_translation_m":row.get("raw_delta_translation_m"),
                        "step_translation_limit_m":row.get("step_translation_limit_m"),"step_scale_ratio":row.get("step_scale_ratio"),
                        "backtracking_performed":row.get("backtracking_performed",False),
                        "backtracking_attempt_count":len(row.get("backtracking_attempts",[])),
                        "backtracking_results":json.dumps([{"attempt":x["attempt"],"scale_ratio":x["scale_ratio"],
                            "delta_rotation_deg":x["delta_rotation_deg"],"delta_translation_m":x["delta_translation_m"],
                            "from_parent_rotation_deg":x["from_parent_rotation_deg"],"from_parent_translation_m":x["from_parent_translation_m"],
                            "accepted":x["accepted"],"rejection_reasons":x["rejection_reasons"]} for x in row.get("backtracking_attempts",[])],separators=(",",":")),
                        "final_decision_reason":row.get("final_decision_reason"),
                        "raw_proposal_rejection_reasons":"|".join(row.get("raw_proposal_rejection_reasons",[])),
                        "stop_reason":level["stop_reason"] if row["iteration"]==level["actual_iterations"] else "",
                        "rejection_reasons":"|".join(row["rejection_reasons"])})


def backtracking_summary(candidates:list[dict[str,Any]])->dict[str,Any]:
    rows=[row for c in candidates for level in c["levels"] for row in level["trajectory"]]
    backtracked=[row for row in rows if row.get("backtracking_performed")]
    return {"iterations_total":len(rows),"raw_translation_limit_exceedances":len(backtracked),
        "accepted_after_backtracking":sum(row.get("accepted",False) for row in backtracked),
        "rejected_after_backtracking":sum(not row.get("accepted",False) for row in backtracked),
        "backtracking_attempts_total":sum(len(row.get("backtracking_attempts",[])) for row in backtracked),
        "raw_proposal_gate_failure_frequencies":dict(sorted(Counter(reason for row in rows for reason in row.get("raw_proposal_rejection_reasons",[])).items())),
        "backtracking_attempt_gate_failure_frequencies":dict(sorted(Counter(reason for row in backtracked for attempt in row.get("backtracking_attempts",[])
            for reason in attempt.get("rejection_reasons",[])).items())),
        "final_rejection_reason_frequencies":dict(sorted(Counter(reason for row in rows if not row.get("accepted",False)
            for reason in row.get("rejection_reasons",[])).items()))}


def validate_outputs(out:Path,candidates:list[dict[str,Any]])->dict[str,Any]:
    payload=_json(out/"refined_candidates.json")
    if payload.get("candidate_count")!=16 or [x["refined_candidate_id"] for x in payload["candidates"]]!=[x["refined_candidate_id"] for x in candidates]:
        raise Stage4Failure("refined JSON count/order/IDs are inconsistent")
    with (out/"refinement_summary.csv").open(encoding="utf-8-sig",newline="") as f:rows=list(csv.DictReader(f))
    if [x["refined_candidate_id"] for x in rows]!=[x["refined_candidate_id"] for x in candidates]:raise Stage4Failure("summary CSV IDs differ from JSON")
    for c in candidates:
        text=np.loadtxt(out/"refined_matrices"/f"{c['refined_candidate_id']}.txt")
        if not np.array_equal(text,np.asarray(c["matrix_m"])):raise Stage4Failure(f"refined matrix serialization mismatch: {c['refined_candidate_id']}")
    return {"passed":True,"candidate_count":16,"json_csv_id_order_identical":True,"matrix_text_json_values_identical":True}


def main(argv:list[str]|None=None)->int:
    args,_=parse_with_dataset(parser(),argv,ROOT,4);out=args.output_dir.resolve();logger=configure_logging(out/"stage_04_refinement.log","pointcloud_registration.stage4",file_mode="w")
    started=time.perf_counter();process=psutil.Process();peak=process.memory_info().rss
    report={**stage4_contract(),"stage_name":"candidate_refinement","status":"failed","failure_reasons":[],"warnings":[],
            "created_at":dt.datetime.now(dt.timezone.utc).isoformat(),"dataset":{"id":args.dataset_id,"manifest":str(args.dataset)}}
    try:
        parents,protected_paths,preconditions=validate_preconditions(args);before=integrity(protected_paths)
        full_cfg=_json(args.config.resolve());cfg=full_cfg["refinement"]
        overrides={k:v for k,v in vars(args).items() if v is not None and k in {"random_seed","max_iterations","max_points_per_region"}}
        if args.random_seed is not None:cfg["random_seed"]=args.random_seed
        if args.max_iterations is not None:
            for level in cfg["gicp_levels"]+cfg["robust_point_to_plane_levels"]:level["max_iterations"]=args.max_iterations
        if args.max_points_per_region is not None:cfg["sampling"]["maximum_points_per_region"]=args.max_points_per_region
        validate_level_configuration(cfg)
        clouds={};points={};inputs={}
        for role in ("source_board","source_object","target_board","target_object"):
            clouds[role],points[role]=load_metric(protected_paths[role]);inputs[role]={**file_record(protected_paths[role]),"unit":"m",
                "unit_conversion":"none; canonical stage-2 cloud already uses metres","coordinates_modified":False,"point_count":len(points[role])}
        logger.info("Preconditions passed: stage 2/3 success, human approval recorded, 16 canonical matrices consistent")
        prepared=[]
        sequence=[("gicp",x) for x in cfg["gicp_levels"]]+[("robust_point_to_plane",x) for x in cfg["robust_point_to_plane_levels"]]
        for index,(algorithm,level) in enumerate(sequence):
            source,sr=prepare_balanced_level(clouds["source_board"],clouds["source_object"],level["voxel_size_m"],cfg["sampling"],cfg["normal_estimation"],cfg["covariance_estimation"],int(cfg["random_seed"])+index*101)
            target,tr=prepare_balanced_level(clouds["target_board"],clouds["target_object"],level["voxel_size_m"],cfg["sampling"],cfg["normal_estimation"],cfg["covariance_estimation"],int(cfg["random_seed"])+index*101)
            level=dict(level);level["gicp_epsilon"]=cfg["covariance_estimation"]["gicp_epsilon"]
            prepared.append((algorithm,level,source,target,{"source":sr,"target":tr}))
        peak=max(peak,process.memory_info().rss);refined=[]
        diag=lambda matrix:region_diagnostics(matrix,points["source_board"],points["source_object"],points["target_board"],points["target_object"],
            float(cfg["sanity_gates"]["object_trim_fraction"]),cfg["sanity_gates"]["object_overlap_thresholds_m"])
        for ci,parent in enumerate(parents,1):
            parent_matrix=np.asarray(parent["matrix_m"],float);current=parent_matrix.copy();levels=[];algorithms=[];candidate_errors=[]
            logger.info("Candidate %d/16 %s: starting GICP then robust point-to-plane",ci,parent["candidate_id"])
            for li,(algorithm,level,source,target,sampling) in enumerate(prepared,1):
                if algorithm not in algorithms:algorithms.append(algorithm)
                logger.info("Candidate %d/16 | %s | level %d/3 | voxel=%.1f mm threshold=%.1f mm",ci,algorithm,
                    1+sum(1 for x in levels if x["algorithm"]==algorithm),1000*level["voxel_size_m"],1000*level["max_correspondence_distance_m"])
                try:
                    current,record=run_level(source,target,current,parent_matrix,algorithm,1+sum(1 for x in levels if x["algorithm"]==algorithm),level,
                        cfg["convergence"],cfg["sanity_gates"],diag,points["source_object"],points["target_board"])
                    record["sampling_and_geometry_preparation"]=sampling;levels.append(record)
                except Exception as exc:
                    candidate_errors.append(f"{algorithm}_level_{li}: {type(exc).__name__}: {exc}")
                    levels.append({"algorithm":algorithm,"level_index":li,"parameters":level,"sampling_and_geometry_preparation":sampling,
                        "actual_iterations":0,"maximum_iterations":level["max_iterations"],"stop_reason":"exception","exception":candidate_errors[-1],"trajectory":[],
                        "before_region_metrics":diag(current),"after_region_metrics":diag(current)})
            parent_metrics=diag(parent_matrix);final=diag(current);reasons=candidate_errors+final_sanity(final,parent_metrics,cfg["sanity_gates"])
            rejected=sum(not row["accepted"] for level_record in levels for row in level_record["trajectory"])
            accepted=sum(row["accepted"] for level_record in levels for row in level_record["trajectory"])
            incomplete=bool(candidate_errors) or any(x["stop_reason"] in {"low_correspondence","numerical_failure","exception"} for x in levels)
            status="refinement_failed" if incomplete else ("refined_valid" if accepted and not reasons else "no_safe_refinement")
            if status!="refined_valid":
                rejection_reasons=[reason for level_record in levels for row in level_record["trajectory"] for reason in row["rejection_reasons"]]
                reasons=list(dict.fromkeys(reasons+rejection_reasons+(["no_update_passed_all_constrained_acceptance_gates"] if not accepted else [])))
            if status!="refined_valid":current=parent_matrix.copy();final=parent_metrics
            rid=refined_candidate_id(parent["candidate_id"]);rd=rotation_difference_deg(parent_matrix,current);td=float(np.linalg.norm(current[:3,3]-parent_matrix[:3,3]))
            refined.append({"refined_candidate_id":rid,"parent_candidate_id":parent["candidate_id"],"parent_rank":parent.get("rank"),
                "parent_generator":parent.get("generator"),"parent_generators":parent.get("generators",[parent.get("generator")]),
                "status":status,"failure_reasons":reasons,"algorithms_attempted":algorithms,"accepted_update_count":accepted,"rejected_update_count":rejected,
                "algorithm_order":["gicp","robust_point_to_plane"],"parent_matrix_m":parent_matrix.tolist(),"matrix_m":current.tolist(),
                "parent_metrics":parent_metrics,"final_metrics":final,"from_parent_rotation_deg":rd,"from_parent_translation_m":td,"levels":levels})
            peak=max(peak,process.memory_info().rss)
        if len(refined)!=16:raise Stage4Failure("not all 16 parent candidates were traversed")
        if any(c["algorithms_attempted"]!=["gicp","robust_point_to_plane"] or len(c["levels"])!=6 for c in refined):
            raise Stage4Failure("required GICP -> robust point-to-plane execution sequence is incomplete")
        visualization_candidates=[{**c,**select_visualization_record(c)} for c in refined]
        visualization_manifest={"schema":"pointcloudlab.stage4.visualization_manifest","version":1,"stage":4,"status":"success",
            "diagnostic_only":True,"export_transform_source":False,
            "selection_rule":"refined_valid: formal accepted matrix_m; otherwise: last legal proposed_matrix_m, then last legal current matrix, then parent matrix",
            "note":"Rejected proposals are shown for diagnosis and are not exported transforms.",
            "candidates":[{"refined_candidate_id":c["refined_candidate_id"],"parent_candidate_id":c["parent_candidate_id"],
                "formal_status":c["status"],**{k:c[k] for k in VISUALIZATION_FIELDS},
                "visualization_source_kind":c["visualization_source_kind"]} for c in visualization_candidates]}
        valid=sum(c["status"]=="refined_valid" for c in refined)
        matrix_dir=out/"refined_matrices";matrix_dir.mkdir(parents=True,exist_ok=True)
        for stale in matrix_dir.glob("s4_ref_*.txt"):stale.unlink()
        for c in refined:np.savetxt(matrix_dir/f"{c['refined_candidate_id']}.txt",np.asarray(c["matrix_m"]),fmt="%.17g")
        payload={"schema":"pointcloudlab.stage4.refined_candidates","version":3,**stage4_contract(),"status":"success","unit":"m",
            "coordinate_convention":"p_target = T_FAST_from_INSPIRE_m @ p_source","candidate_count":16,"valid_candidate_count":valid,"candidates":refined}
        write_json(out/"refined_candidates.json",payload);write_json(out/"visualization_manifest.json",visualization_manifest)
        write_summary(out/"refinement_summary.csv",refined);write_iterations(out/"refinement_iterations.csv",refined)
        serialization=validate_outputs(out,refined);visualization=save_stage4_comparison(out/"refinement_comparison.png",visualization_candidates,
            points["source_board"],points["source_object"],points["target_board"],points["target_object"],int(cfg["random_seed"]))
        after=integrity(protected_paths)
        if before!=after:raise Stage4Failure("a protected stage-2/stage-3 input changed during stage 4")
        if any((out/x).exists() for x in ("T_fast_from_inspire_m.txt","registered_inspire_full.ply","merged_fast_inspire.ply")):
            raise Stage4Failure("forbidden final-export artifact detected in stage-4 output")
        elapsed=time.perf_counter()-started
        report.update({"status":"success","software":{"python":sys.version,"platform":platform.platform(),"open3d":o3d.__version__,"numpy":np.__version__,
            "scipy":scipy.__version__,"matplotlib":matplotlib.__version__},"config_path":str(args.config.resolve()),"config":cfg,"cli_overrides":overrides,
            "preconditions":preconditions,"inputs":inputs,"protected_input_integrity":{"before":before,"after":after,"unchanged":True},
            "algorithm_execution":{"required_order":["gicp","robust_point_to_plane"],"single_iteration_compatibility_loop":True,
                "correspondence_policy":"explicit board-to-board and object-to-object; no merged-cloud nearest neighbours",
                "joint_solver":"per-region residuals/weights, equal-region normalized normal equations, one shared SE(3) update","all_candidates_all_levels_attempted":True,
                "gicp_covariance_source":cfg["covariance_estimation"],"robust_loss":{"name":"Huber","scales_m":[x["huber_k_m"] for x in cfg["robust_point_to_plane_levels"]],
                "basis":"2-4 mm: comparable to FAST 1.1 mm robust noise and below/current correspondence thresholds"}},
            "candidate_counts":{"parents_attempted":16,"refined_valid":valid,"no_safe_refinement":sum(c["status"]=="no_safe_refinement" for c in refined),
                "refinement_failed":sum(c["status"]=="refinement_failed" for c in refined)},"backtracking_summary":backtracking_summary(refined),"refined_candidates":refined,
            "serialization_validation":serialization,"visualization":visualization,
            "visualization_candidates":visualization_manifest["candidates"],
            "ambiguity_handling":{"lineages_preserved":True,"deduplication_performed":False,"clustering_or_ranking_performed":False,
                "final_ambiguity_decision_performed":False,"note":"competitive symmetric directions remain available for explicit user selection"},
            "limitations":["Only front faces are observed; physical thickness is unavailable and unused.","FAST object depths contain noise/return fusion.",
                "The planar object remains highly symmetric at 0/90/180/270 degrees.","Sanity gates reject local sliding but are not final ranking scores."],
            "elapsed_s":elapsed,"peak_memory_bytes":peak,"peak_memory_measurement":"maximum process RSS polled after preparation and every candidate",
            "artifacts":{"refined_candidates":str((out/"refined_candidates.json").resolve()),"summary_csv":str((out/"refinement_summary.csv").resolve()),
                "iterations_csv":str((out/"refinement_iterations.csv").resolve()),"matrix_directory":str(matrix_dir.resolve()),
                "visualization_manifest":str((out/"visualization_manifest.json").resolve()),
                "comparison":str((out/"refinement_comparison.png").resolve())}})
        write_json(out/"refinement_report.json",report);logger.info("Stage 4 success: attempted=16 refined_valid=%d no_safe=%d failed=%d elapsed=%.2fs peak=%.1f MiB",
            valid,sum(c["status"]=="no_safe_refinement" for c in refined),sum(c["status"]=="refinement_failed" for c in refined),elapsed,peak/2**20);return 0
    except Exception as exc:
        report["failure_reasons"].append(f"{type(exc).__name__}: {exc}");report["elapsed_s"]=time.perf_counter()-started;report["peak_memory_bytes"]=peak
        write_json(out/"refinement_report.json",report);logger.exception("Stage 4 failed: %s",exc);return 2


if __name__=="__main__":raise SystemExit(main())

