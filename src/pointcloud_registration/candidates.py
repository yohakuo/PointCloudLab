"""Auditable geometry-based coarse-candidate generation for stage 3."""

from __future__ import annotations

import hashlib
import json
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from .transforms import (apply_transform, compose_plane_transform, rotation_difference_deg,
                         transform_difference, validate_rigid_transform)


def canonical_fingerprint(value: Any, length: int = 12) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode()).hexdigest()[:length]


def candidate_id(generator: str, provenance: dict[str, Any]) -> str:
    if generator != "geometry":
        raise ValueError(f"unsupported candidate generator: {generator}")
    return f"s3_geo_{canonical_fingerprint({'generator': generator, 'provenance': provenance})}"


def stage3_contract() -> dict[str, Any]:
    return {"stage": 3, "coarse_registration_performed": True, "coarse_transforms_estimated": True,
            "refinement_performed": False, "final_transform_selected": False,
            "full_cloud_transformed": False, "transform_direction": "INSPIRE_TO_FAST"}


def deterministic_sample(points: np.ndarray, limit: int, seed: int) -> np.ndarray:
    if len(points) <= limit: return np.asarray(points).copy()
    rng = np.random.default_rng(seed)
    return np.asarray(points)[np.sort(rng.choice(len(points), limit, replace=False))].copy()


def _trimmed_mean(values: np.ndarray, fraction: float) -> float:
    if not len(values): return float("inf")
    n = max(1, int(np.ceil(len(values)*fraction)))
    return float(np.mean(np.partition(values, n-1)[:n]))


def coarse_geometry_checks(matrix: np.ndarray, source_board_plane: np.ndarray, target_board_plane: np.ndarray,
                           source_object: np.ndarray, target_object: np.ndarray, cfg: dict[str, Any]) -> dict[str, Any]:
    rigid = validate_rigid_transform(matrix)
    if not rigid["valid"]:
        return {"passed": False, "rigid": rigid, "reasons": list(rigid["reasons"])}
    r, t = matrix[:3, :3], matrix[:3, 3]
    moved_normal = r @ source_board_plane[:3]
    angle = float(np.degrees(np.arccos(np.clip(abs(moved_normal @ target_board_plane[:3]), 0, 1))))
    src_anchor = -source_board_plane[3] * source_board_plane[:3]
    plane_distance = abs(float(target_board_plane[:3] @ (r @ src_anchor + t) + target_board_plane[3]))
    limit = int(cfg["diagnostic_sample_limit"])
    src = deterministic_sample(source_object, limit, 7103); tgt = deterministic_sample(target_object, limit, 9109)
    moved = apply_transform(src, matrix)
    d_st = cKDTree(tgt).query(moved, workers=1)[0]; d_ts = cKDTree(moved).query(tgt, workers=1)[0]
    trim = float(cfg["trim_fraction"])
    chamfer = .5*(_trimmed_mean(d_st, trim)+_trimmed_mean(d_ts, trim))
    overlap = .5*(float(np.mean(d_st <= .015))+float(np.mean(d_ts <= .015)))
    amin, amax = moved.min(0), moved.max(0); bmin, bmax = tgt.min(0), tgt.max(0)
    gap = np.maximum(np.maximum(bmin-amax, amin-bmax), 0); disconnected = float(np.linalg.norm(gap)) > float(cfg["aabb_disconnection_margin_m"])
    reasons=[]
    if angle > float(cfg["maximum_board_normal_angle_deg"]): reasons.append("board_normal_angle_exceeds_wide_gate")
    if plane_distance > float(cfg["maximum_board_plane_distance_m"]): reasons.append("board_plane_distance_exceeds_wide_gate")
    if chamfer > float(cfg["maximum_object_trimmed_chamfer_m"]): reasons.append("object_trimmed_chamfer_exceeds_wide_gate")
    if overlap < float(cfg["minimum_object_overlap_ratio_at_15mm"]): reasons.append("object_overlap_below_wide_gate")
    if disconnected: reasons.append("transformed_object_aabb_completely_disconnected")
    return {"passed": not reasons, "reasons": reasons, "rigid": rigid,
            "board": {"normal_angle_deg": angle, "plane_distance_m": plane_distance},
            "object": {"bidirectional_trimmed_chamfer_m": chamfer, "bidirectional_overlap_at_15mm": overlap},
            "aabb": {"gap_norm_m": float(np.linalg.norm(gap)), "completely_disconnected": bool(disconnected)}}


def contour_angle(source_uv: np.ndarray, target_uv: np.ndarray, cfg: dict[str, Any]) -> tuple[float, dict[str, Any]]:
    count=int(cfg["contour_sample_count"]); s=deterministic_sample(source_uv, count, 123); t=deterministic_sample(target_uv, count, 456)
    s=s-np.median(s,axis=0); t=t-np.median(t,axis=0)
    angles=np.arange(-45.,45.+1e-9,float(cfg["contour_angle_step_deg"])); scores=[]
    for a in angles:
        q=np.deg2rad(a); rr=np.array([[np.cos(q),-np.sin(q)],[np.sin(q),np.cos(q)]])
        moved=s@rr.T; d1=cKDTree(t).query(moved,workers=1)[0]; d2=cKDTree(moved).query(t,workers=1)[0]
        scores.append(.5*(_trimmed_mean(d1,.8)+_trimmed_mean(d2,.8)))
    best=int(np.argmin(scores))
    return float(angles[best]), {"method":"centered bidirectional trimmed contour distance angular search",
        "range_deg":[-45.,45.],"step_deg":float(cfg["contour_angle_step_deg"]),"best_distance_m":float(scores[best])}


def translation_hypotheses(source_uv: np.ndarray, target_uv: np.ndarray, angle_deg: float,
                           cfg: dict[str, Any]) -> list[dict[str, Any]]:
    q=np.deg2rad(angle_deg); rr=np.array([[np.cos(q),-np.sin(q)],[np.sin(q),np.cos(q)]])
    rotated=source_uv@rr.T
    robust=np.median(target_uv,axis=0)-np.median(rotated,axis=0)
    obb=.5*(np.percentile(target_uv,2,axis=0)+np.percentile(target_uv,98,axis=0))-.5*(np.percentile(rotated,2,axis=0)+np.percentile(rotated,98,axis=0))
    result=[{"method":"robust_centroid","shift_uv_m":robust},{"method":"obb_center","shift_uv_m":obb}]
    step=float(cfg["search_step_m"]); window=float(cfg["search_window_m"]); trim=float(cfg["search_trim_fraction"])
    src=deterministic_sample(rotated,int(cfg["contour_sample_count"]),811); tgt=deterministic_sample(target_uv,int(cfg["contour_sample_count"]),977)
    scored=[]
    for du in np.arange(-window,window+step/2,step):
        for dv in np.arange(-window,window+step/2,step):
            shift=robust+np.array([du,dv]); moved=src+shift
            score=.5*(_trimmed_mean(cKDTree(tgt).query(moved,workers=1)[0],trim)+_trimmed_mean(cKDTree(moved).query(tgt,workers=1)[0],trim))
            scored.append((score,shift))
    picked=[]
    for score,shift in sorted(scored,key=lambda x:(x[0],x[1][0],x[1][1])):
        if all(np.linalg.norm(shift-p[1])>=float(cfg["search_nms_distance_m"]) for p in picked):
            picked.append((score,shift))
        if len(picked)>=int(cfg["search_peaks_per_direction"]): break
    result.extend({"method":"contour_search","shift_uv_m":shift,"search_score_m":score,"peak_index":i} for i,(score,shift) in enumerate(picked))
    return result


def generate_geometry_candidates(source_frame: dict[str, Any], target_frame: dict[str, Any],
                                 source_object: np.ndarray, target_object: np.ndarray,
                                 cfg: dict[str, Any], gate_cfg: dict[str, Any], input_hashes: dict[str,str],
                                 config_fingerprint: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    correction, contour_report=contour_angle(source_frame["uv"],target_frame["uv"],cfg)
    bases=[("pca_obb",0.,{"method":"2D PCA/OBB axis alignment"}),("contour",correction,contour_report)]
    candidates=[]
    for base_name,base_angle,base_diag in bases:
        for increment in (0,90,180,270):
            angle=base_angle+increment
            for hyp in translation_hypotheses(source_frame["uv"],target_frame["uv"],angle,cfg):
                provenance={"parent_branch":base_name,"continuous_base_angle_deg":base_angle,"discrete_increment_deg":increment,
                            "relative_angle_deg":angle,"translation_method":hyp["method"],"translation_peak_index":hyp.get("peak_index")}
                provenance["normal_alignment_branch"] = target_frame.get("normal_branch", "observed_object_side")
                matrix=compose_plane_transform(source_frame,target_frame,angle,np.asarray(hyp["shift_uv_m"]))
                checks=coarse_geometry_checks(matrix,source_frame["plane"],target_frame["plane"],source_object,target_object,gate_cfg)
                cid=candidate_id("geometry",provenance)
                candidates.append({"candidate_id":cid,"generator":"geometry","source_role":"INSPIRE_2","target_role":"FAST_LIVO2",
                    "transform_direction":"INSPIRE_TO_FAST","matrix_m":matrix.tolist(),"provenance":provenance,
                    "generation_parameters":{**hyp,"base_diagnostics":base_diag},"coarse_checks":checks,
                    "status":"retained_raw" if checks["passed"] else "rejected","status_reasons":checks["reasons"],
                    "input_hashes":input_hashes,"config_fingerprint":config_fingerprint})
    return candidates,{"continuous_bases":bases,"contour":contour_report,"enumerated_increments_deg":[0,90,180,270]}


def deduplicate_candidates(candidates: list[dict[str, Any]], rotation_threshold_deg: float,
                           translation_threshold_m: float, max_candidates: int,
                           minimum_per_generator: int = 0) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    valid=[c for c in candidates if c.get("status") == "retained_raw"]
    valid.sort(key=lambda c:(float(c.get("coarse_rank_score",float("inf"))),c["candidate_id"]))
    clusters=[]
    for candidate in valid:
        matrix=np.asarray(candidate["matrix_m"]); matched=None
        for cluster in clusters:
            rd,td=transform_difference(np.asarray(cluster["representative"]["matrix_m"]),matrix)
            if rd<=rotation_threshold_deg and td<=translation_threshold_m: matched=cluster; break
        if matched is None: clusters.append({"representative":candidate,"members":[candidate]})
        else: matched["members"].append(candidate)
    selected_indices: list[int] = []
    for generator in ("geometry",):
        eligible = [i for i, cluster in enumerate(clusters)
                    if any(m["generator"] == generator for m in cluster["members"])]
        selected_indices.extend(i for i in eligible[:minimum_per_generator] if i not in selected_indices)
    selected_indices.extend(i for i in range(len(clusters)) if i not in selected_indices)
    selected_set = set(selected_indices[:max_candidates])
    selected_clusters = [clusters[i] for i in selected_indices[:max_candidates]]
    selected_clusters.sort(key=lambda x: (float(x["representative"].get("coarse_rank_score", float("inf"))),
                                          x["representative"]["candidate_id"]))
    canonical=[]; audit=[]
    for cluster in selected_clusters:
        rep=cluster["representative"]; members=sorted(cluster["members"],key=lambda c:c["candidate_id"])
        cid="s3_can_"+canonical_fingerprint([m["candidate_id"] for m in members])
        generators=sorted({m["generator"] for m in members}); seeds=sorted({m["provenance"].get("seed") for m in members if m["provenance"].get("seed") is not None})
        item={**rep,"candidate_id":cid,"representative_raw_candidate_id":rep["candidate_id"],
              "member_candidate_ids":[m["candidate_id"] for m in members],"support_count":len(members),
              "generators":generators,"cross_generator_support":len(generators)>1,"support_seeds":seeds,"status":"canonical"}
        canonical.append(item)
    for index, cluster in enumerate(clusters):
        rep=cluster["representative"]; members=sorted(cluster["members"],key=lambda c:c["candidate_id"])
        cid="s3_can_"+canonical_fingerprint([m["candidate_id"] for m in members])
        for member in members:
            member["deduplication_status"] = ("canonical_representative" if index in selected_set and member is rep
                                                 else "merged_duplicate" if index in selected_set
                                                 else "valid_cluster_outside_total_limit")
            member["deduplication_reason"] = f"SE3 cluster {cid}; deterministic representative {rep['candidate_id']}"
        generators=sorted({m["generator"] for m in members});seeds=sorted({m["provenance"].get("seed") for m in members if m["provenance"].get("seed") is not None})
        audit.append({"canonical_candidate_id":cid if index in selected_set else None,"cluster_id":cid,"selected_for_stage4":index in selected_set,
                      "representative_raw_candidate_id":rep["candidate_id"],"member_candidate_ids":[m["candidate_id"] for m in members],
                      "generators":generators,"support_seeds":seeds})
    for rank,item in enumerate(canonical,1): item["rank"]=rank
    return canonical,audit


def assign_coarse_rank(candidates: list[dict[str, Any]]) -> None:
    for c in candidates:
        checks=c.get("coarse_checks",{}); obj=checks.get("object",{}); board=checks.get("board",{})
        c["coarse_rank_score"]=float(obj.get("bidirectional_trimmed_chamfer_m",1e3))+float(board.get("plane_distance_m",1e3))+float(board.get("normal_angle_deg",180))/1800
        c["coarse_rank_basis"]="equal diagnostic terms: object trimmed distance + board plane distance + board angle/1800; not final scoring"
