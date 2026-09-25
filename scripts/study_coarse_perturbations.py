#!/usr/bin/env python3
"""Paired, full coarse-registration perturbation experiment. No stage-4 handoff."""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import os
import platform
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import scipy
from scipy.optimize import linear_sum_assignment
from scipy.spatial import ConvexHull, cKDTree

from pointcloud_registration.candidates import (assign_coarse_rank, deduplicate_candidates,
    generate_geometry_candidates, canonical_fingerprint)
from pointcloud_registration.io_utils import load_metric_stage2_cloud, sha256_file
from pointcloud_registration.model_prior import fit_supported_face, score_supported_face, combine_score
from pointcloud_registration.shape_grid import build_shape_grid, shape_score
from pointcloud_registration.transforms import compose_plane_transform, right_handed_plane_frame, rotation_difference_deg

spec = importlib.util.spec_from_file_location("stage3_runner", ROOT / "scripts/03_generate_candidates.py")
stage3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stage3)
METHODS = ("sampled_random", "grid_sequential", "grid_joint", "joint_rescore", "model_in_search")
ROLES = ("source_board", "source_object", "target_board", "target_object")


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    def convert(item):
        if isinstance(item, np.ndarray): return item.tolist()
        if isinstance(item, np.generic): return item.item()
        raise TypeError(f"Cannot serialize {type(item).__name__}")
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False, default=convert), encoding="utf-8")


def hash_points(points):
    a = np.ascontiguousarray(points, dtype="<f8")
    return hashlib.sha256(np.asarray(a.shape, dtype="<i8").tobytes() + a.tobytes()).hexdigest()


def voxel_resample(points, width, seed):
    if width <= 0: return points.copy()
    rng = np.random.default_rng(seed)
    phase = rng.uniform(0, width, 3)
    cell = np.floor((points + phase) / width).astype(np.int64)
    _, inverse = np.unique(cell, axis=0, return_inverse=True)
    count = np.bincount(inverse)
    centers = np.column_stack([np.bincount(inverse, weights=points[:, j]) / count for j in range(3)])
    return centers


def hull_signed_distance(points_uv, hull_vertices):
    polygon = hull_vertices
    edges = np.roll(polygon, -1, axis=0) - polygon
    cross = edges[:, 0] * (points_uv[:, None, 1] - polygon[None, :, 1]) - edges[:, 1] * (points_uv[:, None, 0] - polygon[None, :, 0])
    inside = np.all(cross >= -1e-10, axis=1)
    length2 = np.sum(edges ** 2, axis=1)
    projection = np.clip(np.sum((points_uv[:, None, :] - polygon[None, :, :]) * edges[None, :, :], axis=2) / length2, 0, 1)
    nearest = polygon[None, :, :] + projection[:, :, None] * edges[None, :, :]
    distance = np.sqrt(np.min(np.sum((points_uv[:, None, :] - nearest) ** 2, axis=2), axis=1))
    return np.where(inside, -distance, distance)


def boundary_perturb(object_points, roi_points, frame, band, grow, tolerance):
    if band <= 0: return object_points.copy(), {"removed": 0, "added": 0}
    origin, basis = frame["origin"], frame["basis"][:, :2]
    uv = (object_points - origin) @ basis
    hull = ConvexHull(uv)
    polygon = uv[hull.vertices]
    signed = hull_signed_distance(uv, polygon)
    if grow:
        # Restrict the candidate ROI to the object's three-dimensional AABB before
        # testing its boundary; all returned points are existing ROI observations.
        low, high = object_points.min(0) - tolerance, object_points.max(0) + tolerance
        nearby = roi_points[np.all((roi_points >= low) & (roi_points <= high), axis=1)]
        candidates = nearby[(hull_signed_distance((nearby - origin) @ basis, polygon) > 0) &
                            (hull_signed_distance((nearby - origin) @ basis, polygon) <= band)]
        if len(candidates):
            candidates = candidates[cKDTree(object_points).query(candidates, workers=1)[0] <= tolerance]
        # Deduplicate exact observed coordinates; do not synthesize boundary points.
        combined = np.unique(np.vstack((object_points, candidates)), axis=0)
        return combined, {"removed": 0, "added": int(len(combined) - len(object_points))}
    kept = object_points[signed <= -band]
    return kept, {"removed": int(len(object_points) - len(kept)), "added": 0}


def make_manifest(config):
    if config["combined_components"] != ["boundary", "spatial", "board"]:
        raise ValueError("combined_components must match the implemented boundary, spatial, board order")
    if config["boundary_direction_rule"] != "even_repeat_erodes_odd_repeat_dilates":
        raise ValueError("unsupported boundary_direction_rule")
    rows = [{"id": "reference", "group": "reference", "strength": "none", "repeat": 0,
             "seed": config["seed"] - 1, "components": []}]
    for group in config["groups"]:
        for strength in config["strengths"]:
            for rep in range(config["maximum_repeats"]):
                components = [group] if group != "combined" else config["combined_components"]
                rows.append({"id": f"{group}_{strength}_{rep:02d}", "group": group,
                             "strength": strength, "repeat": rep,
                             "seed": int(config["seed"] + 10000 * list(config["groups"]).index(group)
                                         + 1000 * list(config["strengths"]).index(strength) + rep),
                             "components": components,
                             "boundary_rule": "even repeat erodes; odd repeat dilates observed ROI support"})
    return rows


def perturb(original, rois, frames, row, config):
    if row["group"] == "reference":
        return {k: v.copy() for k, v in original.items()}, {}
    strength = config["strengths"][row["strength"]]
    points = {k: v.copy() for k, v in original.items()}
    audit = {}
    if "boundary" in row["components"]:
        for side in ("source", "target"):
            key = side + "_object"
            points[key], audit[key + "_boundary"] = boundary_perturb(
                points[key], rois[side], frames[side], strength[side + "_boundary_m"],
                row["repeat"] % 2 == 1, config["boundary_addition_3d_tolerance_m"])
    if "spatial" in row["components"]:
        for i, key in enumerate(("source_object", "target_object")):
            width = strength[("source" if key.startswith("source") else "target") + "_voxel_m"]
            before = len(points[key])
            points[key] = voxel_resample(points[key], width, row["seed"] + 37 * i)
            audit[key + "_spatial"] = {"voxel_m": width, "before": before, "after": len(points[key])}
    if "board" in row["components"]:
        for i, key in enumerate(("source_board", "target_board")):
            before = len(points[key])
            points[key] = voxel_resample(points[key], strength["board_voxel_m"], row["seed"] + 101 + i)
            audit[key + "_board"] = {"voxel_m": strength["board_voxel_m"], "before": before, "after": len(points[key])}
    return points, audit


def plane_gate(report, baseline, cfg):
    current = report["target"]
    old = baseline["target"]
    n = np.asarray(current["oriented_plane_model"][:3])
    n0 = np.asarray(old["oriented_plane_model"][:3])
    angle = float(np.degrees(np.arccos(np.clip(abs(n @ n0), 0, 1))))
    coverage = current["coverage"]["robust_area_m2"] / old["coverage"]["robust_area_m2"]
    reasons = []
    if current["plane_inlier_ratio"] < cfg["minimum_target_inlier_ratio"]: reasons.append("target_board_inlier_ratio")
    if current["absolute_residual_m"]["p90"] > cfg["maximum_target_residual_p90_m"]: reasons.append("target_board_residual_p90")
    if angle > cfg["maximum_target_normal_change_deg"]: reasons.append("target_board_normal_change")
    if coverage < cfg["minimum_target_coverage_fraction"]: reasons.append("target_board_coverage")
    return {"passed": not reasons, "reasons": reasons, "normal_change_deg": angle, "coverage_fraction": coverage}


def planes_and_frames(points, registration, baseline=None, gate_cfg=None):
    pcfg = registration["plane_fit"]
    source_plane, source = stage3.fit_plane_report(points["source_board"], pcfg["source_distance_threshold_m"], pcfg, points["source_object"])
    target_plane, target = stage3.fit_plane_report(points["target_board"], pcfg["target_distance_threshold_m"], pcfg, points["target_object"])
    source["frame"]["plane"] = source_plane
    target["frame"]["plane"] = target_plane
    target["frame"]["normal_branch"] = "observed_object_side"
    reports = {"source": source["report"], "target": target["report"]}
    gate = {"passed": True, "reasons": []} if baseline is None else plane_gate(reports, baseline, gate_cfg)
    target_frames = [target["frame"]]
    if target["report"]["object_side_orientation"]["opposite_alignment_branch_required"]:
        opposite = right_handed_plane_frame(-target_plane, points["target_object"])
        opposite["plane"] = -target_plane
        opposite["normal_branch"] = "opposite_due_to_weak_FAST_sign_evidence"
        target_frames.append(opposite)
    return reports, source["frame"], target_frames, gate


def model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg):
    branch = candidate["provenance"].get("normal_alignment_branch", "observed_object_side")
    if model is None:
        term = {"valid": False, "reason": "source_fit_failed:" + ",".join(fit["reasons"])}
    else:
        frame = next(x for x in target_frames if x["normal_branch"] == branch)
        term = score_supported_face(model, source_frame, frame, grids[branch], candidate["matrix_m"], model_cfg)
    terms = {k: candidate["shape_match"][k] for k in ("occupancy_m", "outer_contour_m", "height_m")}
    combined = combine_score(term, terms, model_cfg)
    return {"model_term": term, "fallback": combined is None,
            "fallback_reason": term.get("reason") if combined is None else None,
            "score_m": candidate["shape_match"]["score_m"] if combined is None else combined}


def one_method(method, points, source_frame, target_frames, registration, model_cfg, input_hashes, roi_bounds,
               frozen_joint_result=None):
    cfg = copy.deepcopy(registration)
    cfg["geometry"]["search_method"] = "joint" if method in ("grid_joint", "joint_rescore", "model_in_search") else "sequential"
    shape_method = "sampled_points" if method == "sampled_random" else "occupancy_grid"
    geometry = cfg["geometry"]
    shape_cfg = {**geometry, **geometry["shape_grid"]}
    source_grid = build_shape_grid(source_frame, points["source_object"], geometry["shape_grid"], roi_bounds["source"]) if shape_method == "occupancy_grid" else None
    fit, model = fit_supported_face(source_grid, model_cfg) if source_grid is not None else ({"valid": False, "reasons": ["method_has_no_grid"]}, None)
    grids = {}
    if method == "joint_rescore" and frozen_joint_result is not None:
        copied = copy.deepcopy({k: frozen_joint_result[k] for k in
            ("raw_candidates", "canonical_candidates", "clusters", "search_audit", "model_fit", "model_fallback_candidates", "scoring_system")})
        for target_frame in target_frames:
            branch = target_frame["normal_branch"]
            grids[branch] = build_shape_grid(target_frame, points["target_object"], geometry["shape_grid"], roi_bounds["target"])
        for candidate in copied["raw_candidates"]:
            if candidate["status"] == "retained_raw":
                candidate["model_assessment"] = model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg)
        for candidate in copied["canonical_candidates"]:
            candidate["model_assessment"] = model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg)
        copied.update({"model_fit": fit, "model_fallback_candidates": sum(
            bool(x.get("model_assessment", {}).get("fallback")) for x in copied["raw_candidates"]),
            "scoring_system": "model_combined_with_grid_fallback",
            "search_pool_shared_with": "grid_joint"})
        return copied
    raw, audits = [], []
    for target_frame in target_frames:
        branch = target_frame["normal_branch"]
        grid = build_shape_grid(target_frame, points["target_object"], geometry["shape_grid"], roi_bounds["target"]) if source_grid is not None else None
        grids[branch] = grid
        score_at_angle = None
        search_model_counter = {"evaluations": 0, "fallback_evaluations": 0}
        if method == "model_in_search":
            def score_at_angle(angle, tf=target_frame, tg=grid):
                def score(shift):
                    search_model_counter["evaluations"] += 1
                    old, terms = shape_score(source_grid, tg, angle, shift, shape_cfg)
                    if model is None:
                        search_model_counter["fallback_evaluations"] += 1
                        return old
                    matrix = compose_plane_transform(source_frame, tf, angle, np.asarray(shift))
                    term = score_supported_face(model, source_frame, tf, tg, matrix, model_cfg)
                    combined = combine_score(term, terms, model_cfg)
                    if combined is None: search_model_counter["fallback_evaluations"] += 1
                    return old if combined is None else combined
                return score
        candidates, audit = generate_geometry_candidates(source_frame, target_frame,
            points["source_object"], points["target_object"], shape_cfg, cfg["coarse_gates"],
            input_hashes, canonical_fingerprint(cfg), source_grid, grid, score_at_angle)
        raw.extend(candidates)
        audits.append({"normal_branch": branch, **audit, "model_search": search_model_counter})
    assign_coarse_rank(raw)
    if method in ("joint_rescore", "model_in_search"):
        for candidate in raw:
            if candidate["status"] != "retained_raw": continue
            assessment = model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg)
            candidate["model_assessment"] = assessment
            if method == "model_in_search": candidate["coarse_rank_score"] = assessment["score_m"]
    canonical, clusters = deduplicate_candidates(raw, cfg["deduplication"]["rotation_threshold_deg"],
        cfg["deduplication"]["translation_threshold_m"], cfg["deduplication"]["maximum_canonical_candidates"],
        cfg["deduplication"]["minimum_per_generator"], preserve_direction_branches=geometry["search_method"] == "joint")
    # Rescoring acts on the frozen grid-joint pool and its frozen canonical representatives.
    if method == "joint_rescore":
        for candidate in canonical:
            candidate["model_assessment"] = model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg)
    if method == "model_in_search":
        for candidate in canonical:
            candidate["model_assessment"] = model_assessment(candidate, fit, model, source_frame, target_frames, grids, model_cfg)
    return {"raw_candidates": raw, "canonical_candidates": canonical, "clusters": clusters,
            "search_audit": audits, "model_fit": fit, "model_fallback_candidates": sum(
                bool(x.get("model_assessment", {}).get("fallback")) for x in raw),
            "scoring_system": "model_combined_with_grid_fallback" if method in ("joint_rescore", "model_in_search") else
                              ("sampled_projection" if method == "sampled_random" else "grid_shape")}


def normal_sign(matrix, source_normal, target_normal):
    return 1 if (np.asarray(matrix)[:3, :3] @ source_normal) @ target_normal >= 0 else -1


def match_branches(reference, candidates, source_normal, target_normal, maximum_angle):
    """One-to-one assignment by actual SO(3) distance, separately per normal sign."""
    assignment = {}
    for sign in (-1, 1):
        refs = [(i, c) for i, c in enumerate(reference) if normal_sign(c["matrix_m"], source_normal, target_normal) == sign]
        runs = [(i, c) for i, c in enumerate(candidates) if normal_sign(c["matrix_m"], source_normal, target_normal) == sign]
        if not refs or not runs: continue
        cost = np.asarray([[rotation_difference_deg(np.asarray(a["matrix_m"]), np.asarray(b["matrix_m"]))
                            for _, b in runs] for _, a in refs])
        rr, cc = linear_sum_assignment(cost)
        for a, b in zip(rr, cc):
            if cost[a, b] <= maximum_angle:
                assignment[refs[a][0]] = {"candidate_index": runs[b][0], "rotation_distance_deg": float(cost[a, b]), "normal_sign": sign}
    return assignment


def direction_representatives(candidates, method, source_normal, target_normal, cluster_angle):
    """Keep the best scored pose per actual rotation cluster, including normal sign."""
    selected = []
    for candidate in sorted(candidates, key=lambda c: (score_of(c, method), canonical_fingerprint(c["matrix_m"]))):
        sign = normal_sign(candidate["matrix_m"], source_normal, target_normal)
        if any(normal_sign(other["matrix_m"], source_normal, target_normal) == sign and
               rotation_difference_deg(np.asarray(candidate["matrix_m"]), np.asarray(other["matrix_m"])) <= cluster_angle
               for other in selected):
            continue
        selected.append(candidate)
    return selected


def project_fixed(matrix, source_point, fixed_frame):
    xyz = np.asarray(matrix)[:3, :3] @ source_point + np.asarray(matrix)[:3, 3]
    relative = xyz - fixed_frame["origin"]
    return [float(relative @ fixed_frame["u"]), float(relative @ fixed_frame["v"])]


def score_of(candidate, method):
    if method in ("joint_rescore", "model_in_search"):
        return candidate["model_assessment"]["score_m"]
    return candidate["coarse_rank_score"]


def compact_run(method, result, reference, point, fixed_frame, source_normal, target_normal, matching):
    original_candidates = result["canonical_candidates"]
    reference = direction_representatives(reference, method, source_normal, target_normal, matching.get("orientation_cluster_deg", 35.0))
    candidates = direction_representatives(original_candidates, method, source_normal, target_normal, matching.get("orientation_cluster_deg", 35.0))
    matched = match_branches(reference, candidates, source_normal, target_normal, matching["maximum_rotation_distance_deg"])
    rows = []
    for ref_index, ref in enumerate(reference):
        match = matched.get(ref_index)
        if match is None:
            rows.append({"branch": ref_index, "present": False,
                         "reference_matrix_m": ref["matrix_m"],
                         "reference_xy_m": project_fixed(ref["matrix_m"], point, fixed_frame)})
            continue
        candidate = candidates[match["candidate_index"]]
        xy = project_fixed(candidate["matrix_m"], point, fixed_frame)
        reference_xy = project_fixed(ref["matrix_m"], point, fixed_frame)
        rows.append({"branch": ref_index, "present": True, "candidate_index": match["candidate_index"],
                     "reference_matrix_m": ref["matrix_m"], "reference_xy_m": reference_xy,
                     "xy_m": xy, "offset_m": float(np.linalg.norm(np.asarray(xy) - reference_xy)),
                     "rotation_distance_deg": match["rotation_distance_deg"], "normal_sign": match["normal_sign"],
                     "score_m": score_of(candidate, method), "grid_score_m": (candidate.get("shape_match") or {}).get("score_m"),
                     "score_terms": candidate.get("shape_match"), "model_assessment": candidate.get("model_assessment"),
                     "matrix_m": candidate["matrix_m"], "boundary_hit": bool(candidate.get("generation_parameters", {}).get("translation_boundary")),
                     "point_chamfer_m": candidate["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"]})
    ordered = sorted((x for x in rows if x["present"]), key=lambda x: (x["score_m"], x["branch"]))
    gap = ordered[1]["score_m"] - ordered[0]["score_m"] if len(ordered) > 1 else None
    reference_winner = min(range(len(reference)), key=lambda i: (score_of(reference[i], method), i))
    reference_winner_xy = np.asarray(project_fixed(reference[reference_winner]["matrix_m"], point, fixed_frame))
    winner_xy = np.asarray(ordered[0]["xy_m"]) if ordered else None
    pair_distances = [float(np.linalg.norm(np.asarray(a["xy_m"]) - b["xy_m"]))
                      for i, a in enumerate(ordered) for b in ordered[i+1:]]
    return {"branches": rows, "winner_branch": ordered[0]["branch"] if ordered else None,
            "first_second_gap_m": gap, "near_tie": gap is not None and gap <= matching["near_tie_gap_m"],
            "reference_winner_branch": reference_winner,
            "winner_switched_from_reference": bool(ordered and ordered[0]["branch"] != reference_winner),
            "winner_position_shift_from_reference_m": float(np.linalg.norm(winner_xy - reference_winner_xy)) if ordered else None,
            "within_run_max_cross_direction_position_m": max(pair_distances) if pair_distances else None,
            "branch_missing_count": len(reference) - len(matched),
            "candidate_coverage": {"raw": len(result["raw_candidates"]), "canonical": len(original_candidates),
                                   "orientation_clusters": len(candidates),
                                   "matched_reference": len(matched), "reference": len(reference)},
            "boundary_hits": sum(x["boundary_hit"] for x in rows if x["present"]),
            "model_fallback_candidates": result["model_fallback_candidates"]}


def quantiles(values):
    return {"p05": float(np.percentile(values, 5)), "p50": float(np.percentile(values, 50)),
            "p90": float(np.percentile(values, 90)), "p95": float(np.percentile(values, 95))} if values else None


def summarize(records, manifest, methods, convergence):
    summaries, trends = [], []
    cells = sorted({(m["group"], m["strength"]) for m in manifest if m["group"] != "reference"})
    for method in methods:
        for group, strength in cells:
            subset = [r for r in records if r["method"] == method and r["group"] == group and r["strength"] == strength]
            valid = [r for r in subset if r["status"] == "success"]
            winners = [r["analysis"]["winner_branch"] for r in valid if r["analysis"]["winner_branch"] is not None]
            gap = [r["analysis"]["first_second_gap_m"] for r in valid if r["analysis"]["first_second_gap_m"] is not None]
            row = {"method": method, "group": group, "strength": strength, "attempts": len(subset), "valid_runs": len(valid),
                   "failure_rate": (len(subset) - len(valid)) / len(subset) if subset else None,
                   "model_fit_failures": sum(not r.get("model_fit", {}).get("valid", False) for r in subset if r.get("model_fit") and method != "sampled_random"),
                   "model_fit_supported_cells": quantiles([r["model_fit"]["supported_cell_count"] for r in valid if r.get("model_fit", {}).get("valid")]),
                   "model_fit_residual_p90_m": quantiles([r["model_fit"]["residual_p90_m"] for r in valid if r.get("model_fit", {}).get("valid")]),
                   "model_fallback_candidates": sum(r.get("analysis", {}).get("model_fallback_candidates", 0) for r in valid),
                   "model_fallback_runs": sum(r.get("analysis", {}).get("model_fallback_candidates", 0) > 0 for r in valid),
                   "model_fallback_fraction_of_valid": sum(r.get("analysis", {}).get("model_fallback_candidates", 0) > 0 for r in valid)/len(valid) if valid else None,
                   "model_search_fallback_evaluations": sum(a.get("model_search", {}).get("fallback_evaluations", 0)
                       for r in valid for a in r.get("search_audit", [])) if method == "model_in_search" else None,
                   "branch_missing_runs": sum(r["analysis"]["branch_missing_count"] > 0 for r in valid),
                   "boundary_hit_runs": sum(r["analysis"]["boundary_hits"] > 0 for r in valid),
                   "coarse_search_boundary_hit_runs": sum(any(p.get("translation_boundary") for a in r.get("search_audit", []) for p in a.get("angle_profile", [])) for r in valid),
                   "raw_candidate_count": quantiles([r["analysis"]["candidate_coverage"]["raw"] for r in valid]),
                   "canonical_candidate_count": quantiles([r["analysis"]["candidate_coverage"]["canonical"] for r in valid]),
                   "orientation_cluster_count": quantiles([r["analysis"]["candidate_coverage"]["orientation_clusters"] for r in valid]),
                   "near_tie_runs": sum(r["analysis"]["near_tie"] for r in valid),
                   "near_tie_fraction_of_valid": sum(r["analysis"]["near_tie"] for r in valid) / len(valid) if valid else None,
                   "winner_counts": dict(Counter(winners)), "winner_frequency_denominator": len(winners),
                   "winner_switches_ordered_by_repeat": sum(a != b for a, b in zip(winners, winners[1:])),
                   "winner_switches_from_reference": sum(r["analysis"]["winner_switched_from_reference"] for r in valid),
                   "winner_position_shift_from_reference_m": quantiles([r["analysis"]["winner_position_shift_from_reference_m"] for r in valid if r["analysis"]["winner_position_shift_from_reference_m"] is not None]),
                   "cross_direction_position_spread_m": quantiles([r["analysis"]["within_run_max_cross_direction_position_m"] for r in valid if r["analysis"]["within_run_max_cross_direction_position_m"] is not None]),
                   "switch_only_winner_position_shift_m": quantiles([r["analysis"]["winner_position_shift_from_reference_m"] for r in valid if r["analysis"]["winner_switched_from_reference"]]),
                   "first_second_gap_m": quantiles(gap), "elapsed_s": quantiles([r["elapsed_s"] for r in valid])}
            branches = []
            indices = sorted({x["branch"] for r in valid for x in r["analysis"]["branches"]})
            for i in indices:
                present = [next(x for x in r["analysis"]["branches"] if x["branch"] == i) for r in valid]
                present = [x for x in present if x["present"]]
                xy = np.asarray([x["xy_m"] for x in present])
                branches.append({"branch": i, "valid_run_denominator": len(valid), "present_runs": len(present),
                                 "survival_fraction": len(present) / len(valid) if valid else None,
                                 "x_m": quantiles(xy[:, 0].tolist()) if len(xy) else None,
                                 "y_m": quantiles(xy[:, 1].tolist()) if len(xy) else None,
                                 "offset_m": quantiles([x["offset_m"] for x in present]),
                                 "angle_deg": quantiles([x["rotation_distance_deg"] for x in present]),
                                 "score_m": quantiles([x["score_m"] for x in present]),
                                 "score_gap_from_run_best_m": quantiles([x["score_m"] - min(y["score_m"] for y in r["analysis"]["branches"] if y["present"])
                                     for r in valid for x in r["analysis"]["branches"] if x["branch"] == i and x["present"]])})
            row["branches"] = branches
            summaries.append(row)
            ordered = sorted(valid, key=lambda r: r["repeat"])
            for n in range(convergence["batch_size"], len(ordered) + 1, convergence["batch_size"]):
                chunk = ordered[:n]
                offsets = [x["offset_m"] for r in chunk for x in r["analysis"]["branches"] if x["present"]]
                win = Counter(r["analysis"]["winner_branch"] for r in chunk)
                trends.append({"method": method, "group": group, "strength": strength, "valid_n": n,
                               "position_p90_m": float(np.percentile(offsets, 90)) if offsets else None,
                               "winner_frequency": {str(k): v/n for k, v in win.items()},
                               "near_tie_fraction": sum(r["analysis"]["near_tie"] for r in chunk)/n})
    return summaries, trends


def convergence_review(trends, config, summary=None):
    groups = {}
    for row in trends:
        groups.setdefault((row["method"], row["group"], row["strength"]), []).append(row)
    for row in summary or []:
        groups.setdefault((row["method"], row["group"], row["strength"]), [])
    output = []
    tolerance = config["convergence"]
    for key, rows in sorted(groups.items()):
        rows.sort(key=lambda r: r["valid_n"])
        status = "insufficient_batches"
        changes = None
        if len(rows) >= 2:
            a, b = rows[-2:]
            branches = set(a["winner_frequency"]) | set(b["winner_frequency"])
            changes = {"position_p90_m": abs(a["position_p90_m"] - b["position_p90_m"]),
                "winner_frequency": max(abs(a["winner_frequency"].get(x, 0) - b["winner_frequency"].get(x, 0)) for x in branches),
                "near_tie_fraction": abs(a["near_tie_fraction"] - b["near_tie_fraction"])}
            status = "within_configured_tolerances" if (
                changes["position_p90_m"] <= tolerance["position_p90_tolerance_m"] and
                changes["winner_frequency"] <= tolerance["winner_frequency_tolerance"] and
                changes["near_tie_fraction"] <= tolerance["tie_frequency_tolerance"]) else "not_converged"
        output.append({"method": key[0], "group": key[1], "strength": key[2],
                       "status": status, "last_valid_n": rows[-1]["valid_n"] if rows else 0,
                       "last_two_batch_changes": changes,
                       "limitation": "Four repeats yield coarse frequency resolution; convergence is empirical, not a confidence guarantee."})
    return output


def write_csv(path, rows):
    if not rows: return
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def candidate_pool_comparison(records):
    indexed = {(r["id"], r["method"]): r for r in records}
    result = []
    for pair_id in sorted({r["id"] for r in records}):
        grid = indexed.get((pair_id, "grid_joint"))
        if grid is None or grid["status"] != "success": continue
        original = {hash_points(np.asarray(c["matrix_m"])) for c in grid["canonical_candidates"]}
        for method in ("joint_rescore", "model_in_search"):
            record = indexed.get((pair_id, method))
            if record is None or record["status"] != "success": continue
            comparison = {hash_points(np.asarray(c["matrix_m"])) for c in record["canonical_candidates"]}
            result.append({"id": pair_id, "method": method,
                           "grid_joint_canonical": len(original), "method_canonical": len(comparison),
                           "shared_exact_matrices": len(original & comparison),
                           "same_exact_pool": original == comparison})
    return result


def plots(out, records, reference):
    valid = [r for r in records if r["status"] == "success" and r["group"] != "reference"]
    branch_count = max((len(r["analysis"]["branches"]) for r in valid), default=len(reference))
    colors = plt.cm.tab10(np.linspace(0, 1, max(branch_count, 1)))
    fig, axes = plt.subplots(1, len(METHODS), figsize=(22, 4), sharex=True, sharey=True)
    for ax, method in zip(axes, METHODS):
        for r in valid:
            if r["method"] != method: continue
            for b in r["analysis"]["branches"]:
                if b["present"]: ax.scatter(1000*b["xy_m"][0], 1000*b["xy_m"][1], color=colors[b["branch"]], s=9, alpha=.3)
        ax.set_title(method); ax.set_xlabel("fixed u (mm)"); ax.grid(alpha=.2)
    axes[0].set_ylabel("fixed v (mm)")
    fig.tight_layout(); fig.savefig(out/"fixed_plane_positions.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, len(METHODS), figsize=(22, 4), sharey=True)
    for ax, method in zip(axes, METHODS):
        rows = [r for r in valid if r["method"] == method]
        count = Counter(r["analysis"]["winner_branch"] for r in rows)
        local_count = max((len(r["analysis"]["branches"]) for r in rows), default=branch_count)
        ax.bar(range(local_count), [count[i]/len(rows) if rows else 0 for i in range(local_count)], color=colors[:local_count])
        ax.set_title(method); ax.set_xlabel("reference branch index"); ax.set_ylim(0, 1)
    axes[0].set_ylabel("wins / valid runs")
    fig.tight_layout(); fig.savefig(out/"direction_frequency.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 5))
    for method in METHODS:
        gaps = [1000*r["analysis"]["first_second_gap_m"] for r in valid if r["method"] == method and r["analysis"]["first_second_gap_m"] is not None]
        if gaps: ax.hist(gaps, bins=20, alpha=.35, label=method)
    ax.set_xlabel("first–second branch gap within each method (mm)"); ax.set_ylabel("runs")
    ax.legend(); fig.tight_layout(); fig.savefig(out/"score_gaps.png", dpi=160); plt.close(fig)


def write_report(out, summary, trends, review, records, config):
    valid = sum(r["status"] == "success" for r in records)
    lines = ["# 完整粗配准扰动稳定性实验", "",
        f"总尝试 {len(records)}；有效 {valid}；失败 {len(records)-valid}（分母均为方法×扰动输入运行）。",
        "无真实位姿标签；以下都是给定扰动方案下的经验稳定性，获胜频率不是方向正确概率。",
        "候选只供人工复核；没有自动选择方向，也没有触及精配准、正式候选与最终输出。", "",
        "## 扰动与执行", "",
        f"种子 {config['seed']}；每组每档 {config['repeats']} 次；试运行 {config['pilot_repeats']} 次；配对扰动在 paired_inputs/ 保存。",
        "空间采样为带随机相位的均匀体素均值；边界仅从原始 ROI 的既有点增减；白板观测重采样后重新拟合并检查内点率、p90 残差、法向和覆盖。",
        "低/高档扫描仪体素与边界为 0.3/0.6 mm，雷达为 1.5/3 mm，白板体素为 1.5/3 mm。",
        "尺度依据：扫描仪白板残差 p90 约 0.175 mm，雷达白板约 2.322 mm，栅格为 3 mm；这些是观测扰动，不是物体真实尺寸误差。",
        f"搜索粗角步长 {config['search']['angle_step_deg']}°、粗平移步长 {config['search']['translation_step_m']*1000:g} mm；四方向全周搜索及局部细化仍启用。",
        f"近似并列阈值 {config['matching']['near_tie_gap_m']*1000:g} mm；方向聚类 {config['matching']['orientation_cluster_deg']}°，跨运行分配上限 {config['matching']['maximum_rotation_distance_deg']}°。", "",
        "## 汇总（分母见 attempts、valid；方向频率分母为 valid 中有赢家者）", "",
        "| 方法 | 组 | 档 | 有效/尝试 | 近并列/有效 | 赢家次数 | 分支缺失运行 | 模型回退运行 | 边界命中运行 | 耗时 p50 s |",
        "| --- | --- | --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |"]
    for row in summary:
        tie = f"{row['near_tie_runs']}/{row['valid_runs']}" if row["valid_runs"] else "0/0"
        runtime = row["elapsed_s"]["p50"] if row["elapsed_s"] else float("nan")
        lines.append(f"| {row['method']} | {row['group']} | {row['strength']} | {row['valid_runs']}/{row['attempts']} | {tie} | "
            f"{row['winner_counts']} | {row['branch_missing_runs']} | {row['model_fallback_runs']} | {row['boundary_hit_runs']} | {runtime:.2f} |")
    lines += ["", "## 分支位置与方向", "",
        "以下位置是在固定雷达白板平面上，把同一个无扰动扫描仪参考点通过实际三维矩阵投影得到。分支通过旋转距离一对一分配；法向符号单独处理，重复旋转先合并。",
        "| 方法 | 组 | 档 | 分支 | 存活/有效 | 固定点偏移 p90 mm | 角度波动 p90 ° | 同一评分内赢家差 p50 mm |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for row in summary:
        for branch in row["branches"]:
            offset = branch["offset_m"]["p90"]*1000 if branch["offset_m"] else float("nan")
            angle = branch["angle_deg"]["p90"] if branch["angle_deg"] else float("nan")
            gap = branch["score_gap_from_run_best_m"]["p50"]*1000 if branch["score_gap_from_run_best_m"] else float("nan")
            lines.append(f"| {row['method']} | {row['group']} | {row['strength']} | {branch['branch']} | "
                f"{branch['present_runs']}/{branch['valid_run_denominator']} | {offset:.3f} | {angle:.2f} | {gap:.3f} |")
    lines += ["", "## 经验判读", "",
        "位置稳定以各存活分支固定点偏移 p90 均不超过 3 mm 为操作阈值；方向竞争以多分支获胜或近似并列为证据。此分类只针对配置中的扰动。",
        "| 方法 | 组 | 档 | 位置 | 方向 | 跨方向固定点最大距离 p90 mm |",
        "| --- | --- | --- | --- | --- | ---: |"]
    for row in summary:
        offsets = [b["offset_m"]["p90"] for b in row["branches"] if b["offset_m"]]
        stable = (bool(offsets) and row["branch_missing_runs"] == 0 and
                  max(offsets) <= config.get("interpretation", {}).get("position_p90_threshold_m", .003))
        competition = len([x for x in row["winner_counts"].values() if x]) > 1 or row["near_tie_runs"] > 0
        position_text = "稳定" if stable else "不稳定或证据不足"
        direction_text = "有歧义" if competition else "未见切换，仍不能判唯一"
        spread = row["cross_direction_position_spread_m"]
        lines.append(f"| {row['method']} | {row['group']} | {row['strength']} | {position_text} | {direction_text} | "
            f"{1000*spread['p90'] if spread else float('nan'):.3f} |")
    lines += ["", "## 批次趋稳检查", "",
        "按有效运行数分批，比较末两批累计位置 p90、赢家频率和近并列比例。少量重复下频率分辨率很粗，满足阈值也不构成置信保证。",
        "| 方法 | 组 | 档 | 状态 | 最后有效数 | p90 变化 mm | 赢家频率最大变化 | 近并列比例变化 |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: |"]
    for row in review:
        delta = row["last_two_batch_changes"] or {}
        p90 = 1000*delta.get("position_p90_m", float("nan"))
        lines.append(f"| {row['method']} | {row['group']} | {row['strength']} | {row['status']} | {row['last_valid_n']} | "
            f"{p90:.3f} | {delta.get('winner_frequency', float('nan')):.3f} | {delta.get('near_tie_fraction', float('nan')):.3f} |")
    not_converged = sum(x["status"] != "within_configured_tolerances" for x in review)
    lines += ["", f"未达到配置趋稳条件的单元：{not_converged}/{len(review)}。因此其频率和高分位数仍有统计限制。", "",
        "不同评分体系原始分数没有直接相减；`joint_rescore` 与 `grid_joint` 使用同一完整搜索候选池，`model_in_search` 独立改变搜索目标与候选池。",
        "候选池逐输入的精确矩阵交集见 pool_changes.csv；模型参与搜索的池变化归属搜索目标，冻结池重评分只改变评价次序。",
        "若某分支获胜较多，也不能据此认定其为真实方向；近似对称性与无真实标签仍要求人工复核。",
        "精配准接口要求 16 个规范候选而本实验的联合方法通常不足 16 个；这是独立衔接事项，不复制候选凑数。", "",
        "## 产物", "",
        "逐次记录 runs/；配对清单 paired_inputs/；汇总 summary.json、summary.csv、branch_summary.csv；图 fixed_plane_positions.png、direction_frequency.png、score_gaps.png。", ""]
    (out/"report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, default=ROOT/"configs/coarse_stability.json")
    p.add_argument("--output-dir", type=Path, default=ROOT/"outputs/coarse_stability")
    p.add_argument("--pilot", action="store_true")
    p.add_argument("--resume", action="store_true")
    args = p.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    for name, value in config["threads"].items():
        if os.environ.get(name) != value: raise RuntimeError(f"set {name}={value} before starting Python")
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=True)
    registration = json.loads((ROOT/"configs/registration.yaml").read_text(encoding="utf-8"))["coarse_registration"]
    model_cfg = json.loads((ROOT/"configs/stage3_model_prior.json").read_text(encoding="utf-8"))
    registration["geometry"]["joint_search"]["angle_step_deg"] = config["search"]["angle_step_deg"]
    registration["geometry"]["joint_search"]["translation_step_m"] = config["search"]["translation_step_m"]
    inputs = {key: ROOT/path for key, path in config["input_paths"].items()}
    source_report = json.loads((ROOT/"outputs/stage_02_segmentation/segmentation_report.json").read_text(encoding="utf-8"))
    for role in ROLES:
        if sha256_file(inputs[role]) != source_report["outputs"][role+"_points"]["sha256"]:
            raise RuntimeError(f"stage-2 canonical input hash mismatch: {role}")
    original = {role: load_metric_stage2_cloud(inputs[role])[1] for role in ROLES}
    rois = {side: load_metric_stage2_cloud(inputs[side+"_roi"])[1] * (.001 if side == "source" else 1.) for side in ("source", "target")}
    original_hashes = {k: hash_points(v) for k, v in original.items()}
    input_file_hashes = {k: sha256_file(path) for k, path in inputs.items()}
    baseline_planes, base_source_frame, base_target_frames, _ = planes_and_frames(original, registration)
    fixed_frame = base_target_frames[0]
    fixed_point = np.median(original["source_object"], axis=0)
    fixed = {"source_reference_point_m": fixed_point.tolist(), "target_plane": baseline_planes["target"]["oriented_plane_model"],
             "target_frame": {k: fixed_frame[k].tolist() for k in ("origin", "u", "v", "n")}}
    boundary_frames = {"source": base_source_frame, "target": fixed_frame}
    # Mirror the existing stage-3 scorer: its report lookup uses `inputs`, while
    # stage 2 stores ROI metadata under `input_roles`, so crop weights are inactive.
    # The actual ROI observations still constrain boundary perturbations above.
    roi_bounds = {"source": None, "target": None}
    manifest = make_manifest(config)
    limit = config["pilot_repeats"] if args.pilot else config["repeats"]
    manifest = [r for r in manifest if r["group"] == "reference" or r["repeat"] < limit]
    implementation_files = [Path(__file__), ROOT/"scripts/03_generate_candidates.py",
        ROOT/"src/pointcloud_registration/candidates.py", ROOT/"src/pointcloud_registration/joint_search.py",
        ROOT/"src/pointcloud_registration/shape_grid.py", ROOT/"src/pointcloud_registration/model_prior.py"]
    implementation_sha256 = {str(path.resolve()): sha256_file(path) for path in implementation_files}
    if args.resume and (out/"paired_manifest.json").exists():
        old = json.loads((out/"paired_manifest.json").read_text(encoding="utf-8"))
        if old["config_sha256"] != sha256_file(args.config): raise RuntimeError("resume configuration changed")
        if old.get("implementation_sha256") != implementation_sha256: raise RuntimeError("resume implementation changed")
    dump(out/"paired_manifest.json", {"config": config, "config_sha256": sha256_file(args.config),
        "registration_config_sha256": sha256_file(ROOT/"configs/registration.yaml"),
        "model_config_sha256": sha256_file(ROOT/"configs/stage3_model_prior.json"),
        "implementation_sha256": implementation_sha256,
        "input_file_sha256": input_file_hashes, "original_point_sha256": original_hashes,
        "fixed_reference": fixed, "rows": manifest,
        "environment": {"python": sys.version, "platform": platform.platform(), "numpy": np.__version__,
                        "scipy": scipy.__version__, "open3d": o3d.__version__, "threads": config["threads"]}})
    records = []
    reference_candidates = {}
    for row in manifest:
        perturb_error = None
        try:
            points, perturb_audit = perturb(original, rois, boundary_frames, row, config)
            hashes = {k: hash_points(v) for k, v in points.items()}
        except Exception as exc:
            points, perturb_audit, hashes = None, {}, None
            perturb_error = f"perturbation:{type(exc).__name__}: {exc}"
        pair_record = {**row, "original_point_sha256": original_hashes, "perturbed_point_sha256": hashes,
                       "point_counts": {k: len(v) for k, v in points.items()} if points is not None else None,
                       "perturbation_audit": perturb_audit, "failure_reason": perturb_error}
        dump(out/"paired_inputs"/(row["id"]+".json"), pair_record)
        if perturb_error:
            reports, source_frame, target_frames, gate = {}, None, [], {"passed": False, "reasons": [perturb_error]}
            plane_error = perturb_error
        else:
            try:
                reports, source_frame, target_frames, gate = planes_and_frames(points, registration, baseline_planes, config["plane_quality"])
                plane_error = None if gate["passed"] else "plane_quality:"+",".join(gate["reasons"])
            except Exception as exc:
                reports, source_frame, target_frames, gate = {}, None, [], {"passed": False, "reasons": [str(exc)]}
                plane_error = "plane_fit:"+str(exc)
        grid_joint_result = None
        for method in METHODS:
            run_path = out/"runs"/(row["id"]+"__"+method+".json")
            if args.resume and run_path.exists():
                record = json.loads(run_path.read_text(encoding="utf-8"))
                records.append(record)
                if row["group"] == "reference" and record["status"] == "success": reference_candidates[method] = record["canonical_candidates"]
                if method == "grid_joint" and record["status"] == "success": grid_joint_result = record
                continue
            start = time.perf_counter()
            record = {"id": row["id"], "method": method, "group": row["group"], "strength": row["strength"],
                      "repeat": row["repeat"], "seed": row["seed"], "paired_input_sha256": hashes,
                      "plane_report": reports, "plane_gate": gate, "status": "failed", "failure_reason": None}
            try:
                if plane_error: raise RuntimeError(plane_error)
                result = one_method(method, points, source_frame, target_frames, registration, model_cfg, hashes, roi_bounds,
                                    grid_joint_result)
                record.update(result)
                if method == "grid_joint": grid_joint_result = result
                if row["group"] == "reference": reference_candidates[method] = result["canonical_candidates"]
                reference = reference_candidates[method]
                record["analysis"] = compact_run(method, result, reference, fixed_point, fixed_frame,
                    base_source_frame["n"], fixed_frame["n"], config["matching"])
                record["status"] = "success"
            except Exception as exc:
                record["failure_reason"] = f"{type(exc).__name__}: {exc}"
                record["traceback"] = traceback.format_exc()
            record["elapsed_s"] = time.perf_counter() - start
            dump(run_path, record)
            records.append(record)
            print(row["id"], method, record["status"], round(record["elapsed_s"], 2), flush=True)
    by_pair = {(r["id"], r["method"]): r for r in records}
    for record in records:
        if record["method"] == "joint_rescore" and record["status"] == "success" and "incremental_rescore_elapsed_s" not in record:
            parent = by_pair.get((record["id"], "grid_joint"))
            if parent and parent["status"] == "success":
                record["incremental_rescore_elapsed_s"] = record["elapsed_s"]
                record["elapsed_s"] += parent["elapsed_s"]
                dump(out/"runs"/(record["id"]+"__joint_rescore.json"), record)
    pool_changes = candidate_pool_comparison(records)
    write_csv(out/"pool_changes.csv", pool_changes)
    summary, trends = summarize(records, manifest, METHODS, {**config["convergence"], "batch_size": config["batch_size"]})
    review = convergence_review(trends, config, summary)
    dump(out/"summary.json", {"summary": summary, "batch_trends": trends, "pool_changes": pool_changes,
        "convergence_review": review,
        "total_attempts": len(records), "valid_runs": sum(r["status"] == "success" for r in records),
        "failed_runs": sum(r["status"] != "success" for r in records),
        "overall_failure_rate": sum(r["status"] != "success" for r in records)/len(records) if records else None,
        "failure_reasons": dict(Counter(r["failure_reason"] for r in records if r["status"] != "success"))})
    flat = [{k:v for k,v in item.items() if k != "branches"} for item in summary]
    write_csv(out/"summary.csv", [{k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in flat])
    branch_rows = [{"method": item["method"], "group": item["group"], "strength": item["strength"], **branch}
                   for item in summary for branch in item["branches"]]
    write_csv(out/"branch_summary.csv", [{k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in branch_rows])
    if all(method in reference_candidates for method in METHODS): plots(out, records, reference_candidates["grid_joint"])
    write_report(out, summary, trends, review, records, config)
    print(out/"summary.json")


if __name__ == "__main__": main()
