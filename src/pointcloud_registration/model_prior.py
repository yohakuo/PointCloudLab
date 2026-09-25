"""Auditable single observed face prior for stage-3 candidate rescoring.

The footprint is a union of observed grid cells inside a finite rectangle.
No side, rear face, or object thickness is inferred.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def fit_supported_face(grid, cfg):
    uv = np.asarray(grid["centers"], float)
    h = np.asarray(grid["height_p50_m"], float)
    confidence = np.asarray(grid["confidence"], float)
    crop = np.asarray(grid["crop_cell"], bool)
    reasons = []
    eligible = (confidence >= cfg["source_min_confidence"]) & ~crop & np.isfinite(h)
    if eligible.sum() < cfg["minimum_source_cells"]:
        reasons.append("insufficient_reliable_source_cells")
    if reasons:
        return {"valid": False, "reasons": reasons}, None
    # Initialize from the densest *upper* height band, not the highest points:
    # isolated returns above the actual front must not define a false plane.
    band = cfg["source_inlier_threshold_m"]
    upper = eligible & (h >= np.quantile(h[eligible], cfg["front_seed_quantile"]) - band)
    bin_index = np.floor(h[upper] / band).astype(int)
    values, counts = np.unique(bin_index, return_counts=True)
    modal_bin = values[np.lexsort((values, counts))[-1]]
    seed = upper & (np.abs(h - (modal_bin + .5) * band) <= 1.5 * band)
    x = np.column_stack((uv, np.ones(len(uv))))
    if seed.sum() < 3:
        return {"valid": False, "reasons": ["front_seed_degenerate"]}, None
    active = seed.copy()
    for _ in range(6):
        beta = np.linalg.lstsq(x[active] * np.sqrt(confidence[active, None]),
                               h[active] * np.sqrt(confidence[active]), rcond=None)[0]
        residual = h - x @ beta
        active = eligible & (np.abs(residual) <= cfg["source_inlier_threshold_m"])
        if active.sum() < 3:
            break
    if active.sum() < 3:
        return {"valid": False, "reasons": ["planar_fit_has_no_stable_inliers"]}, None
    beta = np.linalg.lstsq(x[active] * np.sqrt(confidence[active, None]),
                           h[active] * np.sqrt(confidence[active]), rcond=None)[0]
    residual = h[active] - x[active] @ beta
    bounds = np.column_stack((uv[active].min(axis=0) - cfg["grid_size_m"] / 2,
                              uv[active].max(axis=0) + cfg["grid_size_m"] / 2))
    dimensions = bounds[:, 1] - bounds[:, 0]
    lattice = np.maximum(1, np.ceil(dimensions / cfg["grid_size_m"]).astype(int))
    coverage = float(active.sum() / np.prod(lattice))
    design = x[active].T @ (x[active] * confidence[active, None])
    condition = float(np.linalg.cond(design))
    sigma = max(1.4826 * float(np.median(np.abs(residual - np.median(residual)))),
                cfg["uncertainty_floor_m"])
    uncertainty = np.sqrt(np.maximum(np.diag(np.linalg.pinv(design)), 0)) * sigma
    p90 = float(np.percentile(np.abs(residual), 90))
    if active.sum() < cfg["minimum_source_cells"]:
        reasons.append("too_few_supported_front_cells")
    if np.min(dimensions) < cfg["minimum_face_extent_m"]:
        reasons.append("front_face_extent_too_small")
    if coverage < cfg["minimum_source_coverage"]:
        reasons.append("observed_front_coverage_too_low")
    if p90 > cfg["maximum_fit_p90_m"]:
        reasons.append("front_fit_residual_too_large")
    if condition > cfg["maximum_design_condition"]:
        reasons.append("face_parameters_degenerate")
    expected = cfg["nominal_face_size_m"]
    z = np.abs(dimensions - expected) / cfg["face_size_sigma_m"]
    if np.any(z > cfg["maximum_size_z"]):
        reasons.append("soft_size_prior_conflicts_with_observed_extent")
    report = {"valid": not reasons, "reasons": reasons, "type": "single_observed_finite_planar_face",
              "model_height_m": {"du": float(beta[0]), "dv": float(beta[1]), "intercept": float(beta[2])},
              "parameter_standard_error": {"du": float(uncertainty[0]),
                                           "dv": float(uncertainty[1]),
                                           "intercept_m": float(uncertainty[2])},
              "design_condition": condition, "residual_p90_m": p90,
              "supported_cell_count": int(active.sum()), "eligible_cell_count": int(eligible.sum()),
              "excluded_other_height_cell_count": int(eligible.sum() - active.sum()),
              "observed_grid_coverage": coverage, "support_rectangle_uv_m": bounds.tolist(),
              "supported_cell_centers_uv_m": uv[active].tolist(),
              "observed_extents_m": dimensions.tolist(), "soft_size_z": z.tolist(),
              "surface_policy": "only inlier cells on the observed front; other cells and all other faces unknown",
              "thickness_m": None}
    return report, {"uv": uv[active], "height": x[active] @ beta,
                    "beta": beta, "bounds": bounds, "grid_size_m": cfg["grid_size_m"]} if not reasons else None


def reliable_radar_cells(grid, cfg):
    confidence = np.asarray(grid["confidence"], float)
    h25 = np.asarray(grid["height_p25_m"], float)
    h75 = np.asarray(grid["height_p75_m"], float)
    crop = np.asarray(grid["crop_cell"], bool)
    mask = ((confidence >= cfg["radar_min_confidence"]) & ~crop &
            (h75 >= cfg["radar_min_front_height_m"]) &
            ((h75 - h25) <= cfg["radar_max_height_iqr_m"]))
    # All weights are fixed from observation quality, independent of a transform.
    spread = np.maximum(h75 - h25, cfg["radar_height_uncertainty_floor_m"])
    weight = confidence * np.clip(cfg["radar_height_weight_scale_m"] / spread, .25, 1.)
    return mask, weight, spread


def score_supported_face(model, source_frame, target_frame, target_grid, matrix, cfg):
    mask, weight_all, spread_all = reliable_radar_cells(target_grid, cfg)
    uv = np.asarray(target_grid["centers"], float)[mask]
    h = np.asarray(target_grid["height_p75_m"], float)[mask]
    weight = weight_all[mask]
    if len(uv) < cfg["minimum_radar_cells"] or weight.sum() <= 0:
        return {"valid": False, "reason": "insufficient_prequalified_radar_cells",
                "eligible_radar_cells": int(len(uv))}
    target_xyz = (np.asarray(target_frame["origin"]) +
                  uv[:, 0, None] * np.asarray(target_frame["u"]) +
                  uv[:, 1, None] * np.asarray(target_frame["v"]) +
                  h[:, None] * np.asarray(target_frame["n"]))
    t = np.asarray(matrix, float)
    source_xyz = (target_xyz - t[:3, 3]) @ t[:3, :3]
    relative = source_xyz - np.asarray(source_frame["origin"])
    local = np.column_stack((relative @ source_frame["u"], relative @ source_frame["v"]))
    local_h = relative @ source_frame["n"]
    bounds = model["bounds"]
    inside = np.all((local >= bounds[:, 0]) & (local <= bounds[:, 1]), axis=1)
    lateral, nearest = cKDTree(model["uv"]).query(local, workers=1)
    lateral = np.maximum(lateral - model["grid_size_m"] / np.sqrt(2), 0.)
    expected_h = local @ model["beta"][:2] + model["beta"][2]
    vertical = np.maximum(np.abs(local_h - expected_h) -
                          np.maximum(spread_all[mask] * cfg["radar_iqr_uncertainty_fraction"],
                                     cfg["radar_height_uncertainty_floor_m"]), 0.)
    distance = np.hypot(lateral, vertical)
    supported = inside & (lateral <= cfg["match_lateral_tolerance_m"]) & (
        vertical <= cfg["match_vertical_tolerance_m"])
    supported_count = int(supported.sum())
    supported_weight = float(weight[supported].sum())
    radar_fraction = supported_weight / float(weight.sum())
    source_fraction = len(np.unique(nearest[supported])) / len(model["uv"])
    if supported_count < cfg["minimum_matched_radar_cells"]:
        reason = "matched_radar_support_below_minimum"
    elif radar_fraction < cfg["minimum_radar_support_fraction"]:
        reason = "radar_supported_fraction_below_minimum"
    elif source_fraction < cfg["minimum_source_match_coverage"]:
        reason = "observed_source_face_coverage_below_minimum"
    else:
        reason = None
    # Points over unobserved holes or outside the supported rectangle are
    # unknown. They never enter the distance average or count as coverage.
    footprint = inside & (lateral <= cfg["match_lateral_tolerance_m"])
    if np.any(footprint):
        clipped = np.minimum(distance[footprint], cfg["model_distance_clip_m"])
        footprint_weight = weight[footprint]
        order = np.argsort(clipped, kind="stable")
        keep = np.cumsum(footprint_weight[order]) <= cfg["model_trim_fraction"] * footprint_weight.sum()
        keep[0] = True
        robust = float(np.average(clipped[order][keep], weights=footprint_weight[order][keep]))
        trimmed_count = int(keep.sum())
    else:
        robust, trimmed_count = float(cfg["model_distance_clip_m"]), 0
    coverage_penalty = cfg["coverage_penalty_scale_m"] * (
        (1 - radar_fraction) + (1 - source_fraction)) / 2
    return {"valid": reason is None, "reason": reason,
            "distance_m": robust + coverage_penalty,
            "robust_supported_distance_m": robust, "coverage_penalty_m": coverage_penalty,
            "eligible_radar_cells": int(len(uv)), "eligible_weight": float(weight.sum()),
            "matched_radar_cells": supported_count, "matched_weight": supported_weight,
            "radar_supported_fraction": radar_fraction, "observed_source_face_coverage": source_fraction,
            "inside_rectangle_cells": int(inside.sum()),
            "observed_footprint_cells": int(footprint.sum()), "trimmed_cells": trimmed_count,
            "weights_determined_before_candidate": True}


def combine_score(model_term, old_terms, cfg):
    weights = cfg["score_weights"]
    if not model_term["valid"]:
        return None
    return (weights["model"] * model_term["distance_m"] +
            weights["occupancy"] * old_terms["occupancy_m"] +
            weights["contour"] * old_terms["outer_contour_m"] +
            weights["height"] * old_terms["height_m"])
