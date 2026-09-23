"""Stage 2 geometric region segmentation; this module performs no registration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree

Side = Literal["auto", "positive", "negative"]


def stage2_contract() -> dict[str, Any]:
    return {"stage": 2, "registration_performed": False, "transform_estimated": False,
            "transform_direction": "INSPIRE_TO_FAST"}


def automatic_target_selection_allowed(mode: str) -> bool:
    """Single auditable guard used by stage 2 target selection."""
    if mode not in {"automatic", "manual_files"}:
        raise ValueError(f"unknown target selection mode: {mode}")
    return mode == "automatic"


def _floats(values: np.ndarray) -> list[float]:
    return [float(x) for x in values]


def canonicalize_plane(model: np.ndarray) -> np.ndarray:
    """Normalize ax+by+cz+d=0 and deterministically fix its sign."""
    plane = np.asarray(model, dtype=np.float64).copy()
    if plane.shape != (4,):
        raise ValueError("plane model must contain four coefficients")
    norm = float(np.linalg.norm(plane[:3]))
    if not np.isfinite(norm) or norm <= np.finfo(float).eps:
        raise ValueError("plane normal is invalid")
    plane /= norm
    pivot = int(np.argmax(np.abs(plane[:3])))
    if plane[pivot] < 0:
        plane = -plane
    return plane


def signed_distances(points_m: np.ndarray, plane: np.ndarray) -> np.ndarray:
    points = np.asarray(points_m, dtype=np.float64)
    normalized = canonicalize_plane(plane)
    return points @ normalized[:3] + normalized[3]


def clean_cloud(cloud: o3d.geometry.PointCloud, scale_to_m: float) -> tuple[o3d.geometry.PointCloud, np.ndarray, dict[str, Any]]:
    """Clean XYZ while keeping color/normal rows aligned and scaling only about origin."""
    native = np.asarray(cloud.points, dtype=np.float64)
    nan_mask = np.isnan(native).any(axis=1)
    inf_mask = np.isinf(native).any(axis=1)
    finite_mask = np.isfinite(native).all(axis=1)
    finite_indices = np.flatnonzero(finite_mask)
    finite = native[finite_mask]
    if len(finite):
        _, first = np.unique(finite, axis=0, return_index=True)
        keep = finite_indices[np.sort(first)]
    else:
        keep = np.empty(0, dtype=np.int64)
    result = cloud.select_by_index(keep.tolist())
    metric = native[keep] * float(scale_to_m)
    result.points = o3d.utility.Vector3dVector(metric)
    scale = float(scale_to_m)
    stats = {
        "raw_point_count": int(len(native)), "finite_point_count": int(finite_mask.sum()),
        "clean_point_count": int(len(keep)), "non_finite_removed": int((~finite_mask).sum()),
        "nan_row_count": int(nan_mask.sum()), "inf_row_count": int(inf_mask.sum()),
        "exact_duplicates_removed": int(len(finite) - len(keep)),
        "duplicate_ratio": float((len(finite) - len(keep)) / len(finite)) if len(finite) else 0.0,
        "operations": ["remove_non_finite_xyz", "remove_exact_duplicate_xyz"],
        "coordinates": ("unchanged FAST metre coordinates; no scale/translation/recentering/normalization"
                        if scale == 1.0 else f"XYZ_m = XYZ_native * {scale:g} about origin (0,0,0)"),
    }
    return result, metric.copy(), stats


@dataclass(frozen=True)
class PlaneFit:
    model: np.ndarray
    inlier_indices: np.ndarray
    report: dict[str, Any]


def fit_plane_ransac(points_m: np.ndarray, *, threshold_m: float, ransac_n: int,
                     iterations: int, seed: int) -> PlaneFit:
    points = np.asarray(points_m, dtype=np.float64)
    if len(points) < ransac_n:
        raise ValueError(f"plane fit needs at least {ransac_n} points, got {len(points)}")
    o3d.utility.random.seed(int(seed))
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    raw_model, indices = cloud.segment_plane(float(threshold_m), int(ransac_n), int(iterations))
    inliers = np.asarray(indices, dtype=np.int64)
    if len(inliers) < ransac_n:
        raise RuntimeError("plane RANSAC returned too few inliers")
    model = canonicalize_plane(np.asarray(raw_model))
    residual = np.abs(signed_distances(points[inliers], model))
    median = float(np.median(residual))
    mad = float(np.median(np.abs(residual - median)))
    report = {
        "plane_model_ax_by_cz_d": _floats(model), "unit_normal": _floats(model[:3]),
        "normal_norm": float(np.linalg.norm(model[:3])),
        "normal_sign_rule": "normalize; force largest-absolute normal component positive",
        "ransac": {"distance_threshold_m": float(threshold_m), "ransac_n": int(ransac_n),
                   "num_iterations": int(iterations), "random_seed": int(seed)},
        "plane_inlier_count": int(len(inliers)), "plane_inlier_ratio": float(len(inliers) / len(points)),
        "absolute_residual_m": {f"p{p}": float(np.percentile(residual, p)) for p in (50, 90, 95)},
        "residual_median_m": median, "residual_mad_m": mad,
        "mad_robust_noise_sigma_m": float(1.4826 * mad),
    }
    return PlaneFit(model, inliers, report)


def threshold_from_plane(plane_report: dict[str, Any], *, sigma_multiplier: float,
                         minimum_m: float, override_m: float | None) -> tuple[float, dict[str, Any]]:
    sigma = float(plane_report["mad_robust_noise_sigma_m"])
    automatic = max(float(minimum_m), float(sigma_multiplier) * sigma)
    final = float(override_m) if override_m is not None else automatic
    if final <= 0:
        raise ValueError("object distance threshold must be positive")
    return final, {
        "formula": "max(minimum_m, sigma_multiplier * 1.4826 * MAD(abs(board_inlier_residual)))",
        "mad_robust_noise_sigma_m": sigma, "sigma_multiplier": float(sigma_multiplier),
        "minimum_m": float(minimum_m), "automatic_threshold_m": automatic,
        "manual_override_m": override_m, "final_threshold_m": final,
        "selection": "manual_override" if override_m is not None else "automatic_robust_statistics",
        "interpretation": "segmentation distance only; not physical thickness",
    }


def side_candidate_indices(points_m: np.ndarray, plane: np.ndarray, threshold_m: float) -> dict[str, np.ndarray]:
    distances = signed_distances(points_m, plane)
    return {
        "positive": np.flatnonzero(distances > threshold_m),
        "negative": np.flatnonzero(distances < -threshold_m),
    }


def _point_stats(points: np.ndarray) -> dict[str, Any]:
    if not len(points):
        return {"point_count": 0, "centroid_m": None, "xyz_min_m": None, "xyz_max_m": None, "extent_m": None}
    low, high = points.min(axis=0), points.max(axis=0)
    extent = high - low
    return {"point_count": int(len(points)), "centroid_m": _floats(points.mean(axis=0)),
            "xyz_min_m": _floats(low), "xyz_max_m": _floats(high), "extent_m": _floats(extent),
            "bounding_box_diagonal_m": float(np.linalg.norm(extent)),
            "coordinate_max_abs_m": float(np.max(np.abs(points)))}


def cluster_candidate(points_m: np.ndarray, candidate_indices: np.ndarray, *, eps_m: float,
                      min_points: int, min_cluster_points: int, max_extent_m: float,
                      centrality_limit: float, plane: np.ndarray | None = None) -> tuple[list[np.ndarray], list[dict[str, Any]]]:
    """Find and assess all connected components using centrality and scale, not size alone."""
    candidate_indices = np.asarray(candidate_indices, dtype=np.int64)
    if not len(candidate_indices):
        return [], []
    candidate = points_m[candidate_indices]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(candidate))
    labels = np.asarray(cloud.cluster_dbscan(eps=float(eps_m), min_points=int(min_points), print_progress=False))
    normal = canonicalize_plane(np.asarray(plane if plane is not None else [0., 0., 1., 0.]))[:3]
    reference = np.array([0., 0., 1.]) if abs(normal[2]) < .9 else np.array([1., 0., 0.])
    u = np.cross(normal, reference); u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    roi_uv = np.column_stack((points_m @ u, points_m @ v))
    roi_low, roi_high = np.percentile(roi_uv, [1, 99], axis=0)
    roi_center = np.median(roi_uv, axis=0)
    roi_extent = np.maximum(roi_high - roi_low, np.finfo(float).eps)
    clusters: list[np.ndarray] = []
    reports: list[dict[str, Any]] = []
    for label in sorted(set(labels.tolist())):
        if label < 0:
            continue
        local = np.flatnonzero(labels == label)
        global_indices = candidate_indices[local]
        pts = points_m[global_indices]
        stats = _point_stats(pts)
        uv = np.column_stack((pts @ u, pts @ v))
        centrality = float(np.linalg.norm((np.median(uv, axis=0) - roi_center) / roi_extent))
        radial = np.linalg.norm((uv - roi_center) / roi_extent, axis=1)
        radial_p50 = float(np.percentile(radial, 50))
        edge = (uv < roi_low + .1 * roi_extent) | (uv > roi_high - .1 * roi_extent)
        boundary_fraction = float(np.mean(np.any(edge, axis=1)))
        extent_max = float(np.max(np.ptp(pts, axis=0)))
        eligible = len(pts) >= min_cluster_points and extent_max <= max_extent_m and centrality <= centrality_limit
        reasons = []
        if len(pts) < min_cluster_points: reasons.append("below_min_cluster_points")
        if extent_max > max_extent_m: reasons.append("extent_exceeds_object_limit")
        if centrality > centrality_limit: reasons.append("not_central_in_roi")
        if eligible: reasons.append("passes_count_scale_and_centrality_rules")
        # Object semantics favor a central, concentrated face and penalize ROI-edge residuals.
        score = (np.log1p(len(pts)) * (1.0 - boundary_fraction) /
                 ((1.0 + centrality) * max(radial_p50, .05))) if eligible else 0.0
        reports.append({"cluster_label": int(label), **stats, "centrality_normalized": centrality,
                        "plane_local_radial_p50_normalized": radial_p50,
                        "roi_boundary_fraction": boundary_fraction,
                        "max_extent_m": extent_max, "eligible": bool(eligible), "selection_score": float(score),
                        "selection_score_formula": "log1p(count)*(1-boundary_fraction)/((1+centrality)*max(radial_p50,0.05))",
                        "decision": "eligible_candidate" if eligible else "deleted_outlier_or_non_object",
                        "reasons": reasons})
        clusters.append(global_indices)
    noise_count = int(np.count_nonzero(labels < 0))
    if noise_count:
        reports.append({"cluster_label": -1, "point_count": noise_count, "eligible": False,
                        "selection_score": 0.0, "decision": "deleted_dbscan_noise",
                        "reasons": ["does_not_meet_local_connectivity_min_points"]})
    return clusters, reports


def choose_object_side(points_m: np.ndarray, plane: np.ndarray, threshold_m: float, *, requested_side: Side,
                       eps_m: float, min_points: int, min_cluster_points: int, max_extent_m: float,
                       centrality_limit: float, auto_score_ratio: float) -> dict[str, Any]:
    sides = side_candidate_indices(points_m, plane, threshold_m)
    result: dict[str, Any] = {"requested_side": requested_side, "sides": {}}
    best: dict[str, tuple[np.ndarray, dict[str, Any]] | None] = {}
    for side, indices in sides.items():
        clusters, reports = cluster_candidate(points_m, indices, eps_m=eps_m, min_points=min_points,
                                               min_cluster_points=min_cluster_points, max_extent_m=max_extent_m,
                                               centrality_limit=centrality_limit, plane=plane)
        pairs = [(idx, rep) for idx, rep in zip(clusters, reports) if rep.get("eligible")]
        pairs.sort(key=lambda pair: pair[1]["selection_score"], reverse=True)
        best[side] = pairs[0] if pairs else None
        distances = signed_distances(points_m[indices], plane)
        result["sides"][side] = {"raw_candidate_indices": indices, "raw_candidate_count": int(len(indices)),
                                  "signed_distance_m": ({f"p{p}": float(np.percentile(distances, p)) for p in (5, 50, 95)} if len(distances) else None),
                                  "clusters": reports,
                                  "best_cluster_indices": pairs[0][0] if pairs else np.empty(0, dtype=np.int64)}
    chosen: str | None = None
    mode: str | None = None
    reason: str
    if requested_side in ("positive", "negative"):
        if best[requested_side] is None:
            reason = f"manual override {requested_side} has no eligible central cluster"
        else:
            chosen, mode, reason = requested_side, "manual_override", f"CLI/config explicitly selected {requested_side}"
    else:
        available = [side for side in ("positive", "negative") if best[side] is not None]
        if len(available) == 1:
            chosen, mode = available[0], "automatic_with_evidence"
            reason = f"only {chosen} has a cluster passing count, scale, connectivity, and centrality rules"
        elif len(available) == 2:
            scores = {side: float(best[side][1]["selection_score"]) for side in available}  # type: ignore[index]
            winner = max(scores, key=scores.get)
            loser = "negative" if winner == "positive" else "positive"
            ratio = scores[winner] / max(scores[loser], np.finfo(float).eps)
            if ratio >= auto_score_ratio:
                chosen, mode = winner, "automatic_with_evidence"
                reason = f"central eligible cluster score ratio {ratio:.3f} >= {auto_score_ratio:.3f}"
            else:
                reason = f"both sides have plausible central clusters; score ratio {ratio:.3f} < {auto_score_ratio:.3f}"
        else:
            reason = "neither side has an eligible central connected cluster"
    selected = result["sides"][chosen]["best_cluster_indices"] if chosen else np.empty(0, dtype=np.int64)
    result.update({"selected_side": chosen, "selection_mode": mode, "selection_reason": reason,
                   "selected_indices": selected, "confirmed": chosen is not None and len(selected) > 0})
    return result


_TARGET_HEIGHTMAP_REQUIRED_KEYS = {
    "enabled", "side", "plane_outer_fraction", "minimum_refit_points",
    "minimum_refit_inlier_ratio", "grid_size_m", "minimum_points_per_cell",
    "cell_depth_quantile", "high_threshold_sigma_multiplier",
    "high_threshold_minimum_m", "low_threshold_sigma_multiplier",
    "low_threshold_minimum_m", "morphology_close_radius_cells", "maximum_gap_cells",
    "connectivity", "minimum_object_points", "minimum_occupied_cells",
    "minimum_projection_extent_m", "maximum_projection_extent_m",
    "minimum_aspect_ratio", "maximum_aspect_ratio",
    "source_extent_minimum_ratio", "source_extent_maximum_ratio",
    "maximum_roi_boundary_fraction", "maximum_object_depth_m",
    "allow_plane_refit_fallback",
}


def _plane_basis(plane: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normal = canonicalize_plane(plane)[:3]
    reference = np.array([0., 0., 1.]) if abs(normal[2]) < .9 else np.array([1., 0., 0.])
    u = np.cross(normal, reference)
    u /= np.linalg.norm(u)
    return u, np.cross(normal, u)


def _validate_target_heightmap_config(config: dict[str, Any]) -> dict[str, Any]:
    missing = sorted(_TARGET_HEIGHTMAP_REQUIRED_KEYS - set(config))
    if missing:
        raise ValueError(f"target_heightmap configuration is missing required keys: {', '.join(missing)}")
    cfg = dict(config)
    if not bool(cfg["enabled"]):
        raise ValueError("target_heightmap.enabled must be true for automatic target selection")
    if cfg["side"] not in {"auto", "positive", "negative"}:
        raise ValueError("target_heightmap.side must be auto, positive, or negative")
    if float(cfg["grid_size_m"]) <= 0:
        raise ValueError("target_heightmap.grid_size_m must be positive")
    if int(cfg["minimum_points_per_cell"]) < 1:
        raise ValueError("target_heightmap.minimum_points_per_cell must be at least 1")
    if not 0 < float(cfg["cell_depth_quantile"]) <= 1:
        raise ValueError("target_heightmap.cell_depth_quantile must be in (0, 1]")
    if not 0 < float(cfg["plane_outer_fraction"]) < .5:
        raise ValueError("target_heightmap.plane_outer_fraction must be in (0, 0.5)")
    if int(cfg["minimum_refit_points"]) < 3:
        raise ValueError("target_heightmap.minimum_refit_points must be at least 3")
    if not 0 <= float(cfg["minimum_refit_inlier_ratio"]) <= 1:
        raise ValueError("target_heightmap.minimum_refit_inlier_ratio must be in [0, 1]")
    if int(cfg["connectivity"]) not in {4, 8}:
        raise ValueError("target_heightmap.connectivity must be 4 or 8")
    if int(cfg["morphology_close_radius_cells"]) < 0 or int(cfg["maximum_gap_cells"]) < 0:
        raise ValueError("target_heightmap morphology radii must be non-negative")
    positive_names = (
        "high_threshold_sigma_multiplier", "high_threshold_minimum_m",
        "low_threshold_sigma_multiplier", "low_threshold_minimum_m",
        "minimum_object_points", "minimum_occupied_cells", "minimum_projection_extent_m",
        "maximum_projection_extent_m", "minimum_aspect_ratio", "maximum_aspect_ratio",
        "source_extent_minimum_ratio", "source_extent_maximum_ratio",
        "maximum_object_depth_m",
    )
    if any(float(cfg[name]) <= 0 for name in positive_names):
        raise ValueError("target_heightmap thresholds, counts, extents, and ratios must be positive")
    if float(cfg["minimum_projection_extent_m"]) >= float(cfg["maximum_projection_extent_m"]):
        raise ValueError("target_heightmap projection extent minimum must be below maximum")
    if float(cfg["minimum_aspect_ratio"]) > 1 or float(cfg["maximum_aspect_ratio"]) < 1:
        raise ValueError("target_heightmap aspect-ratio interval must contain 1")
    if float(cfg["source_extent_minimum_ratio"]) >= float(cfg["source_extent_maximum_ratio"]):
        raise ValueError("target_heightmap source extent minimum ratio must be below maximum ratio")
    if not 0 <= float(cfg["maximum_roi_boundary_fraction"]) <= 1:
        raise ValueError("target_heightmap.maximum_roi_boundary_fraction must be in [0, 1]")
    return cfg


def _target_plane_refit(points: np.ndarray, initial: PlaneFit, cfg: dict[str, Any], *,
                        threshold_m: float, ransac_n: int, iterations: int, seed: int) -> tuple[PlaneFit, dict[str, Any]]:
    """Refit from the ROI perimeter so the central object face cannot dominate."""
    u, v = _plane_basis(initial.model)
    uv = np.column_stack((points @ u, points @ v))
    low, high = np.percentile(uv, [1, 99], axis=0)
    extent = np.maximum(high - low, np.finfo(float).eps)
    fraction = float(cfg["plane_outer_fraction"])
    outer = np.any((uv <= low + fraction * extent) | (uv >= high - fraction * extent), axis=1)
    outer_indices = np.flatnonzero(outer)
    report: dict[str, Any] = {
        "fit_point_source": "target_roi_outer_perimeter",
        "outer_fraction": fraction,
        "outer_point_count": int(len(outer_indices)),
        "inner_excluded_point_count": int(len(points) - len(outer_indices)),
        "fallback_used": False,
        "fallback_reason": None,
        "initial_plane": initial.report,
    }
    reason: str | None = None
    refit: PlaneFit | None = None
    if len(outer_indices) < int(cfg["minimum_refit_points"]):
        reason = "insufficient_outer_refit_points"
    else:
        try:
            local = fit_plane_ransac(points[outer_indices], threshold_m=threshold_m, ransac_n=ransac_n,
                                     iterations=iterations, seed=seed)
            normal_agreement = abs(float(np.dot(local.model[:3], initial.model[:3])))
            if local.report["plane_inlier_ratio"] < float(cfg["minimum_refit_inlier_ratio"]):
                reason = "outer_refit_inlier_ratio_below_minimum"
            elif normal_agreement < .8:
                reason = "outer_refit_normal_disagrees_with_initial_plane"
            else:
                refit = local
                report["normal_agreement_abs_dot"] = normal_agreement
        except Exception as exc:  # RANSAC failure is an explicitly reported fallback condition.
            reason = f"outer_refit_failed:{type(exc).__name__}:{exc}"
    if refit is None:
        if not bool(cfg["allow_plane_refit_fallback"]):
            raise RuntimeError(f"target board plane refit failed and fallback is disabled: {reason}")
        refit = initial
        report.update({"fit_point_source": "initial_target_roi_fallback", "fallback_used": True,
                       "fallback_reason": reason})
    report["selected_plane"] = refit.report
    report["selected_plane_model_ax_by_cz_d"] = _floats(refit.model)
    return refit, report


def select_target_object_heightmap(points_m: np.ndarray, initial_plane: PlaneFit, config: dict[str, Any], *,
                                   requested_side: Side | None = None,
                                   source_extent_m: np.ndarray | list[float] | None = None,
                                   high_threshold_override_m: float | None = None,
                                   plane_threshold_m: float, plane_ransac_n: int,
                                   plane_iterations: int, seed: int) -> dict[str, Any]:
    """Select a FAST target as a plane-local 2-D grown region of original point indices.

    The grid is used only for decisions. ``selected_indices`` always indexes ``points_m``;
    coordinates are never resampled, transformed, recentered, or normalized.
    """
    points = np.asarray(points_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("target heightmap requires at least three finite XYZ points")
    cfg = _validate_target_heightmap_config(config)
    selected_request = requested_side or str(cfg["side"])
    if selected_request not in {"auto", "positive", "negative"}:
        raise ValueError("requested target side must be auto, positive, or negative")

    plane_fit, refit_report = _target_plane_refit(
        points, initial_plane, cfg, threshold_m=float(plane_threshold_m),
        ransac_n=int(plane_ransac_n), iterations=int(plane_iterations), seed=int(seed))
    plane = plane_fit.model
    u, v = _plane_basis(plane)
    uv = np.column_stack((points @ u, points @ v))
    signed = signed_distances(points, plane)
    sigma = float(plane_fit.report["mad_robust_noise_sigma_m"])
    high_auto = max(float(cfg["high_threshold_minimum_m"]),
                    float(cfg["high_threshold_sigma_multiplier"]) * sigma)
    low = max(float(cfg["low_threshold_minimum_m"]),
              float(cfg["low_threshold_sigma_multiplier"]) * sigma)
    high = float(high_threshold_override_m) if high_threshold_override_m is not None else high_auto
    if not (0 < low < high):
        raise ValueError(f"illegal target height thresholds: require 0 < low ({low:g}) < high ({high:g})")
    thresholds = {
        "plane_sigma_m": sigma,
        "noise_formula": "1.4826 * MAD(abs(selected_board_plane_inlier_residual))",
        "high_threshold_m": high, "high_automatic_threshold_m": high_auto,
        "high_threshold_source": "cli_override" if high_threshold_override_m is not None else "automatic_robust_statistics",
        "high_threshold_sigma_multiplier": float(cfg["high_threshold_sigma_multiplier"]),
        "high_threshold_minimum_m": float(cfg["high_threshold_minimum_m"]),
        "low_threshold_m": low, "low_threshold_source": "automatic_robust_statistics",
        "low_threshold_sigma_multiplier": float(cfg["low_threshold_sigma_multiplier"]),
        "low_threshold_minimum_m": float(cfg["low_threshold_minimum_m"]),
        "validated_relation": "0 < low_threshold < high_threshold",
    }

    grid_size = float(cfg["grid_size_m"])
    grid_origin = np.floor(np.min(uv, axis=0) / grid_size) * grid_size
    ij = np.floor((uv - grid_origin) / grid_size).astype(np.int64)
    shape = tuple((np.max(ij, axis=0) + 1).tolist())
    if int(shape[0]) * int(shape[1]) > 10_000_000:
        raise ValueError(f"target heightmap grid is unreasonably large: {shape[0]}x{shape[1]}")
    flat = ij[:, 0] * shape[1] + ij[:, 1]
    order = np.argsort(flat, kind="stable")
    unique, starts, counts = np.unique(flat[order], return_index=True, return_counts=True)
    cell_groups = {int(cell): order[start:start + count] for cell, start, count in zip(unique, starts, counts)}
    roi_center = np.median(uv, axis=0)
    roi_extent = np.maximum(np.percentile(uv, 99, axis=0) - np.percentile(uv, 1, axis=0), grid_size)
    source_extent = None
    if source_extent_m is not None:
        source_extent = np.sort(np.asarray(source_extent_m, dtype=float).ravel())
        if source_extent.shape != (2,) or np.any(~np.isfinite(source_extent)) or np.any(source_extent <= 0):
            raise ValueError("source_extent_m must contain two positive finite projection extents")

    connectivity = int(cfg["connectivity"])
    structure = ndimage.generate_binary_structure(2, 2 if connectivity == 8 else 1)
    close_radius = int(cfg["morphology_close_radius_cells"])
    maximum_gap = int(cfg["maximum_gap_cells"])
    effective_close_radius = min(close_radius, int(np.ceil(maximum_gap / 2))) if maximum_gap else 0
    all_candidates: list[dict[str, Any]] = []
    side_debug: dict[str, Any] = {}
    candidate_id = 0
    sides_to_evaluate = ("positive", "negative") if selected_request == "auto" else (selected_request,)

    for side in sides_to_evaluate:
        depth = signed if side == "positive" else -signed
        occupied = np.zeros(shape, dtype=bool)
        allowed = np.zeros(shape, dtype=bool)
        strong = np.zeros(shape, dtype=bool)
        cell_depth = np.full(shape, np.nan, dtype=float)
        cell_median = np.full(shape, np.nan, dtype=float)
        cell_min = np.full(shape, np.nan, dtype=float)
        cell_max = np.full(shape, np.nan, dtype=float)
        minimum_cell_points = int(cfg["minimum_points_per_cell"])
        quantile = float(cfg["cell_depth_quantile"])
        for cell, indices in cell_groups.items():
            if len(indices) < minimum_cell_points:
                continue
            a, b = divmod(cell, shape[1])
            values = depth[indices]
            occupied[a, b] = True
            cell_depth[a, b] = float(np.quantile(values, quantile))
            cell_median[a, b] = float(np.median(values))
            cell_min[a, b] = float(np.min(values))
            cell_max[a, b] = float(np.max(values))
            allowed[a, b] = cell_depth[a, b] >= low
            strong[a, b] = cell_depth[a, b] >= high
        grown = allowed.copy()
        if effective_close_radius:
            close_structure = np.ones((2 * effective_close_radius + 1,) * 2, dtype=bool)
            grown |= ndimage.binary_closing(allowed, structure=close_structure)
        # FAST multi-return noise can put isolated above-threshold cells across nearly the
        # whole board. Strong cells remain fully reported, while a connected, deeper subset
        # anchors candidate growth. This suppresses isolated board returns without changing
        # the documented high/low classification or synthesizing output points.
        anchor_multiplier = float(cfg.get("strong_anchor_depth_multiplier", 2.0))
        minimum_anchor_cells = int(cfg.get("minimum_strong_anchor_cells", 3))
        if anchor_multiplier < 1:
            raise ValueError("target_heightmap.strong_anchor_depth_multiplier must be at least 1")
        if minimum_anchor_cells < 1:
            raise ValueError("target_heightmap.minimum_strong_anchor_cells must be positive")
        anchor = strong & (cell_depth >= anchor_multiplier * high)
        anchor_open_radius = int(cfg.get("strong_anchor_open_radius_cells", 1))
        if anchor_open_radius < 0:
            raise ValueError("target_heightmap.strong_anchor_open_radius_cells must be non-negative")
        if anchor_open_radius:
            anchor_structure = np.ones((2 * anchor_open_radius + 1,) * 2, dtype=bool)
            anchor = ndimage.binary_opening(anchor, structure=anchor_structure)
        anchor_labels, anchor_count = ndimage.label(anchor, structure=structure)
        anchor_sizes = np.bincount(anchor_labels.ravel(), minlength=anchor_count + 1)
        keep_anchor = np.flatnonzero(anchor_sizes >= minimum_anchor_cells)
        keep_anchor = keep_anchor[keep_anchor > 0]
        supported_strong = np.isin(anchor_labels, keep_anchor)
        if maximum_gap:
            reachable = ndimage.binary_dilation(supported_strong, structure=structure, iterations=maximum_gap)
        else:
            reachable = supported_strong
        candidate_growth = grown & reachable
        labels, component_count = ndimage.label(candidate_growth, structure=structure)
        strong_labels = set(int(x) for x in np.unique(labels[strong]) if x > 0)
        side_candidate_ids: list[int] = []
        for label in range(1, int(component_count) + 1):
            component_grid = labels == label
            has_strong = label in strong_labels
            actual_cells = np.argwhere(component_grid & allowed & occupied)
            if not len(actual_cells):
                continue
            point_parts: list[np.ndarray] = []
            for a, b in actual_cells:
                cell_points = cell_groups.get(int(a * shape[1] + b), np.empty(0, dtype=np.int64))
                point_parts.append(cell_points[depth[cell_points] >= low])
            indices = np.unique(np.concatenate(point_parts)) if point_parts else np.empty(0, dtype=np.int64)
            candidate_uv = uv[indices]
            extents = np.ptp(candidate_uv, axis=0) if len(indices) else np.zeros(2)
            sorted_extents = np.sort(extents)
            projection_area = float(np.prod(extents))
            aspect = float(sorted_extents[0] / max(sorted_extents[1], np.finfo(float).eps))
            cell_low, cell_high = actual_cells.min(axis=0), actual_cells.max(axis=0)
            bbox_cells = int(np.prod(cell_high - cell_low + 1))
            fill = float(len(actual_cells) / bbox_cells)
            boundary = ((actual_cells[:, 0] <= 0) | (actual_cells[:, 1] <= 0) |
                        (actual_cells[:, 0] >= shape[0] - 1) | (actual_cells[:, 1] >= shape[1] - 1))
            boundary_fraction = float(np.mean(boundary))
            center_distance = float(np.linalg.norm((np.median(candidate_uv, axis=0) - roi_center) / roi_extent)) if len(indices) else float("inf")
            depth_median = float(np.median(depth[indices])) if len(indices) else 0.
            depth_quantile = float(np.quantile(depth[indices], quantile)) if len(indices) else 0.
            extent_ratios = (sorted_extents / source_extent) if source_extent is not None else None
            source_match = (float(np.exp(-np.mean(np.abs(np.log(np.maximum(extent_ratios, 1e-12))))))
                            if extent_ratios is not None else None)
            reasons: list[str] = []
            if not has_strong: reasons.append("does_not_contain_strong_seed")
            if len(indices) < int(cfg["minimum_object_points"]): reasons.append("below_minimum_object_points")
            if len(actual_cells) < int(cfg["minimum_occupied_cells"]): reasons.append("below_minimum_occupied_cells")
            if sorted_extents[0] < float(cfg["minimum_projection_extent_m"]): reasons.append("projection_extent_below_minimum")
            if sorted_extents[1] > float(cfg["maximum_projection_extent_m"]): reasons.append("projection_extent_above_maximum")
            if not float(cfg["minimum_aspect_ratio"]) <= aspect <= float(cfg["maximum_aspect_ratio"]): reasons.append("aspect_ratio_out_of_range")
            if boundary_fraction > float(cfg["maximum_roi_boundary_fraction"]): reasons.append("roi_boundary_contact_exceeds_maximum")
            if depth_quantile > float(cfg["maximum_object_depth_m"]): reasons.append("object_depth_exceeds_maximum")
            if extent_ratios is not None and (np.any(extent_ratios < float(cfg["source_extent_minimum_ratio"])) or
                                               np.any(extent_ratios > float(cfg["source_extent_maximum_ratio"]))):
                reasons.append("source_projection_extent_mismatch")
            eligible = not reasons
            size_mid = .5 * (float(cfg["minimum_projection_extent_m"]) + float(cfg["maximum_projection_extent_m"]))
            generic_size = float(np.exp(-abs(float(np.mean(sorted_extents)) - size_mid) / max(size_mid, grid_size)))
            size_score = source_match if source_match is not None else generic_size
            depth_score = min(1., max(0., (depth_quantile - low) / max(high - low, np.finfo(float).eps)))
            support_score = min(1., len(indices) / max(4 * int(cfg["minimum_object_points"]), 1))
            cell_support_score = min(1., len(actual_cells) / max(4 * int(cfg["minimum_occupied_cells"]), 1))
            score = (3.0 * size_score + 2.0 * aspect + 1.2 * fill + 1.5 * depth_score +
                     .75 * (1. - boundary_fraction) + .75 * support_score + .75 * cell_support_score +
                     .25 / (1. + center_distance)) if eligible else 0.0
            candidate = {
                "component_id": candidate_id, "side": side, "label": int(label),
                "contains_strong_seed": bool(has_strong), "original_point_count": int(len(indices)),
                "occupied_cell_count": int(len(actual_cells)), "projection_extent_uv_m": _floats(extents),
                "sorted_projection_extent_m": _floats(sorted_extents), "projection_area_m2": projection_area,
                "aspect_ratio_short_over_long": aspect, "grid_fill_ratio": fill,
                "depth_median_m": depth_median, "depth_quantile_m": depth_quantile,
                "roi_boundary_contact_fraction": boundary_fraction,
                "roi_center_distance_normalized": center_distance,
                "projection_center_uv_m": _floats(np.median(candidate_uv, axis=0)) if len(indices) else None,
                "source_extent_ratio_sorted": _floats(extent_ratios) if extent_ratios is not None else None,
                "source_extent_match_score": source_match, "eligible": bool(eligible),
                "selection_score": float(score), "rejection_reasons": reasons,
                "decision": "eligible_candidate" if eligible else "rejected_candidate",
                "_indices": indices,
            }
            all_candidates.append(candidate)
            side_candidate_ids.append(candidate_id)
            candidate_id += 1
        side_debug[side] = {
            "occupied_cell_count": int(np.count_nonzero(occupied)),
            "strong_seed_cell_count": int(np.count_nonzero(strong)),
            "strong_seed_point_count": int(np.count_nonzero(depth >= high)),
            "spatially_supported_strong_seed_cell_count": int(np.count_nonzero(supported_strong)),
            "strong_anchor_depth_threshold_m": float(anchor_multiplier * high),
            "strong_anchor_depth_multiplier": anchor_multiplier,
            "minimum_strong_anchor_cells": minimum_anchor_cells,
            "strong_anchor_open_radius_cells": anchor_open_radius,
            "low_threshold_cell_count": int(np.count_nonzero(allowed)),
            "grown_cell_count": int(np.count_nonzero(grown)),
            "seed_bounded_grown_cell_count": int(np.count_nonzero(candidate_growth)),
            "connected_component_count": int(component_count),
            "candidate_component_ids": side_candidate_ids,
            "_strong_mask": strong, "_supported_strong_mask": supported_strong,
            "_raw_grown_mask": grown, "_grown_mask": candidate_growth, "_cell_depth": cell_depth,
            "_labels": labels,
        }

    eligible = sorted((c for c in all_candidates if c["eligible"]),
                      key=lambda c: (-c["selection_score"], -c["occupied_cell_count"], c["component_id"]))
    selected_component: dict[str, Any] | None = eligible[0] if eligible else None
    confirmed = selected_component is not None
    reason: str
    if not any(side_debug[s]["strong_seed_cell_count"] for s in side_debug):
        confirmed, reason = False, "no_strong_seed_cells"
    elif not any(side_debug[s]["spatially_supported_strong_seed_cell_count"] for s in side_debug):
        confirmed, reason = False, "no_spatially_supported_strong_seed_cells"
    elif not eligible:
        confirmed, reason = False, "no_component_satisfies_target_geometry_constraints"
    elif len(eligible) > 1:
        ratio = float(eligible[0]["selection_score"] / max(eligible[1]["selection_score"], np.finfo(float).eps))
        minimum_ratio = float(cfg.get("minimum_best_score_ratio", 1.08))
        if ratio < minimum_ratio:
            confirmed, reason = False, f"best_and_second_candidate_scores_too_close:{ratio:.3f}<{minimum_ratio:.3f}"
        else:
            reason = f"best_geometry_candidate_score_ratio:{ratio:.3f}"
    else:
        reason = "single_component_satisfies_heightmap_geometry_constraints"
    selected_indices = (np.asarray(selected_component["_indices"], dtype=np.int64)
                        if confirmed and selected_component is not None else np.empty(0, dtype=np.int64))
    if confirmed and (len(selected_indices) < int(cfg["minimum_object_points"]) or
                      int(selected_component["occupied_cell_count"]) < int(cfg["minimum_occupied_cells"])):
        confirmed, reason = False, "final_target_support_below_minimum"
        selected_indices = np.empty(0, dtype=np.int64)
    board_inliers = np.flatnonzero(np.abs(signed) <= float(plane_threshold_m))

    def public_candidate(candidate: dict[str, Any]) -> dict[str, Any]:
        return {key: value for key, value in candidate.items() if not key.startswith("_")}

    public_candidates = [public_candidate(c) for c in all_candidates]
    selected_public = public_candidate(selected_component) if selected_component is not None else None
    return {
        "requested_side": selected_request,
        "selected_side": selected_component["side"] if confirmed and selected_component is not None else None,
        "selection_mode": "target_heightmap" if confirmed else "needs_manual_confirmation",
        "selection_reason": reason, "selected_indices": selected_indices,
        "selected_point_count": int(len(selected_indices)), "confirmed": bool(confirmed),
        "thresholds": thresholds, "plane_refit": refit_report,
        "selected_plane_model": plane, "board_inlier_indices": board_inliers,
        "grid_statistics": {
            "grid_size_m": grid_size, "grid_origin_uv_m": _floats(grid_origin),
            "grid_shape": [int(shape[0]), int(shape[1])], "input_point_count": int(len(points)),
            "nonempty_cell_count": int(len(unique)), "minimum_points_per_cell": int(cfg["minimum_points_per_cell"]),
            "cell_depth_quantile": float(cfg["cell_depth_quantile"]), "connectivity": connectivity,
            "morphology_close_radius_cells": close_radius, "maximum_gap_cells": maximum_gap,
            "effective_close_radius_cells": effective_close_radius, "sides": {
                side: {k: v for k, v in stats.items() if not k.startswith("_")} for side, stats in side_debug.items()
            },
        },
        "candidate_components": public_candidates,
        "selected_component": selected_public,
        "rejected_components": [c for c in public_candidates if not c["eligible"]],
        "fallback_used": bool(refit_report["fallback_used"]),
        "output_contract": {
            "original_target_roi_index_subset": True, "coordinates_modified": False,
            "voxel_downsampling_performed": False, "registration_performed": False,
            "transform_estimated": False,
        },
        "sides": {side: {k: v for k, v in stats.items() if not k.startswith("_")} for side, stats in side_debug.items()},
        "_diagnostics": {"uv": uv, "signed_depth": signed, "grid_indices": ij,
                         "grid_shape": shape, "side_debug": side_debug},
    }


def coordinates_report(points: np.ndarray) -> dict[str, Any]:
    return _point_stats(np.asarray(points))


def exact_overlap_report(board_points: np.ndarray, object_points: np.ndarray) -> dict[str, Any]:
    """Report exact XYZ overlap without removing or changing either input."""
    board = np.ascontiguousarray(np.asarray(board_points, dtype=np.float64))
    obj = np.ascontiguousarray(np.asarray(object_points, dtype=np.float64))
    row_type = np.dtype((np.void, board.dtype.itemsize * 3))
    board_rows = board.view(row_type).ravel()
    object_rows = obj.view(row_type).ravel()
    overlap_count = int(len(np.intersect1d(board_rows, object_rows, assume_unique=False)))
    return {
        "comparison": "exact XYZ equality after documented non-finite/duplicate cleaning",
        "overlap_count": overlap_count,
        "board_overlap_ratio": float(overlap_count / len(board)) if len(board) else 0.0,
        "object_overlap_ratio": float(overlap_count / len(obj)) if len(obj) else 0.0,
        "points_removed": 0,
    }


def plane_projection_report(points_m: np.ndarray, plane: np.ndarray) -> dict[str, Any]:
    points = np.asarray(points_m, dtype=np.float64)
    normal = canonicalize_plane(plane)[:3]
    reference = np.array([0., 0., 1.]) if abs(normal[2]) < .9 else np.array([1., 0., 0.])
    u = np.cross(normal, reference); u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    uv = np.column_stack((points @ u, points @ v))
    low, high = uv.min(axis=0), uv.max(axis=0)
    extent = high - low
    return {
        "basis_u": _floats(u), "basis_v": _floats(v),
        "uv_min_m": _floats(low), "uv_max_m": _floats(high),
        "uv_extent_m": _floats(extent), "coverage_area_m2": float(np.prod(extent)),
    }


def coordinate_compatibility_report(points_m: np.ndarray, target_roi_m: np.ndarray,
                                    target_full_m: np.ndarray, *, full_margin_m: float,
                                    roi_margin_m: float, minimum_roi_fraction: float) -> dict[str, Any]:
    """Check FAST-metre coordinates against declared references without modifying them."""
    points = np.asarray(points_m, dtype=np.float64)
    roi = np.asarray(target_roi_m, dtype=np.float64)
    full = np.asarray(target_full_m, dtype=np.float64)

    def fraction_inside(reference: np.ndarray, margin: float) -> float:
        low, high = reference.min(axis=0) - margin, reference.max(axis=0) + margin
        return float(np.mean(np.all((points >= low) & (points <= high), axis=1)))

    full_fraction = fraction_inside(full, full_margin_m)
    roi_fraction = fraction_inside(roi, roi_margin_m)
    compatible = full_fraction == 1.0 and roi_fraction >= minimum_roi_fraction
    return {
        "compatible": bool(compatible), "unit_expected": "m", "coordinates_modified": False,
        "target_full_aabb_fraction": full_fraction,
        "target_roi_expanded_aabb_fraction": roi_fraction,
        "full_aabb_margin_m": float(full_margin_m), "roi_aabb_margin_m": float(roi_margin_m),
        "minimum_roi_fraction": float(minimum_roi_fraction),
        "decision_rule": "all points inside expanded target_full AABB and required fraction inside expanded target_roi AABB",
    }


def signed_distance_report(points_m: np.ndarray, plane: np.ndarray) -> dict[str, Any]:
    distances = signed_distances(points_m, plane)
    return {
        "interpretation": "geometric diagnostic only; not physical thickness and not used to rewrite the manual selection",
        "signed_distance_quantiles_m": {f"p{p}": float(np.percentile(distances, p)) for p in (0, 5, 10, 25, 50, 75, 90, 95, 100)},
        "positive_count": int(np.count_nonzero(distances > 0)),
        "negative_count": int(np.count_nonzero(distances < 0)),
        "zero_count": int(np.count_nonzero(distances == 0)),
    }


def manual_geometry_report(board_points_m: np.ndarray, object_points_m: np.ndarray,
                           target_roi_m: np.ndarray, plane_fit: PlaneFit, *, cluster_eps_m: float,
                           cluster_min_points: int, minimum_object_points: int,
                           minimum_projection_extent_m: float, maximum_boundary_fraction: float,
                           minimum_largest_component_ratio: float = 0.5) -> dict[str, Any]:
    """Diagnose manual clouds only; this function never edits or replaces user points."""
    board = np.asarray(board_points_m, dtype=np.float64)
    obj = np.asarray(object_points_m, dtype=np.float64)
    board_projection = plane_projection_report(board[plane_fit.inlier_indices], plane_fit.model)
    object_projection = plane_projection_report(obj, plane_fit.model)
    roi_projection = plane_projection_report(target_roi_m, plane_fit.model)

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(obj))
    labels = np.asarray(cloud.cluster_dbscan(float(cluster_eps_m), int(cluster_min_points), print_progress=False))
    component_sizes = sorted((int(np.count_nonzero(labels == label)) for label in set(labels.tolist()) if label >= 0), reverse=True)
    noise_count = int(np.count_nonzero(labels < 0))
    largest_ratio = float(component_sizes[0] / len(obj)) if component_sizes else 0.0

    normal = plane_fit.model[:3]
    u = np.asarray(object_projection["basis_u"]); v = np.asarray(object_projection["basis_v"])
    object_uv = np.column_stack((obj @ u, obj @ v))
    roi_uv = np.column_stack((np.asarray(target_roi_m) @ u, np.asarray(target_roi_m) @ v))
    roi_low, roi_high = np.percentile(roi_uv, [1, 99], axis=0)
    roi_extent = np.maximum(roi_high - roi_low, np.finfo(float).eps)
    edge_margin = .05 * roi_extent
    boundary = np.any((object_uv <= roi_low + edge_margin) | (object_uv >= roi_high - edge_margin), axis=1)
    boundary_fraction = float(np.mean(boundary))

    plane_extents = np.sort(np.asarray(board_projection["uv_extent_m"]))
    object_extents = np.sort(np.asarray(object_projection["uv_extent_m"]))
    board_degenerate = bool(plane_extents[0] < minimum_projection_extent_m)
    object_degenerate = bool(object_extents[0] < minimum_projection_extent_m)
    reasons: list[str] = []
    if len(obj) < minimum_object_points: reasons.append("manual_object_too_few_points")
    if not component_sizes: reasons.append("manual_object_has_no_connected_component")
    elif largest_ratio < minimum_largest_component_ratio: reasons.append("manual_object_connectivity_suspicious")
    if object_degenerate: reasons.append("manual_object_projection_degenerate")
    if boundary_fraction > maximum_boundary_fraction: reasons.append("manual_object_touches_roi_boundary_excessively")
    return {
        "board_plane_projection": board_projection,
        "board_plane_degenerate": board_degenerate,
        "object": {
            **object_projection, "point_count": int(len(obj)),
            "connected_component_count": int(len(component_sizes)), "component_sizes_descending": component_sizes,
            "dbscan_noise_count": noise_count, "largest_component_ratio": largest_ratio,
            "cluster_eps_m": float(cluster_eps_m), "cluster_min_points": int(cluster_min_points),
            "projection_degenerate": object_degenerate,
            "roi_boundary_contact_count": int(boundary.sum()), "roi_boundary_contact_fraction": boundary_fraction,
            "roi_boundary_definition": "outside inner 90% of target_roi P1-P99 plane-projection rectangle",
        },
        "target_roi_projection_reference": roi_projection,
        "signed_distance_to_manual_board_plane": signed_distance_report(obj, plane_fit.model),
        "suspicious_reasons": reasons,
        "points_modified_or_removed_by_diagnostics": 0,
    }
