"""Stage-1-only point-cloud diagnostics; no registration is performed here."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def _array(value: np.ndarray) -> list[float]:
    return [float(x) for x in value]


def coordinate_stats(points: np.ndarray, unit: str, abnormal_offset_ratio: float) -> dict[str, Any]:
    if len(points) == 0:
        return {"point_count": 0, "unit": unit, "available": False}
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    centroid = points.mean(axis=0)
    extent = maximum - minimum
    absolute = np.abs(points)
    max_abs = float(absolute.max())
    nonzero = absolute[absolute > 0]
    min_nonzero = float(nonzero.min()) if len(nonzero) else 0.0
    diagonal = float(np.linalg.norm(extent))
    offset_ratio = float(np.linalg.norm(centroid) / max(diagonal, np.finfo(float).eps))
    return {
        "available": True,
        "point_count": int(len(points)),
        "unit": unit,
        "centroid": _array(centroid),
        "xyz_min": _array(minimum),
        "xyz_max": _array(maximum),
        "axis_aligned_bounding_box_size": _array(extent),
        "bounding_box_diagonal": diagonal,
        "coordinate_absolute_range": [min_nonzero, max_abs],
        "coordinate_max_abs_order_of_magnitude": int(math.floor(math.log10(max_abs))) if max_abs > 0 else None,
        "centroid_to_bbox_diagonal_ratio": offset_ratio,
        "abnormal_coordinate_offset_suspected": offset_ratio >= abnormal_offset_ratio,
        "offset_heuristic_threshold": float(abnormal_offset_ratio),
    }


def clean_points(points: np.ndarray, duplicate_exact_limit: int) -> tuple[np.ndarray, dict[str, Any]]:
    nan_rows = np.isnan(points).any(axis=1)
    inf_rows = np.isinf(points).any(axis=1)
    finite = points[~(nan_rows | inf_rows)]
    if len(finite) <= duplicate_exact_limit:
        cleaned = np.unique(finite, axis=0)
        duplicate_count: int | None = int(len(finite) - len(cleaned))
        estimate = False
    else:
        # Deterministic evenly spaced sample; the report never presents this as an exact count.
        sample_indices = np.linspace(0, len(finite) - 1, duplicate_exact_limit, dtype=np.int64)
        sample = finite[sample_indices]
        unique_sample = np.unique(sample, axis=0)
        ratio = 1.0 - len(unique_sample) / len(sample)
        duplicate_count = None
        estimate = True
        cleaned = finite
    duplicate_ratio = (duplicate_count / len(finite)) if duplicate_count is not None and len(finite) else (
        ratio if estimate else 0.0
    )
    return cleaned, {
        "raw_point_count": int(len(points)),
        "finite_point_count": int(len(finite)),
        "valid_clean_point_count": int(len(cleaned)),
        "nan_row_count": int(nan_rows.sum()),
        "inf_row_count": int(inf_rows.sum()),
        "non_finite_row_count": int((nan_rows | inf_rows).sum()),
        "duplicate_point_count": duplicate_count,
        "duplicate_ratio": float(duplicate_ratio),
        "duplicate_measurement": "estimated_ratio_sample" if estimate else "exact",
        "cleaning_operations": ["remove_non_finite_rows", "remove_exact_duplicate_xyz"] if not estimate else [
            "remove_non_finite_rows", "duplicate_ratio_estimated; duplicates retained because exact limit was exceeded"
        ],
    }


def nearest_neighbor_stats(points: np.ndarray, sample_size: int, seed: int, unit: str) -> dict[str, Any]:
    if len(points) < 2:
        return {"available": False, "reason": "fewer than two points", "unit": unit}
    count = min(sample_size, len(points))
    rng = np.random.default_rng(seed)
    indices = rng.choice(len(points), count, replace=False)
    distances = cKDTree(points).query(points[indices], k=2, workers=1)[0][:, 1]
    return {
        "available": True,
        "unit": unit,
        "sample_size": int(count),
        "random_seed": int(seed),
        "p10": float(np.percentile(distances, 10)),
        "p50": float(np.percentile(distances, 50)),
        "p90": float(np.percentile(distances, 90)),
    }


def _canonical_plane(plane: np.ndarray) -> np.ndarray:
    normal = plane[:3]
    pivot = int(np.argmax(np.abs(normal)))
    if normal[pivot] < 0:
        plane = -plane
    return plane


def _side_stats(values: np.ndarray) -> dict[str, Any]:
    if len(values) == 0:
        return {"count": 0, "range_m": None, "quantiles_m": None}
    return {
        "count": int(len(values)),
        "range_m": [float(values.min()), float(values.max())],
        "quantiles_m": {f"p{p}": float(np.percentile(values, p)) for p in (5, 10, 25, 50, 75, 90, 95)},
    }


def plane_diagnostics(points_m: np.ndarray, *, threshold_m: float, ransac_n: int,
                      iterations: int, protrusion_sigma_multiplier: float,
                      protrusion_min_height_m: float, seed: int = 42) -> dict[str, Any]:
    if len(points_m) < ransac_n:
        return {"status": "not_available", "reason": "too few points"}
    o3d.utility.random.seed(seed)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points_m))
    model, indices = cloud.segment_plane(threshold_m, ransac_n, iterations)
    plane = _canonical_plane(np.asarray(model, dtype=np.float64))
    model_norm = float(np.linalg.norm(plane[:3]))
    plane = plane / model_norm
    normal = plane[:3]
    signed = points_m @ normal + plane[3]
    inlier_distances = np.abs(signed[np.asarray(indices, dtype=np.int64)])
    median = float(np.median(inlier_distances)) if len(inlier_distances) else 0.0
    mad = float(np.median(np.abs(inlier_distances - median))) if len(inlier_distances) else 0.0
    robust_sigma = 1.4826 * mad
    protrusion_threshold = max(protrusion_min_height_m, protrusion_sigma_multiplier * robust_sigma)
    positive = signed[signed > protrusion_threshold]
    negative = signed[signed < -protrusion_threshold]
    return {
        "status": "success",
        "interpretation": "dominant-plane candidate; confirm visually that this is the whiteboard",
        "plane_model_ax_by_cz_d": _array(plane),
        "normal_sign_rule": "the largest-absolute normal component is forced positive",
        "front_direction_status": "unknown_without_sensor_pose; both signed sides are reported",
        "ransac": {"distance_threshold_m": threshold_m, "ransac_n": ransac_n,
                   "num_iterations": iterations, "random_seed": seed},
        "plane_inlier_count": int(len(indices)),
        "plane_inlier_ratio": float(len(indices) / len(points_m)),
        "absolute_plane_residual_m": {
            "p50": float(np.percentile(inlier_distances, 50)),
            "p90": float(np.percentile(inlier_distances, 90)),
            "p95": float(np.percentile(inlier_distances, 95)),
        },
        "mad_robust_noise_sigma_m": robust_sigma,
        "protrusion_diagnostic_threshold_m": protrusion_threshold,
        "positive_side_protrusion": _side_stats(positive),
        "negative_side_protrusion": _side_stats(negative),
    }


def inspect_arrays(native_points: np.ndarray, metric_points: np.ndarray, *, native_unit: str,
                   config: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    cleaned_native, cleaning = clean_points(native_points, int(config["duplicate_exact_limit"]))
    finite_metric = metric_points[np.isfinite(metric_points).all(axis=1)]
    if len(native_points) <= int(config["duplicate_exact_limit"]):
        cleaned_metric = np.unique(finite_metric, axis=0)
    else:
        cleaned_metric = finite_metric
    report = {
        "raw_native_coordinates": coordinate_stats(native_points[np.isfinite(native_points).all(axis=1)], native_unit, config["abnormal_offset_ratio"]),
        "raw_metric_coordinates": coordinate_stats(finite_metric, "m", config["abnormal_offset_ratio"]),
        "nearest_neighbor_raw_metric": nearest_neighbor_stats(finite_metric, int(config["nearest_neighbor_sample_size"]), int(config["random_seed"]), "m"),
        "cleaning": cleaning,
        "cleaned_native_coordinates": coordinate_stats(cleaned_native, native_unit, config["abnormal_offset_ratio"]),
        "cleaned_metric_coordinates": coordinate_stats(cleaned_metric, "m", config["abnormal_offset_ratio"]),
        "nearest_neighbor_cleaned_metric": nearest_neighbor_stats(cleaned_metric, int(config["nearest_neighbor_sample_size"]), int(config["random_seed"]), "m"),
        "plane_and_protrusion_diagnostics": plane_diagnostics(
            cleaned_metric,
            threshold_m=float(config["plane_distance_threshold_m"]),
            ransac_n=int(config["plane_ransac_n"]),
            iterations=int(config["plane_num_iterations"]),
            protrusion_sigma_multiplier=float(config["protrusion_sigma_multiplier"]),
            protrusion_min_height_m=float(config["protrusion_min_height_m"]),
            seed=int(config["random_seed"]),
        ),
    }
    return cleaned_metric, report
