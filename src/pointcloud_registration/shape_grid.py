"""Density independent, observed support for stage 3 planar shape matching."""

from __future__ import annotations

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def build_shape_grid(frame, points, cfg, roi_bounds=None):
    """One vote per observed cell. Empty cells are unknown, including filled silhouette holes."""
    size = float(cfg["grid_size_m"])
    uv = np.asarray(frame["uv"], float)
    height = np.asarray(points) @ frame["n"] + float(frame["plane"][3])
    lo = np.floor((uv.min(axis=0) - size) / size) * size
    ij = np.floor((uv - lo) / size).astype(int)
    dims = tuple(np.max(ij, axis=0) + 3)
    if np.prod(dims) > 2_000_000:
        raise ValueError("shape grid exceeds 2 million cells; check units or grid size")
    unique, inverse, count = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    occupied = np.zeros(dims, bool)
    occupied[unique[:, 0], unique[:, 1]] = True
    centers = lo + (unique + .5) * size
    sorted_height = height[np.argsort(inverse, kind="stable")]
    groups = np.split(sorted_height, np.cumsum(count)[:-1])
    height_p25 = np.array([np.percentile(group, 25) for group in groups])
    height_p50 = np.array([np.median(group) for group in groups])
    height_p75 = np.array([np.percentile(group, 75) for group in groups])
    support = ndimage.uniform_filter(occupied.astype(float), size=3, mode="constant")
    # Sparse returns often include a broad, shallow halo. Retain its observed
    # cells, but derive the outline from a connected high-relief core.
    sparse = float(np.median(count)) < float(cfg["sparse_cell_count_threshold"])
    reliable = occupied.copy()
    height_threshold = None
    if sparse:
        height_threshold = float(np.percentile(height_p50, cfg["outline_height_quantile"]))
        high = np.zeros(dims, bool)
        high[unique[:, 0], unique[:, 1]] = height_p50 >= height_threshold
        joined = ndimage.binary_closing(high, structure=np.ones((3, 3), bool))
        labels, components = ndimage.label(joined)
        if components:
            sizes = np.bincount(labels.ravel()); sizes[0] = 0
            core = labels == int(np.argmax(sizes))
            if core.sum() >= 8:
                reliable = occupied & ndimage.binary_dilation(core, iterations=int(cfg["outline_core_dilation_cells"]))
    # Closing/filling estimates only the exterior silhouette, never occupancy.
    envelope = ndimage.binary_fill_holes(ndimage.binary_closing(
        reliable, structure=np.ones((3, 3), bool), border_value=0))
    edge = envelope & ~ndimage.binary_erosion(envelope)
    local_reliable = ndimage.uniform_filter(reliable.astype(float), size=3, mode="constant")
    near = ndimage.distance_transform_edt(~reliable) <= 1.5
    contour_ij = np.argwhere(edge & near & (local_reliable >= float(cfg["minimum_contour_support"])))
    if len(contour_ij) < 8:
        raise ValueError("too few observed, supported outer contour cells")
    contour = lo + (contour_ij + .5) * size
    # Point count saturates quickly; shallow sparse returns have lower weight.
    cell_support = support[unique[:, 0], unique[:, 1]]
    confidence = np.clip(count / float(cfg["confidence_saturation_points"]), 0, 1) * (.35 + .65 * cell_support)
    if sparse:
        lower = float(np.percentile(height_p50, 10))
        upper = max(float(np.percentile(height_p50, 90)), lower + 1e-6)
        confidence *= np.clip((height_p50 - lower) / (upper - lower), .1, 1)
    contour_confidence = np.clip(local_reliable[contour_ij[:, 0], contour_ij[:, 1]] * 2, .1, 1)
    crop = np.zeros(len(unique), bool)
    if roi_bounds is not None:
        low, high = np.asarray(roi_bounds[0]), np.asarray(roi_bounds[1])
        margin = float(cfg["crop_margin_m"])
        near_crop = np.any((np.asarray(points) - low <= margin) | (high - np.asarray(points) <= margin), axis=1)
        crop = np.bincount(inverse, weights=near_crop.astype(float), minlength=len(unique)) / count > .25
        confidence[crop] *= float(cfg["crop_confidence_factor"])
        contour_crop = crop[cKDTree(centers).query(contour, workers=1)[1]]
        contour_confidence[contour_crop] *= float(cfg["crop_confidence_factor"])
    return {"centers": centers, "contour": contour, "confidence": confidence,
            "contour_confidence": contour_confidence, "point_count": count,
            "height_p25_m": height_p25, "height_p50_m": height_p50, "height_p75_m": height_p75,
            "crop_cell": crop, "occupied": occupied, "envelope": envelope,
            "summary": {"grid_size_m": size, "observed_cells": len(unique),
                        "unknown_cells_in_grid": int(occupied.size - len(unique)),
                        "envelope_cells": int(envelope.sum()), "outer_contour_cells": len(contour),
                        "sparse_relief_outline": bool(sparse), "outline_height_threshold_m": height_threshold,
                        "reliable_outline_support_cells": int(reliable.sum()),
                        "crop_downweighted_cells": int(crop.sum()),
                        "point_count": int(len(points)), "height_reference": "signed distance from fitted board plane"}}


def export_shape_grid(grid):
    keys = ("centers", "contour", "confidence", "contour_confidence", "point_count",
            "height_p25_m", "height_p50_m", "height_p75_m", "crop_cell")
    return {"summary": grid["summary"], **{key: np.asarray(grid[key]).tolist() for key in keys},
            "empty_cell_policy": "unknown; not counted as empty object or matching penalty"}


def weighted_trimmed_distance(query, reference, weights, fraction, reference_weights=None, minimum_reference_confidence=0.):
    if reference_weights is not None:
        strong = reference_weights >= minimum_reference_confidence
        if np.any(strong):
            reference = reference[strong]
    distances = cKDTree(reference).query(query, workers=1)[0]
    order = np.argsort(distances, kind="stable")
    cumulative = np.cumsum(weights[order])
    keep = cumulative <= fraction * cumulative[-1]
    keep[0] = True
    return float(np.average(distances[order][keep], weights=weights[order][keep]))


def shape_score(source, target, angle_deg, shift, cfg):
    q = np.deg2rad(angle_deg)
    r = np.array([[np.cos(q), -np.sin(q)], [np.sin(q), np.cos(q)]])
    moved_cells = source["centers"] @ r.T + shift
    moved_contour = source["contour"] @ r.T + shift
    fraction = float(cfg["match_trim_fraction"])
    minimum = float(cfg["minimum_reference_confidence"])
    occupancy = .5 * (weighted_trimmed_distance(moved_cells, target["centers"], source["confidence"], fraction,
                                                target["confidence"], minimum)
                      + weighted_trimmed_distance(target["centers"], moved_cells, target["confidence"], fraction,
                                                  source["confidence"], minimum))
    contour = .5 * (weighted_trimmed_distance(moved_contour, target["contour"], source["contour_confidence"], fraction)
                    + weighted_trimmed_distance(target["contour"], moved_contour, target["contour_confidence"], fraction))
    # Height comparison is only between observed cells. Target return fusion makes it a weak cue.
    dist, index = cKDTree(target["centers"]).query(moved_cells, workers=1)
    near = dist <= 1.5 * float(cfg["grid_size_m"])
    if np.any(near):
        dh = np.abs(source["height_p50_m"][near] - target["height_p50_m"][index[near]])
        height = float(np.average(np.minimum(dh, float(cfg["height_clip_m"])), weights=source["confidence"][near]))
    else:
        height = float(cfg["height_clip_m"])
    total = (float(cfg["occupancy_weight"]) * occupancy
             + float(cfg["contour_weight"]) * contour
             + float(cfg["height_weight"]) * height)
    return total, {"occupancy_m": occupancy, "outer_contour_m": contour, "height_m": height}
