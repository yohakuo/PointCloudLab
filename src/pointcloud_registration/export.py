"""Stage-5 export and verification primitives (INSPIRE -> FAST only)."""

from __future__ import annotations

import csv
import json
import os
import shutil
import tempfile
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .diagnostics import coordinate_stats
from .io_utils import sha256_file
from .refinement import region_diagnostics
from .transforms import apply_transform, validate_rigid_transform


class ExportError(RuntimeError):
    pass


CORE_FILES = [
    "T_fast_from_inspire_m.txt", "T_fast_from_inspire_raw_mm.txt",
    "registered_inspire_full.ply", "merged_fast_inspire.ply", "registration_report.json",
]
VISUAL_FILES = [
    "before_registration.png", "after_registration.png", "top_view.png", "side_view.png",
    "distance_colormap.png", "segmentation_preview.png", "candidate_comparison.png",
]


def stage5_contract(parent_id: str, refined_id: str | None = None) -> dict[str, Any]:
    return {"stage": 5, "export_performed": True, "final_transform_selected": True,
            "selection_method": "explicit_user_selection", "selected_parent_candidate_id": parent_id,
            "selected_refined_candidate_id": refined_id, "automatic_scoring_performed": False,
            "automatic_ranking_performed": False, "ambiguity_adjudication_performed": False,
            "transform_uniqueness_determined": False}


def file_record(path: Path) -> dict[str, Any]:
    p = path.resolve()
    return {"path": str(p), "size_bytes": p.stat().st_size, "sha256": sha256_file(p)}


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ExportError(f"required JSON missing: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ExportError(f"cannot parse JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ExportError(f"JSON root must be an object: {path}")
    return value


def load_selected_refinement(refined_json: Path, summary_csv: Path, matrix_dir: Path,
                             parent_id: str, expected_refined_id: str | None = None,
                             atol: float = 1e-12, allow_nonvalid_selection: bool = False
                             ) -> tuple[dict[str, Any], np.ndarray, dict[str, Any]]:
    payload = load_json(refined_json)
    if payload.get("stage") != 4 or payload.get("status") != "success":
        raise ExportError("refined_candidates JSON is not a successful stage-4 artifact")
    if payload.get("transform_direction") != "INSPIRE_TO_FAST" or payload.get("unit") != "m":
        raise ExportError("stage-4 refined candidate direction/unit mismatch")
    matches = [x for x in payload.get("candidates", []) if x.get("parent_candidate_id") == parent_id]
    if len(matches) != 1:
        raise ExportError(f"parent candidate must map to exactly one refinement, got {len(matches)}: {parent_id}")
    chosen = matches[0]
    refined_id = chosen.get("refined_candidate_id")
    if expected_refined_id is not None and refined_id != expected_refined_id:
        raise ExportError(f"refined candidate mismatch: expected {expected_refined_id}, got {refined_id}")
    selected_status = chosen.get("status")
    if selected_status not in {"refined_valid", "no_safe_refinement", "refinement_failed"}:
        raise ExportError(f"selected refinement has unknown status: {selected_status}")
    safety_gate_overridden = selected_status != "refined_valid"
    if safety_gate_overridden and not allow_nonvalid_selection:
        raise ExportError(f"selected refinement is not refined_valid: {selected_status}")
    matrix = np.asarray(chosen.get("matrix_m"), dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ExportError("selected matrix_m is not 4x4")
    matrix_path = matrix_dir / f"{refined_id}.txt"
    if not matrix_path.is_file():
        raise ExportError(f"selected stage-4 matrix text missing: {matrix_path}")
    text_matrix = np.loadtxt(matrix_path, dtype=np.float64)
    if not np.array_equal(matrix, text_matrix):
        raise ExportError("stage-4 JSON matrix_m and matrix text are not elementwise identical")
    with summary_csv.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r.get("parent_candidate_id") == parent_id]
    if len(rows) != 1 or rows[0].get("refined_candidate_id") != refined_id:
        raise ExportError("refinement_summary.csv selected lineage is missing or non-unique")
    expected_rel = f"refined_matrices/{refined_id}.txt"
    if rows[0].get("matrix_file", "").replace("\\", "/") != expected_rel:
        raise ExportError("refinement_summary.csv matrix path does not name the selected refined matrix")
    rigid = validate_rigid_transform(matrix, atol=atol)
    if not rigid["valid"]:
        raise ExportError(f"selected metric matrix is not rigid: {rigid['reasons']}")
    audit = {
        "unique_parent_match": True, "refined_candidate_id": refined_id,
        "status": selected_status, "safety_gate_overridden": safety_gate_overridden,
        "allow_nonvalid_selection": allow_nonvalid_selection,
        "matrix_source": "matrix_m (parent_matrix_m explicitly forbidden)",
        "json_text_elementwise_identical": True, "summary_matrix_path_verified": True,
        "matrix_file": file_record(matrix_path), "matrix_parsed_values": matrix.tolist(),
        "rigid_validation": rigid,
    }
    return chosen, matrix, audit


def raw_mm_affine(metric_rigid: np.ndarray, scale: float = 0.001) -> np.ndarray:
    t = np.asarray(metric_rigid, dtype=np.float64)
    out = t @ np.diag([scale, scale, scale, 1.0])
    return out


def deterministic_indices(count: int, wanted: int, seed: int, include_boundary: bool = True) -> np.ndarray:
    if count <= 0:
        return np.empty(0, dtype=np.int64)
    fixed = np.array([0, count - 1], dtype=np.int64) if include_boundary else np.empty(0, dtype=np.int64)
    need = min(count, max(wanted, len(fixed)))
    if need == count:
        return np.arange(count, dtype=np.int64)
    rng = np.random.default_rng(seed)
    pool = np.setdiff1d(np.arange(count, dtype=np.int64), fixed, assume_unique=True)
    chosen = rng.choice(pool, need - len(fixed), replace=False)
    return np.unique(np.r_[fixed, chosen])


def clean_cloud(cloud: o3d.geometry.PointCloud, remove_duplicates: bool = True) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    points = np.asarray(cloud.points, dtype=np.float64)
    finite = np.isfinite(points).all(axis=1)
    indices = np.flatnonzero(finite)
    duplicate_count = 0
    if remove_duplicates and len(indices):
        _, first = np.unique(points[indices], axis=0, return_index=True)
        keep = np.sort(first)
        duplicate_count = len(indices) - len(keep)
        indices = indices[keep]
    out = cloud.select_by_index(indices.tolist())
    return out, {
        "raw_point_count": int(len(points)), "finite_point_count": int(finite.sum()),
        "valid_point_count": int(len(indices)), "non_finite_removed": int((~finite).sum()),
        "exact_duplicate_xyz_removed": int(duplicate_count),
        "strategy": "remove non-finite rows, then remove exact duplicate XYZ while retaining first point attributes",
    }


def transform_source_cloud(source: o3d.geometry.PointCloud, matrix_m: np.ndarray,
                           scale: float = 0.001) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    source, cleaning = clean_cloud(source, remove_duplicates=True)
    p_raw = np.asarray(source.points, dtype=np.float64).copy()
    p_m = p_raw * scale
    p_fast = apply_transform(p_m, matrix_m)
    out = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p_fast))
    if source.has_colors():
        out.colors = o3d.utility.Vector3dVector(np.asarray(source.colors, dtype=np.float64).copy())
    if source.has_normals():
        normals = np.asarray(source.normals, dtype=np.float64) @ np.asarray(matrix_m)[:3, :3].T
        lengths = np.linalg.norm(normals, axis=1)
        valid = np.isfinite(normals).all(axis=1) & (lengths > 0)
        if not valid.all():
            raise ExportError("source contains invalid normals after rotation")
        normals /= lengths[:, None]
        out.normals = o3d.utility.Vector3dVector(normals)
    return out, {
        "cleaning": cleaning, "unit_conversion_count": 1,
        "unit_conversion": "XYZ only: raw INSPIRE mm * 0.001 about origin before rigid transform",
        "normal_policy": "rotate by R only and renormalize; no scale or translation",
        "color_policy": "Open3D RGB copied point-for-point without modification",
        "unsupported_property_policy": "properties not represented by Open3D PointCloud (for example PLY alpha) are not exported",
    }


def merge_clouds(target: o3d.geometry.PointCloud, registered: o3d.geometry.PointCloud) -> tuple[o3d.geometry.PointCloud, dict[str, Any]]:
    target, target_clean = clean_cloud(target, remove_duplicates=True)
    points = np.vstack([np.asarray(target.points), np.asarray(registered.points)])
    merged = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    color_kept = target.has_colors() and registered.has_colors()
    if color_kept:
        merged.colors = o3d.utility.Vector3dVector(np.vstack([np.asarray(target.colors), np.asarray(registered.colors)]))
    normal_kept = target.has_normals() and registered.has_normals()
    if normal_kept:
        merged.normals = o3d.utility.Vector3dVector(np.vstack([np.asarray(target.normals), np.asarray(registered.normals)]))
    return merged, {
        "order": "untransformed FAST target, then registered INSPIRE source",
        "target_cleaning": target_clean, "target_coordinates_transformed": False,
        "colors": "preserved because both inputs have RGB" if color_kept else "omitted because not present on both inputs",
        "normals": "preserved because present on both inputs" if normal_kept else "omitted because not present on both inputs",
    }


def cloud_stats(cloud: o3d.geometry.PointCloud) -> dict[str, Any]:
    points = np.asarray(cloud.points, dtype=np.float64)
    if not len(points) or not np.isfinite(points).all():
        raise ExportError("cloud is empty or contains non-finite XYZ")
    return {
        "point_count": int(len(points)), "all_xyz_finite": True,
        "centroid_m": points.mean(axis=0).tolist(), "xyz_min_m": points.min(axis=0).tolist(),
        "xyz_max_m": points.max(axis=0).tolist(), "aabb_extent_m": np.ptp(points, axis=0).tolist(),
        "has_colors": cloud.has_colors(), "has_normals": cloud.has_normals(),
    }


def atomic_write_matrix(path: Path, matrix: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.stem + ".", suffix=".txt", dir=path.parent)
    os.close(fd)
    tmp = Path(name)
    try:
        np.savetxt(tmp, np.asarray(matrix), fmt="%.17g")
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def atomic_write_cloud(path: Path, cloud: o3d.geometry.PointCloud) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.stem + ".", suffix=path.suffix, dir=path.parent)
    os.close(fd); tmp = Path(name)
    try:
        if not o3d.io.write_point_cloud(str(tmp), cloud, write_ascii=False, compressed=False):
            raise ExportError(f"Open3D failed to write point cloud: {path}")
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def verify_cloud_roundtrip(path: Path, expected: o3d.geometry.PointCloud, atol_m: float,
                           sample_count: int, seed: int) -> dict[str, Any]:
    loaded = o3d.io.read_point_cloud(str(path), remove_nan_points=False, remove_infinite_points=False)
    a, b = np.asarray(expected.points), np.asarray(loaded.points)
    if len(a) != len(b) or not np.isfinite(b).all():
        raise ExportError(f"point count or finiteness failed after reread: {path}")
    ids = deterministic_indices(len(a), sample_count, seed)
    error = np.linalg.norm(a[ids] - b[ids], axis=1)
    if np.max(error, initial=0.0) > atol_m:
        raise ExportError(f"saved XYZ sample differs after reread: {path}; max={error.max()}")
    if expected.has_colors() != loaded.has_colors() or expected.has_normals() != loaded.has_normals():
        raise ExportError(f"attribute presence changed after reread: {path}")
    return {"passed": True, "sample_count": int(len(ids)), "maximum_xyz_error_m": float(np.max(error, initial=0.0)),
            "tolerance_m": atol_m, "reread_stats": cloud_stats(loaded)}


def verify_transform_paths(points_raw_mm: np.ndarray, matrix_m: np.ndarray, matrix_raw: np.ndarray,
                           atol_m: float, sample_count: int, seed: int) -> dict[str, Any]:
    ids = deterministic_indices(len(points_raw_mm), sample_count, seed)
    sample = np.asarray(points_raw_mm)[ids]
    a = apply_transform(sample * 0.001, matrix_m)
    b = apply_transform(sample, matrix_raw)
    errors = np.linalg.norm(a - b, axis=1)
    maximum = float(np.max(errors, initial=0.0))
    if maximum > atol_m:
        raise ExportError(f"metric and raw-mm transform paths differ: {maximum} > {atol_m}")
    return {"passed": True, "definition_a": "T_m @ scale_mm_to_m", "definition_b": "T_raw_mm",
            "sample_includes_first_and_last_boundary_points": True, "sample_count": int(len(ids)),
            "maximum_error_m": maximum, "tolerance_m": atol_m}


def verify_roi_correspondence(source_full_raw_mm: np.ndarray, source_roi_m: np.ndarray,
                              matrix_m: np.ndarray, tolerance_m: float,
                              sample_count: int, seed: int) -> dict[str, Any]:
    full_m = np.asarray(source_full_raw_mm) * 0.001
    ids = deterministic_indices(len(source_roi_m), sample_count, seed)
    roi = np.asarray(source_roi_m)[ids]
    distances, matches = cKDTree(full_m).query(roi, workers=1)
    if float(np.max(distances, initial=0.0)) > tolerance_m:
        raise ExportError("stage-2 source ROI sample cannot be matched to the metric full INSPIRE cloud")
    direct = apply_transform(roi, matrix_m)
    via_full = apply_transform(full_m[matches], matrix_m)
    error = np.linalg.norm(direct - via_full, axis=1)
    if float(np.max(error, initial=0.0)) > tolerance_m:
        raise ExportError("stage-2 ROI and corresponding full-cloud transform results differ")
    return {"passed": True, "sample_count": int(len(ids)), "maximum_input_correspondence_error_m": float(np.max(distances, initial=0.0)),
            "maximum_transformed_error_m": float(np.max(error, initial=0.0)), "tolerance_m": tolerance_m,
            "same_scale_and_matrix": True}


def _sample(points: np.ndarray, n: int, seed: int) -> np.ndarray:
    ids = deterministic_indices(len(points), n, seed, include_boundary=False)
    return np.asarray(points)[ids]


def _scatter_pair(path: Path, a: np.ndarray, b: np.ndarray, labels: tuple[str, str], title: str,
                  axes: tuple[int, int], colors: tuple[str, str], n: int, seed: int, dpi: int) -> None:
    aa, bb = _sample(a, n, seed), _sample(b, n, seed + 1)
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(bb[:, axes[0]], bb[:, axes[1]], s=0.35, c=colors[1], alpha=.35, label=labels[1], rasterized=True)
    ax.scatter(aa[:, axes[0]], aa[:, axes[1]], s=0.45, c=colors[0], alpha=.55, label=labels[0], rasterized=True)
    names = ("X", "Y", "Z")
    ax.set_xlabel(f"FAST {names[axes[0]]} (m)"); ax.set_ylabel(f"FAST {names[axes[1]]} (m)")
    ax.set_title(title); ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2); ax.legend(markerscale=8)
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    fig.tight_layout(); fig.savefig(temporary, dpi=dpi); plt.close(fig); os.replace(temporary, path)


def create_visualizations(out: Path, source_before_m: np.ndarray, registered: np.ndarray, target: np.ndarray,
                          source_object_registered: np.ndarray, target_object: np.ndarray,
                          cfg: dict[str, Any], segmentation_preview: Path, candidate_comparison: Path,
                          selected_lineage: str) -> dict[str, Any]:
    n, seed, dpi = int(cfg["visualization_sample_count"]), int(cfg["random_seeds"]["visualization"]), int(cfg["image_dpi"])
    _scatter_pair(out/"before_registration.png", source_before_m, target, ("INSPIRE (m, before rigid)", "FAST (unchanged)"),
                  "Before registration — deterministic sample", (0, 2), ("#d95f02", "#377eb8"), n, seed, dpi)
    _scatter_pair(out/"after_registration.png", registered, target, ("INSPIRE registered", "FAST (unchanged)"),
                  f"After registration — {selected_lineage}", (0, 2), ("#e41a1c", "#377eb8"), n, seed, dpi)
    _scatter_pair(out/"top_view.png", registered, target, ("INSPIRE registered", "FAST (unchanged)"),
                  "Top view in FAST frame: +X right, +Y up", (0, 1), ("#e41a1c", "#377eb8"), n, seed, dpi)
    _scatter_pair(out/"side_view.png", registered, target, ("INSPIRE registered", "FAST (unchanged)"),
                  "Side view in FAST frame: +Y right, +Z up", (1, 2), ("#e41a1c", "#377eb8"), n, seed, dpi)
    ids = deterministic_indices(len(source_object_registered), int(cfg["distance_colormap_sample_count"]), seed)
    pts = np.asarray(source_object_registered)[ids]
    distances = cKDTree(target_object).query(pts, workers=1)[0]
    clip = float(cfg["distance_colormap_clip_m"])
    fig, ax = plt.subplots(figsize=(8, 7)); sc = ax.scatter(pts[:,0], pts[:,1], c=np.minimum(distances, clip)*1000,
        s=2, cmap="viridis", vmin=0, vmax=clip*1000, rasterized=True)
    ax.set_title("Object region source→target nearest distance (clipped)")
    ax.set_xlabel("FAST X (m)"); ax.set_ylabel("FAST Y (m)"); ax.set_aspect("equal", adjustable="box")
    cb=fig.colorbar(sc,ax=ax);cb.set_label(f"distance (mm), clipped at {clip*1000:g} mm")
    distance_path=out/"distance_colormap.png";temporary=distance_path.with_name(distance_path.stem+".tmp"+distance_path.suffix)
    fig.tight_layout();fig.savefig(temporary,dpi=dpi);plt.close(fig);os.replace(temporary,distance_path)
    provenance={}
    for source,name in ((segmentation_preview,"segmentation_preview.png"),(candidate_comparison,"candidate_comparison.png")):
        if not source.is_file(): raise ExportError(f"required provenance visualization missing: {source}")
        temporary=(out/name).with_name(Path(name).stem+".tmp"+Path(name).suffix)
        shutil.copyfile(source,temporary);os.replace(temporary,out/name)
        if sha256_file(source)!=sha256_file(out/name): raise ExportError(f"byte-copy hash mismatch: {name}")
        provenance[name]={"method":"byte-for-byte copy","source":file_record(source),"copied_sha256":sha256_file(out/name)}
    return {"fixed_camera":True,"coordinate_unit":"m","sampling_seed":seed,"sample_limit_per_cloud":n,
            "distance_colormap":{"definition":"registered source object -> target object nearest-neighbour distance",
              "region":"stage-2 canonical object regions only; full-scene background excluded","clip_m":clip,"sample_count":int(len(ids))},
            "copied_provenance":provenance,"candidate_comparison_interpretation":f"stage-3 display only; explicit selection {selected_lineage}; no automatic ranking"}
