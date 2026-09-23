"""Input discovery, point-cloud loading, and header metadata extraction."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

SUPPORTED_SUFFIXES = {".pcd", ".ply"}


def load_metric_stage2_cloud(path: Path) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    """Read a canonical stage-2 cloud already expressed in metres."""
    cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=False, remove_infinite_points=False)
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise RuntimeError(f"empty or invalid stage 2 cloud: {path}")
    if not np.isfinite(points).all():
        raise RuntimeError(f"stage 2 canonical input contains NaN/Inf: {path}")
    return cloud, points.copy()


def discover_candidates(data_dir: Path) -> dict[str, list[Path]]:
    files = [p.resolve() for p in data_dir.rglob("*") if p.is_file()]
    return {
        "target_fast_livo2_pcd": sorted(p for p in files if p.suffix.lower() == ".pcd"),
        "source_inspire_2_ply": sorted(p for p in files if p.suffix.lower() == ".ply"),
    }


def validate_role_path(path: Path, role: str) -> Path:
    resolved = path.expanduser().resolve()
    expected = ".ply" if role == "source" else ".pcd"
    if not resolved.is_file():
        raise FileNotFoundError(f"{role} file does not exist: {resolved}")
    if resolved.suffix.lower() != expected:
        raise ValueError(f"{role} must be {expected} ({'INSPIRE 2' if role == 'source' else 'FAST-LIVO2'}), got: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_header(path: Path) -> dict[str, Any]:
    lines: list[str] = []
    with path.open("rb") as stream:
        for _ in range(2048):
            raw = stream.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            lines.append(line)
            if (path.suffix.lower() == ".ply" and line.strip().lower() == "end_header") or (
                path.suffix.lower() == ".pcd" and line.strip().upper().startswith("DATA ")
            ):
                break
    comments = [line for line in lines if line.lstrip().lower().startswith(("comment", "#", "obj_info"))]
    global_lines = [line for line in lines if "global" in line.lower() and ("shift" in line.lower() or "scale" in line.lower())]
    encoding = "unknown"
    declared_points: int | None = None
    fields: list[str] = []
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        key = parts[0].lower()
        if key == "format" and len(parts) > 1:
            encoding = parts[1]
        elif key == "data" and len(parts) > 1:
            encoding = parts[1]
        elif key in {"points", "element"} and len(parts) > 1:
            if key == "points" or (len(parts) > 2 and parts[1].lower() == "vertex"):
                declared_points = int(parts[-1])
        elif key in {"fields", "property"}:
            fields.extend(parts[1:] if key == "fields" else [parts[-1]])
    return {
        "encoding": encoding,
        "declared_point_count": declared_points,
        "fields_or_properties": fields,
        "comments": comments,
        "header_text": "\n".join(lines),
        "cloudcompare_global_shift_scale": {
            "status": "metadata_present_requires_manual_interpretation" if global_lines else "unknown / not recoverable from current file",
            "matching_header_lines": global_lines,
        },
    }


def read_points(path: Path) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=False, remove_infinite_points=False)
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise RuntimeError(f"Open3D returned invalid point shape {points.shape} for {path}")
    if len(points) == 0:
        raise RuntimeError(f"no points were read from {path}")
    return cloud, points.copy()
