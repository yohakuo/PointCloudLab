#!/usr/bin/env python3
"""Stage 1: inspect INSPIRE 2 (source) and FAST-LIVO2 (target), without registration."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import platform
import sys
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.diagnostics import inspect_arrays  # noqa: E402
from pointcloud_registration.dataset_config import add_dataset_argument, parse_with_dataset  # noqa: E402
from pointcloud_registration.io_utils import (  # noqa: E402
    discover_candidates, read_header, read_points, sha256_file, validate_role_path,
)
from pointcloud_registration.reporting import configure_logging, write_json  # noqa: E402
from pointcloud_registration.units import inspire_mm_to_m  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="阶段 1：INSPIRE → FAST 数据检查（不执行配准）")
    add_dataset_argument(parser, ROOT)
    parser.add_argument("--source", "--source-full", dest="source", type=Path, help="INSPIRE 2 full .ply（原始单位 mm）")
    parser.add_argument("--target", "--target-full", dest="target", type=Path, help="FAST-LIVO2 full .pcd（单位 m）")
    parser.add_argument("--source-roi", type=Path, help="可选 INSPIRE 2 ROI .ply，仍保持原始全局坐标和 mm 单位")
    parser.add_argument("--target-roi", type=Path, help="可选 FAST-LIVO2 ROI .pcd，仍保持原始全局坐标和 m 单位")
    parser.add_argument("--source-roi-bounds-m", nargs=6, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="可选：从换算为米后的 source full 按 AABB 选取 ROI，不改变点坐标")
    parser.add_argument("--target-roi-bounds-m", nargs=6, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="可选：从 target full 按米制 AABB 选取 ROI，不改变点坐标")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--config", type=Path, default=ROOT / "configs" / "registration.yaml")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "stage_01_inspection")
    parser.add_argument("--list-candidates", action="store_true", help="只列出候选文件")
    args, _ = parse_with_dataset(parser, argv, ROOT, 1)
    return args


def load_config(path: Path) -> dict[str, Any]:
    # JSON is a strict YAML subset, keeping this stage dependency-free beyond pinned scientific packages.
    return json.loads(path.read_text(encoding="utf-8"))


def candidate_text(candidates: dict[str, list[Path]]) -> str:
    lines = ["发现的候选文件（存在多个时必须显式选择，程序不会猜测）："]
    for role, paths in candidates.items():
        lines.append(f"  {role}:")
        lines.extend(f"    - {path}" for path in paths) if paths else lines.append("    - <none>")
    return "\n".join(lines)


def inspect_file(path: Path, role: str, label: str, config: dict[str, Any],
                 bounds_m: list[float] | None = None) -> dict[str, Any]:
    cloud, native = read_points(path)
    metric = inspire_mm_to_m(native) if role == "source" else native.copy()
    selection: dict[str, Any] = {"method": "entire_file", "coordinates_modified": False}
    if bounds_m is not None:
        bounds = np.asarray(bounds_m, dtype=np.float64).reshape(3, 2)
        if np.any(bounds[:, 0] > bounds[:, 1]):
            raise ValueError(f"invalid {label} bounds: each MIN must be <= MAX")
        finite = np.isfinite(metric).all(axis=1)
        mask = finite & np.all((metric >= bounds[:, 0]) & (metric <= bounds[:, 1]), axis=1)
        if not np.any(mask):
            raise ValueError(f"{label} bounding box selected zero points")
        native, metric = native[mask].copy(), metric[mask].copy()
        selection = {"method": "axis_aligned_bounding_box_in_metres", "bounds_m": bounds_m,
                     "coordinates_modified": False, "selected_point_count": int(mask.sum())}
    _, diagnostics = inspect_arrays(native, metric, native_unit="mm" if role == "source" else "m", config=config)
    return {
        "label": label,
        "role": "source_INSPIRE_2" if role == "source" else "target_FAST_LIVO2",
        "path": path,
        "format": path.suffix.lower().lstrip("."),
        "native_unit": "mm" if role == "source" else "m",
        "working_unit": "m",
        "unit_conversion": "XYZ_m = XYZ_mm * 0.001 about origin (0,0,0)" if role == "source" else "none",
        "selection": selection,
        "file_size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "open3d_attributes": {"has_colors": cloud.has_colors(), "has_normals": cloud.has_normals()},
        "metadata": read_header(path),
        "diagnostics": diagnostics,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    candidates = discover_candidates(args.data_dir.resolve())
    print(candidate_text(candidates))
    if args.list_candidates:
        return 0
    if args.source is None or args.target is None:
        print("ERROR: 必须通过 --source 和 --target 明确选择 INSPIRE .ply 与 FAST .pcd。", file=sys.stderr)
        return 2
    if args.source_roi and args.source_roi_bounds_m:
        raise ValueError("choose either --source-roi or --source-roi-bounds-m, not both")
    if args.target_roi and args.target_roi_bounds_m:
        raise ValueError("choose either --target-roi or --target-roi-bounds-m, not both")
    source = validate_role_path(args.source, "source")
    target = validate_role_path(args.target, "target")
    source_roi = validate_role_path(args.source_roi, "source") if args.source_roi else None
    target_roi = validate_role_path(args.target_roi, "target") if args.target_roi else None
    config = load_config(args.config.resolve())
    project = config["project"]
    if project.get("transform_direction") != "INSPIRE_TO_FAST" or project.get("source_to_m_scale") != 0.001:
        raise ValueError("config must preserve INSPIRE_TO_FAST and source_to_m_scale=0.001")
    output_dir = args.output_dir.resolve()
    logger = configure_logging(output_dir / "stage_01_inspection.log")
    logger.info("Stage 1 started: source=INSPIRE 2, target=FAST-LIVO2; registration_performed=false")
    entries = [inspect_file(source, "source", "source_full", config["inspection"]),
               inspect_file(target, "target", "target_full", config["inspection"])]
    if source_roi:
        entries.append(inspect_file(source_roi, "source", "source_roi", config["inspection"]))
    elif args.source_roi_bounds_m:
        entries.append(inspect_file(source, "source", "source_roi_bbox", config["inspection"], args.source_roi_bounds_m))
    if target_roi:
        entries.append(inspect_file(target_roi, "target", "target_roi", config["inspection"]))
    elif args.target_roi_bounds_m:
        entries.append(inspect_file(target, "target", "target_roi_bbox", config["inspection"], args.target_roi_bounds_m))
    report = {
        "stage": 1,
        "stage_name": "data_inspection",
        "registration_performed": False,
        "transform_direction": "INSPIRE_TO_FAST",
        "coordinate_convention": "p_FAST_m = T_FAST_from_INSPIRE_m @ p_INSPIRE_m (T is not estimated in stage 1)",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "dataset": {"id": args.dataset_id, "manifest": str(args.dataset)},
        "software": {"python": sys.version, "platform": platform.platform(), "open3d": o3d.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "config": config,
        "inputs": entries,
        "warnings": [
            "Dominant-plane fits require visual confirmation that the plane is the whiteboard.",
            "Sensor front direction is unknown; positive and negative protrusion sides are both reported.",
            "CloudCompare Global Shift/Scale is never inferred when absent from file metadata.",
        ],
    }
    report_path = output_dir / "inspection_report.json"
    write_json(report_path, report)
    for entry in entries:
        diag = entry["diagnostics"]
        plane = diag["plane_and_protrusion_diagnostics"]
        logger.info("%s | raw=%d valid=%d duplicate_ratio=%.6g NN_P50_m=%s plane_inliers=%s",
                    entry["label"], diag["cleaning"]["raw_point_count"], diag["cleaning"]["valid_clean_point_count"],
                    diag["cleaning"]["duplicate_ratio"], diag["nearest_neighbor_cleaned_metric"].get("p50"),
                    plane.get("plane_inlier_count"))
    logger.info("Report saved: %s", report_path)
    logger.info("Stage 1 complete; no registration, segmentation, or transformed cloud was produced.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
