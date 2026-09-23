#!/usr/bin/env python3
"""Stage 2: board/object segmentation and manual target validation. No registration."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import platform
import sys
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import open3d as o3d
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.dataset_config import add_dataset_argument, parse_with_dataset  # noqa: E402
from pointcloud_registration.io_utils import read_header, read_points, sha256_file, validate_role_path  # noqa: E402
from pointcloud_registration.reporting import configure_logging, write_json  # noqa: E402
from pointcloud_registration.segmentation import (  # noqa: E402
    automatic_target_selection_allowed, choose_object_side, clean_cloud, coordinate_compatibility_report, coordinates_report,
    exact_overlap_report, fit_plane_ransac, manual_geometry_report, plane_projection_report,
    select_target_object_heightmap, threshold_from_plane,
)
from pointcloud_registration.visualization import (save_manual_segmentation_preview, save_segmentation_preview,
                                                    save_target_heightmap_preview)  # noqa: E402


class Stage2Failure(RuntimeError):
    def __init__(self, reason: str, message: str) -> None:
        super().__init__(message)
        self.reason = reason


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="阶段 2：INSPIRE/FAST 白板与中央共同物体分割（不执行配准）")
    add_dataset_argument(parser, ROOT)
    parser.add_argument("--source-full", type=Path, help="INSPIRE full PLY，source 白板拟合的唯一来源")
    parser.add_argument("--target-full", type=Path, help="FAST full PCD，仅作显式坐标相容性验证")
    parser.add_argument("--source-roi", type=Path, help="INSPIRE 中央共同物体候选 ROI")
    parser.add_argument("--target-roi", type=Path, help="FAST 中央区域坐标相容性与边界参考")
    parser.add_argument("--target-board-file", type=Path, help="FAST 米制手工白板 PCD；须与 --target-object-file 同时提供")
    parser.add_argument("--target-object-file", type=Path, help="FAST 米制手工对象 PCD；须与 --target-board-file 同时提供")
    parser.add_argument("--target-selection-mode", choices=("automatic", "manual_files"), help="target 选择模式；默认读取配置")
    parser.add_argument("--confirm-target-manual-preview", action="store_true", help="确认已检查本次手工 target 预览；不能覆盖硬错误或几何警告")
    parser.add_argument("--config", type=Path, default=ROOT / "configs" / "registration.yaml")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "stage_02_segmentation")
    parser.add_argument("--source-object-threshold-m", type=float)
    parser.add_argument("--target-object-threshold-m", type=float, help="仅 automatic target 模式使用")
    parser.add_argument("--source-side", choices=("auto", "positive", "negative"))
    parser.add_argument("--target-side", choices=("auto", "positive", "negative"), help="仅 automatic target 模式使用")
    parser.add_argument("--seed", type=int)
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    args, _ = parse_with_dataset(build_parser(), argv, ROOT, 2)
    return args


def _load_config(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def resolve_manual_target_paths(args: argparse.Namespace, cfg: dict[str, Any]) -> tuple[str, Path | None, Path | None, str]:
    cli_board, cli_object = args.target_board_file, args.target_object_file
    if (cli_board is None) != (cli_object is None):
        raise Stage2Failure("manual_target_file_pair_incomplete", "--target-board-file and --target-object-file must be provided together")
    configured = cfg.get("target_manual_files", {})
    mode = args.target_selection_mode or cfg.get("target_selection_mode", "automatic")
    if cli_board is not None:
        if args.target_selection_mode == "automatic":
            raise Stage2Failure("manual_target_mode_conflict", "manual target files cannot be combined with --target-selection-mode automatic")
        return "manual_files", cli_board, cli_object, "cli"
    board_value, object_value = configured.get("board_file"), configured.get("object_file")
    if (board_value is None) != (object_value is None):
        raise Stage2Failure("manual_target_file_pair_incomplete", "configured target manual board/object paths must both be set")
    if mode == "manual_files" and board_value is None:
        raise Stage2Failure("manual_target_files_missing", "manual_files mode requires both target manual file paths")
    return mode, (Path(board_value) if board_value else None), (Path(object_value) if object_value else None), "config"


def _input_record(path: Path, role: str, responsibility: str, used_for: list[str], cloud: o3d.geometry.PointCloud,
                  cleaning: dict[str, Any], points: np.ndarray) -> dict[str, Any]:
    header = read_header(path)
    return {
        "path": str(path), "format": path.suffix.lower().lstrip("."),
        "native_unit": "mm" if role == "source" else "m", "working_unit": "m",
        "file_size_bytes": path.stat().st_size, "sha256": sha256_file(path),
        "responsibility": responsibility, "actually_used_for": used_for,
        "fields": header.get("fields_or_properties", []),
        "attributes": {"colors_preserved": bool(cloud.has_colors()), "normals_preserved": bool(cloud.has_normals())},
        "header": header, "cleaning": cleaning, "coordinates": coordinates_report(points),
    }


def _write_cloud(cloud: o3d.geometry.PointCloud, path: Path) -> dict[str, Any]:
    points_before = np.asarray(cloud.points).copy()
    if not len(points_before):
        raise Stage2Failure("empty_output_cloud", f"refusing to write empty point cloud: {path.name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=False, print_progress=False):
        raise OSError(f"Open3D failed to save {path}")
    reread = o3d.io.read_point_cloud(str(path))
    points_after = np.asarray(reread.points)
    if len(points_after) != len(points_before) or not np.array_equal(points_after, points_before):
        raise OSError(f"saved points differ from preserved input coordinates for {path}")
    return {"path": str(path.resolve()), "sha256": sha256_file(path), **coordinates_report(points_after),
            "reread_verified": True, "coordinates_modified": False}


def _write_subset(cloud: o3d.geometry.PointCloud, indices: np.ndarray, path: Path) -> dict[str, Any]:
    return _write_cloud(cloud.select_by_index(np.asarray(indices, dtype=np.int64).tolist()), path)


def _serializable_selection(selection: dict[str, Any]) -> dict[str, Any]:
    excluded = {"selected_indices", "raw_candidate_indices", "best_cluster_indices", "board_inlier_indices"}

    def convert(value: Any) -> Any:
        if isinstance(value, np.ndarray): return value.tolist()
        if isinstance(value, np.generic): return value.item()
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()
                    if key not in excluded and not key.startswith("_")}
        if isinstance(value, (list, tuple)): return [convert(item) for item in value]
        return value

    clean = convert(selection)
    clean["selected_point_count"] = int(len(selection["selected_indices"]))
    return clean


def manual_identity_report(board_path: Path, object_path: Path, board: np.ndarray, obj: np.ndarray) -> dict[str, Any]:
    board_hash, object_hash = sha256_file(board_path), sha256_file(object_path)
    return {
        "board_sha256": board_hash, "object_sha256": object_hash,
        "sha256_identical": board_hash == object_hash,
        "cleaned_xyz_identical": bool(board.shape == obj.shape and np.array_equal(board, obj)),
    }


def validate_distinct_manual_paths(board_path: Path, object_path: Path) -> None:
    if board_path.expanduser().resolve() == object_path.expanduser().resolve():
        raise Stage2Failure("manual_target_files_same_path", "manual target board and object paths resolve to the same file")


def _load_cloud(path: Path, label: str, scale: float) -> tuple[o3d.geometry.PointCloud, np.ndarray, dict[str, Any]]:
    try:
        native, _ = read_points(path)
    except Exception as exc:
        reason = "manual_target_cloud_empty_or_unreadable" if label.startswith("target_manual") else "input_cloud_empty_or_unreadable"
        raise Stage2Failure(reason, f"cannot read {label} from {path}: {exc}") from exc
    cloud, points, cleaning = clean_cloud(native, scale)
    if not len(points):
        raise Stage2Failure("manual_target_cloud_empty_after_cleaning", f"{label} is empty after non-finite/duplicate cleaning")
    return cloud, points, cleaning


def _select_source(source_roi: np.ndarray, source_plane: Any, cfg: dict[str, Any], args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    threshold, threshold_report = threshold_from_plane(
        source_plane.report, sigma_multiplier=float(cfg["source_object"]["threshold_sigma_multiplier"]),
        minimum_m=float(cfg["source_object"]["minimum_threshold_m"]), override_m=args.source_object_threshold_m)
    scfg = cfg["source_object"]
    selection = choose_object_side(source_roi, source_plane.model, threshold,
        requested_side=args.source_side or scfg["side"], eps_m=float(scfg["cluster_eps_m"]),
        min_points=int(scfg["cluster_min_points"]), min_cluster_points=int(scfg["minimum_cluster_points"]),
        max_extent_m=float(scfg["maximum_object_extent_m"]), centrality_limit=float(scfg["centrality_limit"]),
        auto_score_ratio=float(scfg["auto_score_ratio"]))
    return selection, threshold_report


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output_dir = args.output_dir.resolve()
    logger = configure_logging(output_dir / "stage_02_segmentation.log", "pointcloud_registration.stage2", file_mode="w")
    report_path = output_dir / "segmentation_report.json"
    report: dict[str, Any] = {
        "stage": 2, "stage_name": "region_segmentation", "status": "failed", "failure_reason": None,
        "registration_performed": False, "transform_estimated": False, "transform_direction": "INSPIRE_TO_FAST",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(), "warnings": [], "failure_reasons": [],
        "dataset": {"id": args.dataset_id, "manifest": str(args.dataset)},
    }
    try:
        config_path = args.config.resolve(); config = _load_config(config_path)
        project, cfg = config["project"], config["segmentation"]
        if project.get("transform_direction") != "INSPIRE_TO_FAST" or float(project.get("source_to_m_scale")) != .001:
            raise Stage2Failure("invalid_stage_contract", "config must preserve INSPIRE_TO_FAST and source_to_m_scale=0.001")
        mode, manual_board_arg, manual_object_arg, manual_path_source = resolve_manual_target_paths(args, cfg)
        seed = int(args.seed if args.seed is not None else cfg["random_seed"])
        paths = {
            "source_full": validate_role_path(args.source_full, "source"), "target_full": validate_role_path(args.target_full, "target"),
            "source_roi": validate_role_path(args.source_roi, "source"), "target_roi": validate_role_path(args.target_roi, "target"),
        }
        if mode == "manual_files":
            assert manual_board_arg is not None and manual_object_arg is not None
            board_path = validate_role_path(manual_board_arg, "target"); object_path = validate_role_path(manual_object_arg, "target")
            validate_distinct_manual_paths(board_path, object_path)
            paths["target_manual_board"] = board_path; paths["target_manual_object"] = object_path

        logger.info("Stage 2 started; target_selection_mode=%s; registration_performed=false; transform_estimated=false", mode)
        loaded: dict[str, tuple[o3d.geometry.PointCloud, np.ndarray, dict[str, Any]]] = {}
        for label, path in paths.items():
            loaded[label] = _load_cloud(path, label, .001 if label.startswith("source") else 1.0)
            logger.info("Loaded %s: raw=%d effective=%d", label, loaded[label][2]["raw_point_count"], len(loaded[label][1]))

        source_full_cloud, source_full, source_full_clean = loaded["source_full"]
        target_full_cloud, target_full, target_full_clean = loaded["target_full"]
        source_roi_cloud, source_roi, source_roi_clean = loaded["source_roi"]
        target_roi_cloud, target_roi, target_roi_clean = loaded["target_roi"]
        source_plane = fit_plane_ransac(source_full, threshold_m=float(cfg["source_plane"]["distance_threshold_m"]),
                                        ransac_n=int(cfg["source_plane"]["ransac_n"]), iterations=int(cfg["source_plane"]["num_iterations"]), seed=seed)
        source_selection, source_threshold_report = _select_source(source_roi, source_plane, cfg, args)
        outputs: dict[str, Any] = {}
        outputs["source_board_points"] = _write_subset(source_full_cloud, source_plane.inlier_indices, output_dir / "source_board_points.ply")
        if source_selection["confirmed"]:
            outputs["source_object_points"] = _write_subset(source_roi_cloud, source_selection["selected_indices"], output_dir / "source_object_points.ply")
            outputs["source_object_points"]["selection_mode"] = source_selection["selection_mode"]

        input_roles = {
            "source_full": _input_record(paths["source_full"], "source", "only source board fit/output source", ["source_board_plane", "source_board_points"], source_full_cloud, source_full_clean, source_full),
            "source_roi": _input_record(paths["source_roi"], "source", "central common object only; source-only magnets excluded by ROI", ["source_object_points"], source_roi_cloud, source_roi_clean, source_roi),
            "target_roi": _input_record(paths["target_roi"], "target", "target coordinate/boundary reference in manual mode", ["manual_coordinate_validation", "roi_boundary_diagnostic"], target_roi_cloud, target_roi_clean, target_roi),
            "target_full": _input_record(paths["target_full"], "target", "explicit coordinate compatibility reference only", ["manual_coordinate_validation"], target_full_cloud, target_full_clean, target_full),
        }
        common = {
            "software": {"python": sys.version, "platform": platform.platform(), "open3d": o3d.__version__, "numpy": np.__version__, "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
            "config_path": str(config_path), "config": config, "random_seed": seed,
            "target_selection_mode": mode, "target_manual_files_preserved": mode == "manual_files",
            "input_roles": input_roles, "outputs": outputs, "raw_inputs_modified": False,
            "planes": {"source": {"fit_source": "source_full", "transferred_to_object_source": "source_roi", **source_plane.report}},
            "object_thresholds": {"source": source_threshold_report}, "object_selection": {"source": _serializable_selection(source_selection)},
        }

        if mode == "manual_files":
            board_cloud, board, board_clean = loaded["target_manual_board"]
            object_cloud, obj, object_clean = loaded["target_manual_object"]
            input_roles["target_manual_board"] = _input_record(paths["target_manual_board"], "target", "authoritative manual target board", ["target_board_plane", "target_board_points"], board_cloud, board_clean, board)
            input_roles["target_manual_object"] = _input_record(paths["target_manual_object"], "target", "authoritative manual target central object", ["target_object_points", "geometry_diagnostics_only"], object_cloud, object_clean, obj)
            identity = manual_identity_report(paths["target_manual_board"], paths["target_manual_object"], board, obj)
            if identity["sha256_identical"] or identity["cleaned_xyz_identical"]:
                report.update(common); report["manual_target_validation"] = {"identity": identity}
                raise Stage2Failure("manual_target_files_identical", "manual target board and object clouds are identical")

            mcfg = cfg["target_manual_validation"]
            board_compat = coordinate_compatibility_report(board, target_roi, target_full, full_margin_m=float(mcfg["target_full_aabb_margin_m"]),
                roi_margin_m=float(mcfg["target_roi_aabb_margin_m"]), minimum_roi_fraction=float(mcfg["minimum_target_roi_fraction"]))
            object_compat = coordinate_compatibility_report(obj, target_roi, target_full, full_margin_m=float(mcfg["target_full_aabb_margin_m"]),
                roi_margin_m=float(mcfg["target_roi_aabb_margin_m"]), minimum_roi_fraction=float(mcfg["minimum_target_roi_fraction"]))
            if not board_compat["compatible"] or not object_compat["compatible"]:
                report.update(common); report["manual_target_validation"] = {"identity": identity, "coordinate_compatibility": {"board": board_compat, "object": object_compat}}
                raise Stage2Failure("manual_target_coordinates_incompatible", "manual target coordinates are incompatible with target_roi/target_full FAST metre coordinates")

            try:
                target_plane = fit_plane_ransac(board, threshold_m=float(cfg["target_plane"]["distance_threshold_m"]),
                    ransac_n=int(cfg["target_plane"]["ransac_n"]), iterations=int(cfg["target_plane"]["num_iterations"]), seed=seed)
            except Exception as exc:
                raise Stage2Failure("manual_target_board_plane_fit_failed", f"manual target board cannot be fit robustly: {exc}") from exc
            geometry = manual_geometry_report(board, obj, target_roi, target_plane,
                cluster_eps_m=float(mcfg["object_cluster_eps_m"]), cluster_min_points=int(mcfg["object_cluster_min_points"]),
                minimum_object_points=int(mcfg["minimum_object_points"]), minimum_projection_extent_m=float(mcfg["minimum_projection_extent_m"]),
                maximum_boundary_fraction=float(mcfg["maximum_object_roi_boundary_fraction"]),
                minimum_largest_component_ratio=float(mcfg["minimum_largest_component_ratio"]))
            if geometry["board_plane_degenerate"] or target_plane.report["plane_inlier_ratio"] < float(mcfg["minimum_board_plane_inlier_ratio"]):
                report.update(common); report["planes"]["target"] = {"fit_source": "target_manual_board", **target_plane.report}
                report["manual_target_validation"] = {"identity": identity, "coordinate_compatibility": {"board": board_compat, "object": object_compat}, "geometry": geometry}
                raise Stage2Failure("manual_target_board_plane_degenerate", "manual target board plane is degenerate or has insufficient inlier support")

            overlap = exact_overlap_report(board, obj)
            overlap_obvious = overlap["overlap_count"] >= int(mcfg["overlap_minimum_count"]) and (
                overlap["board_overlap_ratio"] >= float(mcfg["overlap_ratio_threshold"]) or overlap["object_overlap_ratio"] >= float(mcfg["overlap_ratio_threshold"]))
            suspicious = list(geometry["suspicious_reasons"])
            suspicious.extend(str(item) for item in mcfg.get("known_manual_concerns", []))
            if overlap_obvious: suspicious.append("manual_target_clouds_obviously_overlap")

            outputs["target_board_points"] = _write_cloud(board_cloud, output_dir / "target_board_points.pcd")
            outputs["target_object_points"] = _write_cloud(object_cloud, output_dir / "target_object_points.pcd")
            outputs["target_board_points"]["selection_mode"] = "manual_files"; outputs["target_object_points"]["selection_mode"] = "manual_files"
            preview_path = output_dir / "segmentation_preview.png"
            save_manual_segmentation_preview(preview_path, source_board=source_full[source_plane.inlier_indices],
                source_object=source_roi[source_selection["selected_indices"]], source_plane=source_plane.model,
                target_board=board, target_object=obj, target_plane=target_plane.model, target_plane_inliers=target_plane.inlier_indices, seed=seed)
            report.update(common)
            report["planes"]["target"] = {"fit_source": "target_manual_board", **target_plane.report, "coverage": geometry["board_plane_projection"], "degenerate": geometry["board_plane_degenerate"]}
            report["planes"]["target_full_validation"] = {"performed": False, "used_for_plane_fit": False, "used_for_coordinate_compatibility_only": True}
            report["manual_target_validation"] = {
                "manual_path_source": manual_path_source, "identity": identity, "coordinate_compatibility": {"board": board_compat, "object": object_compat},
                "exact_overlap": overlap, "overlap_obvious": overlap_obvious, "geometry": geometry,
                "manual_preview_confirmed_via_cli": bool(args.confirm_target_manual_preview),
                "acknowledged_manual_limitations": [str(item) for item in mcfg.get("acknowledged_manual_limitations", [])],
                "automatic_target_side_selection_called": False, "distance_threshold_object_extraction_called": False,
                "dbscan_replaced_manual_selection": False, "manual_points_removed_due_to_overlap_or_geometry": 0,
            }
            report["preview"] = {"path": str(preview_path), "sha256": sha256_file(preview_path),
                "layout": "six fixed plane-local views: source/target combined, raw manual board/object, plane coverage, signed-distance side view",
                "unit": "m", "fixed_camera_direction": True, "colors": {"board": "gray", "object": "red", "plane_inlier": "blue", "plane_outlier": "orange"}}
            if not source_selection["confirmed"]: suspicious.append("source_object_side_not_confirmed")
            if suspicious:
                report["status"] = "needs_manual_confirmation"; report["warnings"] = suspicious
                report["required_manual_action"] = "Correct or explicitly review the reported overlap/geometric concerns; diagnostics never delete manual points."
            elif not args.confirm_target_manual_preview:
                report["status"] = "needs_manual_confirmation"
                report["required_manual_action"] = "Inspect segmentation_preview.png, then rerun the same command with --confirm-target-manual-preview."
            else:
                report["status"] = "success"; report["required_manual_action"] = None
            report["warnings"].extend(["Signed face-to-board distance is a geometric diagnostic, not physical thickness.",
                "No target scaling, translation, recentering, normalization, registration, or automatic selection replacement was performed."])
        elif automatic_target_selection_allowed(mode):
            target_plane = fit_plane_ransac(target_roi, threshold_m=float(cfg["target_plane"]["distance_threshold_m"]),
                ransac_n=int(cfg["target_plane"]["ransac_n"]), iterations=int(cfg["target_plane"]["num_iterations"]), seed=seed)
            source_extent = None
            if source_selection["confirmed"]:
                source_projection = plane_projection_report(source_roi[source_selection["selected_indices"]], source_plane.model)
                source_extent = np.sort(np.asarray(source_projection["uv_extent_m"], dtype=float))
            hcfg = cfg.get("target_heightmap")
            if not isinstance(hcfg, dict):
                raise Stage2Failure("target_heightmap_config_missing", "segmentation.target_heightmap configuration is required in automatic mode")
            target_selection = select_target_object_heightmap(
                target_roi, target_plane, hcfg, requested_side=args.target_side or hcfg.get("side"),
                source_extent_m=source_extent, high_threshold_override_m=args.target_object_threshold_m,
                plane_threshold_m=float(cfg["target_plane"]["distance_threshold_m"]),
                plane_ransac_n=int(cfg["target_plane"]["ransac_n"]),
                plane_iterations=int(cfg["target_plane"]["num_iterations"]), seed=seed)
            outputs["target_board_points"] = _write_subset(target_roi_cloud, target_selection["board_inlier_indices"], output_dir / "target_board_points.pcd")
            if target_selection["confirmed"]:
                outputs["target_object_points"] = _write_subset(target_roi_cloud, target_selection["selected_indices"], output_dir / "target_object_points.pcd")
                outputs["target_object_points"].update({"selection_mode": "target_heightmap",
                    "original_target_roi_index_subset": True, "attributes_preserved_by_index": True})
            report.update(common)
            report["planes"]["target"] = {
                "fit_source": "target_roi_then_outer_perimeter_refit",
                "initial": target_plane.report,
                "refit": target_selection["plane_refit"],
                "selected_plane_model_ax_by_cz_d": np.asarray(target_selection["selected_plane_model"]).tolist(),
            }
            report["object_thresholds"]["target"] = target_selection["thresholds"]
            report["object_selection"]["target"] = _serializable_selection(target_selection)
            report["target_automatic_algorithm"] = "target_heightmap_double_threshold_2d_connected_components"
            report["target_dbscan_used"] = False
            report["target_output_contract"] = target_selection["output_contract"]
            report["status"] = "success" if source_selection["confirmed"] and target_selection["confirmed"] else "needs_manual_confirmation"
            report["required_manual_action"] = None if report["status"] == "success" else (
                f"Review automatic target heightmap selection: {target_selection['selection_reason']}. "
                "An existing target_object_points.pcd was not overwritten by an unconfirmed result.")
            preview_path = output_dir / "segmentation_preview.png"
            save_target_heightmap_preview(preview_path, points=target_roi, selection=target_selection, seed=seed)
            report["preview"] = {"path": str(preview_path), "sha256": sha256_file(preview_path), "unit": "m",
                "layout": "six target diagnostics: raw u-v, depth, strong seeds, grown regions, final original points, rejected candidates",
                "fixed_plane_local_coordinates": True}
        else:
            raise Stage2Failure("invalid_target_selection_mode", f"unsupported target selection mode: {mode}")

        write_json(report_path, report)
        logger.info("STATUS=%s; target_selection_mode=%s; registration_performed=false; transform_estimated=false", report["status"], mode)
        logger.info("Report: %s", report_path)
        return 0
    except Exception as exc:
        reason = exc.reason if isinstance(exc, Stage2Failure) else f"{type(exc).__name__}"
        report["status"] = "failed"; report["failure_reason"] = reason
        report["failure_reasons"] = [reason, f"{type(exc).__name__}: {exc}"]
        report["registration_performed"] = False; report["transform_estimated"] = False
        write_json(report_path, report); logger.exception("Stage 2 failed [%s]: %s", reason, exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
