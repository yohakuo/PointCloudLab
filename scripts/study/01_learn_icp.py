#!/usr/bin/env python3
"""Experiment 1: create a known rigid transform and recover it with ICP.

The input PLY is expressed in millimetres.  This script never modifies the
raw file: filtering, voxel down-sampling, and all synthetic clouds live in
memory until results are explicitly written beneath outputs/experiment_01.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "data" / "raw" / "20260909_203617_pc.ply"
DEFAULT_OUTPUT = ROOT / "outputs" / "experiment_01"


def euler_xyz_degrees_to_matrix(angles_deg: np.ndarray) -> np.ndarray:
    """Return Rz @ Ry @ Rx for degree inputs and column vectors.

    This is an extrinsic XYZ description: a point is rotated about fixed X,
    then fixed Y, then fixed Z axes.  The rightmost Rx acts first.
    """
    rx, ry, rz = np.deg2rad(np.asarray(angles_deg, dtype=np.float64))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rot_z @ rot_y @ rot_x


def make_transform(angles_deg: np.ndarray, translation_mm: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = euler_xyz_degrees_to_matrix(angles_deg)
    transform[:3, 3] = np.asarray(translation_mm, dtype=np.float64)
    return transform


def transform_errors(estimated: np.ndarray, ground_truth: np.ndarray) -> dict[str, float]:
    """Return source-to-target transform errors in mm and degrees."""
    translation_error = float(np.linalg.norm(estimated[:3, 3] - ground_truth[:3, 3]))
    rotation_error = estimated[:3, :3] @ ground_truth[:3, :3].T
    cosine = float((np.trace(rotation_error) - 1.0) / 2.0)
    # Rounding can otherwise make acos receive 1.000000000... and return NaN.
    angle_deg = float(np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0))))
    return {"translation_mm": translation_error, "rotation_deg": angle_deg}


def json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def finite_copy(cloud: o3d.geometry.PointCloud) -> tuple[o3d.geometry.PointCloud, int, int]:
    """Keep only finite XYZ coordinates without changing the original cloud."""
    points = np.asarray(cloud.points, dtype=np.float64)
    valid = np.isfinite(points).all(axis=1)
    clean = o3d.geometry.PointCloud()
    clean.points = o3d.utility.Vector3dVector(points[valid])
    if cloud.has_colors() and len(cloud.colors) == len(points):
        clean.colors = o3d.utility.Vector3dVector(np.asarray(cloud.colors)[valid])
    if cloud.has_normals() and len(cloud.normals) == len(points):
        clean.normals = o3d.utility.Vector3dVector(np.asarray(cloud.normals)[valid])
    return clean, int(len(points)), int(valid.sum())


def normal_status(cloud: o3d.geometry.PointCloud) -> dict[str, Any]:
    normals = np.asarray(cloud.normals)
    complete = cloud.has_normals() and len(normals) == len(cloud.points)
    finite = bool(complete and np.isfinite(normals).all())
    return {"present": bool(cloud.has_normals()), "complete": bool(complete), "finite": finite}


def registration_metrics(result: Any, ground_truth: np.ndarray, elapsed_s: float) -> dict[str, Any]:
    return {
        "transform": result.transformation,
        "correspondence_count": int(len(result.correspondence_set)),
        "fitness": float(result.fitness),
        "inlier_rmse_mm": float(result.inlier_rmse),
        "transform_error": transform_errors(result.transformation, ground_truth),
        "runtime_seconds": float(elapsed_s),
        "reached_max_iterations": None,
        "reached_max_iterations_note": "Open3D 0.19 RegistrationResult does not reliably expose iteration count.",
    }


def run_icp(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
            initial: np.ndarray, threshold_mm: float, criteria: Any,
            estimation: Any, ground_truth: np.ndarray) -> tuple[Any, dict[str, Any]]:
    start = time.perf_counter()
    # ICP builds nearest-neighbour correspondences after applying the current pose.
    # Pairs farther than threshold_mm are discarded. Each iteration then estimates
    # an incremental rigid motion: point-to-point minimizes ||Rp+t-q||, whereas
    # point-to-plane minimizes the target-normal projection n_q dot (Rp+t-q).
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold_mm, initial, estimation, criteria)
    return result, registration_metrics(result, ground_truth, time.perf_counter() - start)


def colored_view_cloud(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
                       transform: np.ndarray, title: str) -> None:
    # Deep copies prevent transform() calls in one view from accumulating into later views.
    source_view = copy.deepcopy(source)
    target_view = copy.deepcopy(target)
    source_view.transform(transform)
    source_view.paint_uniform_color([1.0, 0.45, 0.05])
    target_view.paint_uniform_color([0.15, 0.45, 1.0])
    both = source_view + target_view
    diagonal = max(float(np.linalg.norm(both.get_axis_aligned_bounding_box().get_extent())), 1e-6)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(diagonal * 0.12, 1e-6), origin=both.get_center())
    print(f"[VIEW] {title}：橙色 source，蓝色 target；关闭窗口后继续。")
    o3d.visualization.draw_geometries([source_view, target_view, axis], window_name=title,
                                      width=1200, height=850)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experiment 1: synthetic rigid transform and ICP")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--voxel-size-mm", type=float, default=1.0)
    parser.add_argument("--gt-rotation-deg", type=float, nargs=3, default=[5.0, -8.0, 15.0], metavar=("RX", "RY", "RZ"))
    parser.add_argument("--gt-translation-mm", type=float, nargs=3, default=[100.0, -50.0, 80.0], metavar=("TX", "TY", "TZ"))
    parser.add_argument("--initial-residual-rotation-deg", type=float, nargs=3, default=[0.3, -0.4, 0.5], metavar=("RX", "RY", "RZ"))
    parser.add_argument("--initial-residual-translation-mm", type=float, nargs=3, default=[2.0, -2.0, 2.0], metavar=("TX", "TY", "TZ"))
    parser.add_argument("--normal-radius-mm", type=float, default=3.0)
    parser.add_argument("--normal-max-neighbors", type=int, default=30)
    parser.add_argument("--correspondence-threshold-mm", type=float, default=10.0)
    parser.add_argument("--maximum-iterations", type=int, default=80)
    parser.add_argument("--relative-fitness", type=float, default=1e-6)
    parser.add_argument("--relative-rmse", type=float, default=1e-6)
    parser.add_argument("--skip-bad-init", action="store_true")
    parser.add_argument("--no-view", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    positive = [args.voxel_size_mm, args.normal_radius_mm, args.correspondence_threshold_mm]
    if any(value <= 0 for value in positive) or args.normal_max_neighbors < 3 or args.maximum_iterations < 1:
        raise SystemExit("voxel、normal radius、threshold 必须为正；normal 邻居至少 3；迭代次数至少 1。")
    if not args.input.is_file():
        raise SystemExit(f"输入文件不存在：{args.input}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw = o3d.io.read_point_cloud(str(args.input), remove_nan_points=False, remove_infinite_points=False)
    raw_normal_info = normal_status(raw)
    clean, original_count, valid_count = finite_copy(raw)
    if valid_count < 3:
        raise SystemExit("有限点少于 3，无法进行 ICP。")
    source = clean.voxel_down_sample(args.voxel_size_mm)
    if len(source.points) < 3:
        raise SystemExit("降采样后点数少于 3，请减小 voxel size。")

    ground_truth = make_transform(np.array(args.gt_rotation_deg), np.array(args.gt_translation_mm))
    target = copy.deepcopy(source)
    target.transform(ground_truth)
    delta = make_transform(np.array(args.initial_residual_rotation_deg),
                           np.array(args.initial_residual_translation_mm))
    # T_delta is defined in target/world coordinates.  Thus it left-multiplies
    # T_ground_truth; swapping the order would define a different residual frame.
    initial_good = delta @ ground_truth
    initial_bad = np.eye(4, dtype=np.float64)
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=args.relative_fitness, relative_rmse=args.relative_rmse,
        max_iteration=args.maximum_iterations)

    # The input PLY has normals, but voxel sampling changes point locations and
    # may aggregate/drop attributes. Re-estimation gives target normals consistent
    # with the sampled surface and the chosen 3 mm support radius.
    target_normal_before = normal_status(target)
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.normal_radius_mm, max_nn=args.normal_max_neighbors))
    target.normalize_normals()
    target_normal_after = normal_status(target)

    initial_good_error = transform_errors(initial_good, ground_truth)
    initial_bad_error = transform_errors(initial_bad, ground_truth)
    point_to_point, p2p = run_icp(
        source, target, initial_good, args.correspondence_threshold_mm, criteria,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(), ground_truth)
    point_to_plane, p2l = run_icp(
        source, target, initial_good, args.correspondence_threshold_mm, criteria,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(), ground_truth)
    bad_result = bad_metrics = None
    if not args.skip_bad_init:
        bad_result, bad_metrics = run_icp(
            source, target, initial_bad, args.correspondence_threshold_mm, criteria,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(), ground_truth)

    np.savetxt(args.output_dir / "T_ground_truth.txt", ground_truth, fmt="%.12f")
    np.savetxt(args.output_dir / "T_initial_good.txt", initial_good, fmt="%.12f")
    np.savetxt(args.output_dir / "T_initial_bad.txt", initial_bad, fmt="%.12f")
    np.savetxt(args.output_dir / "T_point_to_point.txt", point_to_point.transformation, fmt="%.12f")
    np.savetxt(args.output_dir / "T_point_to_plane.txt", point_to_plane.transformation, fmt="%.12f")
    if bad_result is not None:
        np.savetxt(args.output_dir / "T_bad_init_result.txt", bad_result.transformation, fmt="%.12f")
    else:
        np.savetxt(args.output_dir / "T_bad_init_result.txt", np.full((4, 4), np.nan), fmt="%.12f")
    o3d.io.write_point_cloud(str(args.output_dir / "source_downsampled.ply"), source)
    o3d.io.write_point_cloud(str(args.output_dir / "target_synthetic.ply"), target)
    aligned_p2p = copy.deepcopy(source); aligned_p2p.transform(point_to_point.transformation)
    aligned_p2l = copy.deepcopy(source); aligned_p2l.transform(point_to_plane.transformation)
    o3d.io.write_point_cloud(str(args.output_dir / "source_aligned_point_to_point.ply"), aligned_p2p)
    o3d.io.write_point_cloud(str(args.output_dir / "source_aligned_point_to_plane.ply"), aligned_p2l)

    bad_observation = "未运行（--skip-bad-init）。"
    if bad_metrics is not None:
        error = bad_metrics["transform_error"]
        if bad_metrics["inlier_rmse_mm"] < args.voxel_size_mm and (error["translation_mm"] > 5 or error["rotation_deg"] > 1):
            bad_observation = "低 RMSE 但 ground-truth error 较大：可能是局部错误最优或局部重复结构对齐。"
        elif bad_metrics["fitness"] == 0:
            bad_observation = "identity 初值在当前 10 mm 阈值下没有有效对应，超出 ICP 捕获范围。"
        else:
            bad_observation = "较差初值的实际结果已记录；若仍收敛，说明该场景与阈值下捕获范围较大。"
    metrics = {
        "experiment_id": "01_learn_icp", "data_file": str(args.input.resolve()), "data_unit": "millimetres",
        "original_point_count": original_count, "valid_point_count": valid_count,
        "downsampled_point_count": int(len(source.points)), "voxel_size_mm": args.voxel_size_mm,
        "normal_radius_mm": args.normal_radius_mm, "normal_max_neighbors": args.normal_max_neighbors,
        "correspondence_threshold_mm": args.correspondence_threshold_mm,
        "icp_convergence": {"maximum_iterations": args.maximum_iterations, "relative_fitness": args.relative_fitness, "relative_rmse": args.relative_rmse},
        "euler_convention": "column vectors; R = Rz @ Ry @ Rx; fixed-axis X then Y then Z; degree inputs converted by numpy.deg2rad",
        "initial_pose_convention": "T_initial_good = T_delta @ T_ground_truth; T_delta is a residual transform in target/world coordinates.",
        "ground_truth_T": ground_truth, "initial_good_T": initial_good, "initial_bad_T": initial_bad,
        "initial_good_error": initial_good_error, "initial_bad_error": initial_bad_error,
        "point_to_point": p2p, "point_to_plane": p2l, "bad_initialization_point_to_point": bad_metrics,
        "raw_normal_status": raw_normal_info, "target_normal_before_reestimation": target_normal_before, "target_normal_after_reestimation": target_normal_after,
        "fpfh_parameters": None, "ransac_parameters": None,
        "metric_interpretation": {
            "fitness": "在当前 correspondence threshold 内形成有效对应的 source 点比例。",
            "inlier_rmse": "仅统计阈值内对应残差，单位为 mm。改变 threshold、voxel 或采样方式后不可脱离条件直接比较。",
            "ground_truth": "本实验有已知真值，因此变换误差反映真实配准误差；真实跨设备配准时 inlier RMSE 不等于绝对精度。",
        },
        "observations": {"bad_initialization": bad_observation}, "failure_analysis": "待依据实际运行结果分析。",
    }
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(json_safe(metrics), handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")

    print("Experiment 1 complete")
    print(f"  原始/有效/降采样点数: {original_count}/{valid_count}/{len(source.points)}")
    print(f"  良好初值误差: {initial_good_error}")
    for name, item in (("Point-to-Point", p2p), ("Point-to-Plane", p2l)):
        print(f"  {name}: correspondences={item['correspondence_count']}, fitness={item['fitness']:.6f}, RMSE={item['inlier_rmse_mm']:.6f} mm, error={item['transform_error']}, time={item['runtime_seconds']:.3f}s")
    if bad_metrics is not None:
        print(f"  Bad-init Point-to-Point: correspondences={bad_metrics['correspondence_count']}, fitness={bad_metrics['fitness']:.6f}, RMSE={bad_metrics['inlier_rmse_mm']:.6f} mm, error={bad_metrics['transform_error']}, time={bad_metrics['runtime_seconds']:.3f}s")
    print(f"  输出目录: {args.output_dir.resolve()}")
    if not args.no_view:
        colored_view_cloud(source, target, np.eye(4), "1. source 与 target（未应用初始位姿）")
        colored_view_cloud(source, target, initial_good, "2. 良好初值后")
        colored_view_cloud(source, target, point_to_point.transformation, "3. Point-to-Point ICP 后")
        colored_view_cloud(source, target, point_to_plane.transformation, "4. Point-to-Plane ICP 后")
        if bad_result is not None:
            colored_view_cloud(source, target, bad_result.transformation, "5. 较差初值 ICP 后")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
