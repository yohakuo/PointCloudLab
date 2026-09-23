#!/usr/bin/env python3
"""Experiment 0: inspect raw point clouds without registration or preprocessing."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import platform
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - Open3D fallback is for minimal installs.
    cKDTree = None


# 集中管理实验参数；它们也可通过命令行覆盖，避免在逻辑中散落 magic numbers。
DEFAULT_SAMPLE_SIZE = 10_000
DEFAULT_SEED = 42
DEFAULT_NEAR_ZERO_THRESHOLD = 1e-9

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUTS = (
    ROOT / "data" / "raw" / "09091909.pcd",
    ROOT / "data" / "raw" / "09092051.pcd",
    ROOT / "data" / "raw" / "20260909_203617_pc.ply",
)
DEFAULT_OUTPUT = ROOT / "outputs" / "experiment_00" / "inspection.json"


def json_safe(value: Any) -> Any:
    """将 NumPy 标量、数组和非有限浮点数转换成有效 JSON 值。"""
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def read_header(path: Path) -> dict[str, Any]:
    """只读文件头，补充 Open3D 几何属性之外的格式和顶点属性信息。"""
    suffix = path.suffix.lower()
    if suffix not in {".pcd", ".ply"}:
        raise ValueError(f"不支持的扩展名 {suffix!r}，仅支持 .pcd 或 .ply")

    header_lines: list[str] = []
    with path.open("rb") as handle:
        for _ in range(512):
            line = handle.readline()
            if not line:
                break
            decoded = line.decode("ascii", errors="replace").rstrip("\r\n")
            header_lines.append(decoded)
            if (suffix == ".pcd" and decoded.upper().startswith("DATA ")) or (
                suffix == ".ply" and decoded.strip().lower() == "end_header"
            ):
                break

    text = "\n".join(header_lines)
    if suffix == ".pcd":
        fields: list[str] = []
        sizes: list[str] = []
        types: list[str] = []
        counts: list[str] = []
        point_count: int | None = None
        encoding = "unknown"
        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "SIZE":
                sizes = parts[1:]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = parts[1:]
            elif key == "POINTS" and len(parts) > 1:
                point_count = int(parts[1])
            elif key == "DATA" and len(parts) > 1:
                encoding = parts[1].lower()
        properties = [
            {"name": name, "size_bytes": sizes[i] if i < len(sizes) else None,
             "type": types[i] if i < len(types) else None,
             "count": counts[i] if i < len(counts) else 1}
            for i, name in enumerate(fields)
        ]
        has_color = any(name.lower() in {"rgb", "rgba"} for name in fields)
        return {"format": encoding, "header_point_count": point_count,
                "vertex_properties": properties, "header": text,
                "has_color_property": has_color}

    encoding = "unknown"
    properties: list[dict[str, Any]] = []
    vertex_count: int | None = None
    in_vertex = False
    for line in header_lines:
        parts = line.split()
        lower = line.strip().lower()
        if lower.startswith("format ") and len(parts) > 1:
            encoding = parts[1].lower()
        elif lower.startswith("element "):
            in_vertex = len(parts) > 2 and parts[1].lower() == "vertex"
            if in_vertex:
                vertex_count = int(parts[2])
        elif lower.startswith("property ") and in_vertex and len(parts) >= 3:
            properties.append({"type": parts[1], "name": parts[-1]})
        elif lower == "end_header":
            break
    names = {str(item["name"]).lower() for item in properties}
    return {"format": encoding, "header_point_count": vertex_count,
            "vertex_properties": properties, "header": text,
            "has_color_property": bool({"red", "green", "blue"} <= names)}


def finite_point_stats(points: np.ndarray) -> tuple[dict[str, Any], np.ndarray]:
    """先统计 NaN/Inf，再仅以内存中的有限点计算几何统计。"""
    if points.size == 0:
        finite_mask = np.zeros(0, dtype=bool)
    else:
        finite_mask = np.isfinite(points).all(axis=1)
    nan_mask = np.isnan(points).any(axis=1) if len(points) else np.zeros(0, bool)
    inf_mask = np.isinf(points).any(axis=1) if len(points) else np.zeros(0, bool)
    result: dict[str, Any] = {
        "input_point_count": int(len(points)),
        "valid_point_count": int(finite_mask.sum()),
        "nan_point_count": int(nan_mask.sum()),
        "inf_point_count": int(inf_mask.sum()),
        "non_finite_point_count": int((~finite_mask).sum()),
        "has_non_finite_coordinates": bool((~finite_mask).any()),
    }
    valid = points[finite_mask]
    if len(valid):
        minimum = valid.min(axis=0)
        maximum = valid.max(axis=0)
        dimensions = maximum - minimum
        center = (minimum + maximum) / 2.0
        result.update({
            "minimum": minimum, "maximum": maximum, "range": dimensions,
            "bounding_box_dimensions": dimensions,
            "bounding_box_diagonal": np.linalg.norm(dimensions),
            "center": center,
        })
    else:
        result.update({"minimum": None, "maximum": None, "range": None,
                      "bounding_box_dimensions": None,
                      "bounding_box_diagonal": None, "center": None})
    return result, valid


def nearest_neighbor_estimate(points: np.ndarray, sample_size: int, seed: int,
                              threshold: float) -> dict[str, Any]:
    """在完整有效点集上建树，仅抽样查询点，结果明确标记为估计值。"""
    result: dict[str, Any] = {
        "is_sampling_estimate": True, "sample_size_requested": int(sample_size),
        "random_seed": int(seed), "near_zero_threshold": float(threshold),
        "sample_size_used": 0, "zero_distance_count": 0,
        "near_zero_distance_count": 0, "status": "not_available",
    }
    if len(points) < 2:
        result["status"] = "fewer_than_two_valid_points"
        return result
    count = min(max(int(sample_size), 1), len(points))
    rng = np.random.default_rng(seed)
    indices = rng.choice(len(points), size=count, replace=False)
    query = points[indices]
    if cKDTree is not None:
        distances, _ = cKDTree(points).query(query, k=2, workers=1)
        distances = distances[:, 1]
    else:
        tree = o3d.geometry.KDTreeFlann(o3d.utility.Vector3dVector(points))
        distances = np.array([tree.search_knn_vector_3d(point, 2)[2][-1] ** 0.5
                              for point in query], dtype=np.float64)
    result.update({
        "sample_size_used": int(count), "status": "ok",
        "zero_distance_count": int(np.count_nonzero(distances == 0.0)),
        "near_zero_distance_count": int(np.count_nonzero(distances <= threshold)),
        "mean": distances.mean(), "median": np.median(distances),
        "std": distances.std(), "p10": np.percentile(distances, 10),
        "p90": np.percentile(distances, 90),
    })
    return result


def inspect_one(path: Path, args: argparse.Namespace) -> dict[str, Any]:
    start = time.perf_counter()
    item: dict[str, Any] = {"input_file": str(path), "extension": path.suffix.lower()}
    try:
        if not path.is_file():
            raise FileNotFoundError(f"文件不存在：{path}")
        item["file_size_bytes"] = path.stat().st_size
        item["file_properties"] = read_header(path)
        cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=False,
                                        remove_infinite_points=False)
        points = np.asarray(cloud.points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise RuntimeError(f"Open3D 返回了异常点坐标形状：{points.shape}")
        header_count = item["file_properties"].get("header_point_count")
        if len(points) == 0 and header_count not in (None, 0):
            raise RuntimeError(
                f"Open3D 未读取到点，但文件头声明了 {header_count} 个点；文件可能损坏或格式不兼容"
            )
        stats, valid = finite_point_stats(points)
        item["status"] = "success"
        item["point_count"] = int(len(points))
        item["has_color"] = bool(cloud.has_colors() or item["file_properties"].get("has_color_property"))
        item["has_normals"] = bool(cloud.has_normals())
        item["other_vertex_attributes"] = item["file_properties"].get("vertex_properties", [])
        item["coordinate_statistics"] = stats
        item["nearest_neighbor_spacing"] = nearest_neighbor_estimate(
            valid, args.sample_size, args.seed, args.near_zero_threshold)
        item["read_and_statistics_seconds"] = time.perf_counter() - start
        item["_cloud"] = cloud
        item["_valid_points"] = valid
        print(f"\n[{path.name}]")
        print(f"  文件：{path}")
        print(f"  格式：{item['file_properties'].get('format')}；大小：{item['file_size_bytes']} bytes")
        print(f"  点数：{len(points)}；颜色：{item['has_color']}；法向量：{item['has_normals']}")
        print(f"  顶点属性：{item['other_vertex_attributes']}")
        print(f"  有效/NaN/Inf：{stats['valid_point_count']}/{stats['nan_point_count']}/{stats['inf_point_count']}")
        print(f"  最小值：{stats['minimum']}；最大值：{stats['maximum']}")
        print(f"  范围：{stats['range']}；包围盒对角线：{stats['bounding_box_diagonal']}")
        print(f"  中心：{stats['center']}")
        nn = item["nearest_neighbor_spacing"]
        print(f"  最近邻（完整有效集上的抽样估计，n={nn['sample_size_used']}）："
              f"mean={nn.get('mean')} median={nn.get('median')} std={nn.get('std')} "
              f"P10={nn.get('p10')} P90={nn.get('p90')} near-zero={nn['near_zero_distance_count']}")
        return item
    except Exception as exc:  # 报告清晰错误，同时继续检查其他输入。
        item["status"] = "failure"
        item["error"] = f"{type(exc).__name__}: {exc}"
        item["read_and_statistics_seconds"] = time.perf_counter() - start
        print(f"[ERROR] {path}: {item['error']}", file=sys.stderr)
        return item


def scale_diagnostics(items: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [item for item in items if item.get("status") != "failure"]
    names = [Path(item["input_file"]).name for item in valid]
    dimensions = {name: item["coordinate_statistics"].get("bounding_box_dimensions")
                  for name, item in zip(names, valid)}
    diagonals = {name: item["coordinate_statistics"].get("bounding_box_diagonal")
                 for name, item in zip(names, valid)}
    spacing = {name: item["nearest_neighbor_spacing"].get("median")
               for name, item in zip(names, valid)}
    ratios: dict[str, Any] = {}
    for i, first in enumerate(names):
        for second in names[i + 1:]:
            def ratio(a: Any, b: Any) -> float | None:
                if a is None or b is None or abs(float(b)) <= np.finfo(float).eps:
                    return None
                return float(a) / float(b)
            a_dim, b_dim = dimensions[first], dimensions[second]
            ratios[f"{first} / {second}"] = {
                "bounding_box_axis_ratios": [ratio(a, b) for a, b in zip(a_dim, b_dim)]
                if a_dim is not None and b_dim is not None else None,
                "diagonal_ratio": ratio(diagonals[first], diagonals[second]),
                "median_spacing_ratio": ratio(spacing[first], spacing[second]),
            }
    return {"bounding_box_dimensions": dimensions, "diagonal_lengths": diagonals,
            "median_nearest_neighbor_spacing": spacing, "pairwise_ratios": ratios,
            "near_zero_axis_policy": "接近零的轴向尺寸比例记为 null，不用于数量级判断"}


def unit_diagnosis(scale: dict[str, Any]) -> str:
    ratios = [entry.get("diagonal_ratio") for entry in scale["pairwise_ratios"].values()]
    ratios = [abs(x) for x in ratios if x is not None and math.isfinite(x) and x > 0]
    if not ratios:
        return "由于缺少可比较的有限尺度，单位诊断待实验。"
    if any(300 < value < 3000 or 1 / 3000 < value < 1 / 300 for value in ratios):
        return "疑似存在约 1000 倍的 m/mm 单位差异；仅凭全局 bounding box 仍需已知物体尺寸进一步确认。"
    return "未发现明显的约 1000 倍数量级差异；由于覆盖范围可能不同，仅凭全局 bounding box 无法判断单位。"


def show_cloud(item: dict[str, Any]) -> None:
    cloud = item["_cloud"]
    all_points = np.asarray(cloud.points, dtype=np.float64)
    finite_mask = np.isfinite(all_points).all(axis=1)
    valid = all_points[finite_mask]
    if len(valid) == 0:
        print(f"[VIEW] 跳过 {item['input_file']}：没有有限坐标点。")
        return
    # 统计对象保持不变；可视化使用独立副本，并只显示有限点。
    view_cloud = o3d.geometry.PointCloud()
    view_cloud.points = o3d.utility.Vector3dVector(valid)
    if cloud.has_colors() and len(cloud.colors) == len(all_points):
        colors = np.asarray(cloud.colors)[finite_mask]
        view_cloud.colors = o3d.utility.Vector3dVector(colors)
    if cloud.has_normals() and len(cloud.normals) == len(all_points):
        normals = np.asarray(cloud.normals)[finite_mask]
        view_cloud.normals = o3d.utility.Vector3dVector(normals)
    if not view_cloud.has_colors():
        view_cloud.paint_uniform_color([0.2, 0.7, 1.0])
    bbox = view_cloud.get_axis_aligned_bounding_box()
    diagonal = max(float(np.linalg.norm(bbox.get_extent())), 1e-6)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(diagonal * 0.15, 1e-6), origin=bbox.get_center())
    print(f"[VIEW] 正在显示 {item['input_file']}；关闭窗口后继续下一份。")
    o3d.visualization.draw_geometries([view_cloud, bbox, axis],
                                      window_name=Path(item["input_file"]).name,
                                      width=1100, height=800)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experiment 0: inspect raw PCD/PLY point clouds")
    parser.add_argument("--inputs", nargs="+", type=Path, default=list(DEFAULT_INPUTS),
                        help="一个或多个 .pcd/.ply 文件；默认检查三个原始文件")
    parser.add_argument("--no-view", action="store_true", help="跳过 Open3D 窗口")
    parser.add_argument("--sample-size", type=int, default=DEFAULT_SAMPLE_SIZE,
                        help=f"最近邻查询抽样数（默认 {DEFAULT_SAMPLE_SIZE}）")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help=f"固定随机种子（默认 {DEFAULT_SEED}）")
    parser.add_argument("--near-zero-threshold", type=float,
                        default=DEFAULT_NEAR_ZERO_THRESHOLD,
                        help=f"近零距离阈值（默认 {DEFAULT_NEAR_ZERO_THRESHOLD:g}）")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="JSON 输出路径")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.sample_size <= 0 or args.near_zero_threshold < 0:
        raise SystemExit("--sample-size 必须为正数，--near-zero-threshold 不能为负数")
    total_start = time.perf_counter()
    print("Experiment 0：真实点云检查（不执行配准或预处理）")
    loaded_items = [inspect_one(path.resolve(), args) for path in args.inputs]
    items = []
    for loaded_item in loaded_items:
        item = loaded_item.copy()
        for key in ("_cloud", "_valid_points"):
            item.pop(key, None)
        items.append(item)
    scale = scale_diagnostics(items)
    print("\n[文件间尺度比较]")
    print(f"  包围盒三轴尺寸：{scale['bounding_box_dimensions']}")
    print(f"  对角线长度：{scale['diagonal_lengths']}")
    print(f"  最近邻中位数：{scale['median_nearest_neighbor_spacing']}")
    print(f"  单位诊断：{unit_diagnosis(scale)}")
    report = {
        "experiment_id": "00_inspect_pointcloud",
        "execution_time": dt.datetime.now(dt.timezone.utc).isoformat(),
        "python_version": sys.version,
        "platform": platform.platform(),
        "open3d_version": o3d.__version__,
        "numpy_version": np.__version__,
        "scipy_version": __import__("scipy").__version__ if cKDTree is not None else None,
        "parameters": {"sample_size": args.sample_size, "seed": args.seed,
                       "near_zero_threshold": args.near_zero_threshold},
        "inputs": items,
        "file_scale_comparison": scale,
        "unit_diagnosis": unit_diagnosis(scale),
        "observations": "待实验",
        "failure_analysis": "待实验",
        "total_runtime_seconds": time.perf_counter() - total_start,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as handle:
        json.dump(json_safe(report), handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(f"报告已保存：{args.output.resolve()}")
    if not args.no_view:
        for item in loaded_items:
            if item.get("status") != "failure":
                show_cloud(item)
    return 1 if any(item.get("status") == "failure" for item in items) else 0


if __name__ == "__main__":
    raise SystemExit(main())
