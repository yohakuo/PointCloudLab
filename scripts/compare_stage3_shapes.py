#!/usr/bin/env python3
"""Compare the sampled-point baseline with the observed-grid stage 3 run."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.shape_grid import shape_score
from pointcloud_registration.transforms import transform_difference


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, default=ROOT / "outputs/stage_03_sampled_baseline")
    parser.add_argument("--grid", type=Path, default=ROOT / "outputs/stage_03_occupancy_grid")
    args = parser.parse_args()
    old = json.loads((args.baseline / "candidate_report.json").read_text(encoding="utf-8"))
    new = json.loads((args.grid / "candidate_report.json").read_text(encoding="utf-8"))
    if old["status"] != "success" or new["status"] != "success":
        raise ValueError("both stage 3 runs must succeed")
    if old["input_integrity"]["before"] != new["input_integrity"]["before"]:
        raise ValueError("baseline and grid runs have different stage 2 inputs")
    if old["geometry_route"]["shape_method"] != "sampled_points" or new["geometry_route"]["shape_method"] != "occupancy_grid":
        raise ValueError("run methods do not match their comparison roles")
    if new["geometry_route"].get("search_method", "sequential") != "sequential":
        raise ValueError("stage-one comparison requires --search-method sequential; use compare_stage3_search.py for joint search")
    grids = json.loads((args.grid / "shape_grids.json").read_text(encoding="utf-8"))["grids"]
    def restore(name):
        return {k: np.asarray(v) for k, v in grids[name].items() if isinstance(v, list)}
    source, target = restore("source"), restore("observed_object_side")
    cfg = {**new["config"]["geometry"], **new["config"]["geometry"]["shape_grid"]}
    def grid_score(candidate):
        p = candidate["provenance"]
        shift = np.asarray(candidate["generation_parameters"]["shift_uv_m"])
        return shape_score(source, target, p["relative_angle_deg"], shift, cfg)[0]
    old_candidates, new_candidates = old["canonical_candidates"], new["canonical_candidates"]
    old_scores = [(grid_score(c), c) for c in old_candidates if c["provenance"]["normal_alignment_branch"] == "observed_object_side"]
    best_old_score, best_old = min(old_scores, key=lambda x: x[0])
    top_old, top_new = old_candidates[0], new_candidates[0]
    top_new_score = grid_score(top_new)
    nearest = []
    for c in new_candidates:
        match = min(old_candidates, key=lambda o: (lambda d: d[0] / 3 + d[1] / .006)(
            transform_difference(np.asarray(c["matrix_m"]), np.asarray(o["matrix_m"]))))
        dr, dt = transform_difference(np.asarray(c["matrix_m"]), np.asarray(match["matrix_m"]))
        nearest.append({"grid_rank": c["rank"], "baseline_rank": match["rank"],
                        "rotation_difference_deg": dr, "translation_difference_mm": dt * 1000})
    result = {"identical_input_hashes": True,
              "baseline": {"raw": old["candidate_counts"]["raw_total"], "canonical": len(old_candidates),
                           "contour_angle_deg": old["geometry_route"]["audit"][0]["continuous_bases"][1][1],
                           "top_rank": {"id": top_old["candidate_id"], "angle_deg": top_old["provenance"]["relative_angle_deg"],
                                        "point_diagnostic_chamfer_m": top_old["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"],
                                        "grid_score_m": grid_score(top_old)},
                           "best_canonical_under_grid_score": {"id": best_old["candidate_id"], "rank": best_old["rank"], "grid_score_m": best_old_score}},
              "grid": {"raw": new["candidate_counts"]["raw_total"], "canonical": len(new_candidates),
                       "contour_angle_deg": new["geometry_route"]["audit"][0]["continuous_bases"][1][1],
                       "top_rank": {"id": top_new["candidate_id"], "angle_deg": top_new["provenance"]["relative_angle_deg"],
                                    "point_diagnostic_chamfer_m": top_new["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"],
                                    "grid_score_m": top_new_score},
                       "shape_summaries": new["geometry_route"]["shape_summaries"]},
              "nearest_baseline_to_each_grid_candidate": nearest}
    (args.grid / "baseline_comparison.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 阶段 3：占据栅格与随机投影点基线对比", "",
             "两次运行使用相同的阶段 2 四份规范点云（SHA-256 一致），输出目录互不覆盖。",
             "", "| 指标 | 随机投影点基线 | 占据栅格与外轮廓 |", "| --- | ---: | ---: |",
             f"| 原始候选 / 规范候选 | {result['baseline']['raw']} / {result['baseline']['canonical']} | {result['grid']['raw']} / {result['grid']['canonical']} |",
             f"| 连续轮廓角修正 | {result['baseline']['contour_angle_deg']:.0f}° | {result['grid']['contour_angle_deg']:.0f}° |",
             f"| 排名 1 的平面内角 | {result['baseline']['top_rank']['angle_deg']:.0f}° | {result['grid']['top_rank']['angle_deg']:.0f}° |",
             f"| 排名 1 用同一栅格度量复评 | {result['baseline']['top_rank']['grid_score_m']*1000:.2f} mm | {top_new_score*1000:.2f} mm |",
             f"| 各自候选池在栅格度量下的最优值 | {best_old_score*1000:.2f} mm | {min(c['shape_match']['score_m'] for c in new_candidates)*1000:.2f} mm |",
             "", "栅格得分只用于本路线的搜索和粗排序，不能与旧路线的点级粗分数直接相减。点级 Chamfer 仍保留为宽松几何门限和诊断。",
             "", f"source 有 {source['centers'].shape[0]} 个观测格，target 有 {target['centers'].shape[0]} 个观测格；内部无点区域仍是未知。",
             f"target 的可靠外轮廓由 {new['geometry_route']['shape_summaries']['observed_object_side']['reliable_outline_support_cells']} 个高起伏连通支持格估计，",
             f"高度阈值为 {new['geometry_route']['shape_summaries']['observed_object_side']['outline_height_threshold_m']*1000:.1f} mm；外围有点格仍保留并降低可信度。",
             "当前 ROI 边界降权格数为 0；若另一批 ROI 截断对象，才会触发该机制。",
             "", "四个 90° 竞争方向仍在候选中；栅格分数接近，不能据此确认最终朝向。阶段 4 和人工复核仍需执行。", "",
             "## 新候选与最近旧候选", "", "| 新排名 | 最近旧排名 | 旋转差 | 平移差 |", "| ---: | ---: | ---: | ---: |"]
    lines.extend(f"| {x['grid_rank']} | {x['baseline_rank']} | {x['rotation_difference_deg']:.1f}° | {x['translation_difference_mm']:.2f} mm |" for x in nearest)
    (args.grid / "baseline_comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.grid / "baseline_comparison.md")


if __name__ == "__main__":
    main()
