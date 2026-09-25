#!/usr/bin/env python3
"""Compare sequential and joint searches using identical stage-one grids."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.shape_grid import shape_score


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def compare(paths, out):
    reports = [read(p / "candidate_report.json") for p in paths]
    exports = [read(p / "shape_grids.json")["grids"] for p in paths]
    reference = reports[0]
    if reference["geometry_route"]["search_method"] != "sequential":
        raise ValueError("baseline must use sequential search")
    for i, report in enumerate(reports):
        if report["status"] != "success" or report["geometry_route"]["shape_method"] != "occupancy_grid":
            raise ValueError("all runs must be successful occupancy_grid runs")
        if i and report["geometry_route"]["search_method"] != "joint":
            raise ValueError("comparison runs must use joint search")
        if report["input_integrity"]["before"] != reference["input_integrity"]["before"]:
            raise ValueError("stage 2 input hashes differ")
        if exports[i] != exports[0]:
            raise ValueError("observed grids/coordinate frames differ; cannot compare stored planar hypotheses")
        if report["config"]["geometry"]["shape_grid"] != reference["config"]["geometry"]["shape_grid"]:
            raise ValueError("shape score configurations differ")
        for side in ("source", "target"):
            for key in ("origin", "u", "v", "n"):
                if report["planes"][side]["frame"][key] != reference["planes"][side]["frame"][key]:
                    raise ValueError("PCA frames differ")
        if report["software"].get("OMP_NUM_THREADS") != reference["software"].get("OMP_NUM_THREADS"):
            raise ValueError("thread settings differ")
    grids = {name: {k: np.asarray(v) for k, v in grid.items() if isinstance(v, list)}
             for name, grid in exports[0].items()}
    cfg = reference["config"]["geometry"]["shape_grid"]
    summaries = []
    fig, axes = plt.subplots(len(paths), 2, figsize=(12, 4 * len(paths)), squeeze=False)
    for row, (path, report) in enumerate(zip(paths, reports)):
        raw = read(path / "raw_candidates.json")["candidates"]
        candidates = report["canonical_candidates"]
        def rescore(c):
            return shape_score(grids["source"], grids[c["provenance"]["normal_alignment_branch"]],
                               c["provenance"]["relative_angle_deg"],
                               np.asarray(c["generation_parameters"]["shift_uv_m"]), cfg)[0]
        scores = {c["candidate_id"]: rescore(c) for c in candidates}
        branches = []
        for normal in sorted({c["provenance"]["normal_alignment_branch"] for c in raw}):
            for branch in (0, 90, 180, 270):
                selected = [c for c in candidates if c["provenance"]["normal_alignment_branch"] == normal
                            and c["provenance"]["discrete_increment_deg"] == branch]
                pool = [c for c in raw if c["provenance"]["normal_alignment_branch"] == normal
                        and c["provenance"]["discrete_increment_deg"] == branch]
                best = min(selected, key=lambda c: scores[c["candidate_id"]]) if selected else None
                branches.append({"normal_branch": normal, "direction_branch_deg": branch,
                                 "raw_count": len(pool), "canonical_count": len(selected),
                                 "angles_deg": [c["provenance"]["relative_angle_deg"] % 360 for c in selected],
                                 "shifts_uv_m": [c["generation_parameters"]["shift_uv_m"] for c in selected],
                                 "best_shape_score_m": scores[best["candidate_id"]] if best else None})
        best = min(candidates, key=lambda c: scores[c["candidate_id"]])
        first = candidates[0]
        audits = report["geometry_route"]["audit"]
        summary = {"directory": str(path.resolve()), "name": path.name,
                   "search_method": report["geometry_route"]["search_method"],
                   "raw_count": len(raw), "canonical_count": len(candidates),
                   "covered_normal_direction_branches": sum(b["canonical_count"] > 0 for b in branches),
                   "branches": branches, "best_shape_score_m": scores[best["candidate_id"]],
                   "top_candidate_id": first["candidate_id"], "top_angle_deg": first["provenance"]["relative_angle_deg"] % 360,
                   "top_shape_score_m": scores[first["candidate_id"]], "top_shape_terms": first["shape_match"],
                   "top_point_chamfer_m": first["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"],
                   "best_point_chamfer_m": min(c["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"] for c in candidates),
                   "search_elapsed_s": report["geometry_route"]["search_elapsed_s"], "elapsed_s": report["elapsed_s"],
                   "coarse_angle_count_per_normal": [a.get("coarse_angle_count") for a in audits],
                   "total_evaluations": sum(a.get("total_evaluations", 0) for a in audits) or None,
                   "translation_boundary_results": sum(c["generation_parameters"].get("translation_boundary", False) for c in candidates)}
        summaries.append(summary)
        for audit in audits:
            profile = audit.get("angle_profile", [])
            if profile:
                axes[row, 0].plot([p["angle_deg"] for p in profile], [p["score_m"] * 1000 for p in profile],
                                  linewidth=1, alpha=.7, label=audit["normal_branch"] + " coarse")
        for branch, color in zip((0, 90, 180, 270), ("C0", "C1", "C2", "C3")):
            selected = [c for c in candidates if c["provenance"]["discrete_increment_deg"] == branch]
            axes[row, 0].scatter([c["provenance"]["relative_angle_deg"] % 360 for c in selected],
                                  [scores[c["candidate_id"]] * 1000 for c in selected], color=color, label=f"branch {branch}")
            axes[row, 1].scatter([c["generation_parameters"]["shift_uv_m"][0] * 1000 for c in selected],
                                  [c["generation_parameters"]["shift_uv_m"][1] * 1000 for c in selected], color=color, label=f"branch {branch}")
        axes[row, 0].set(xlim=(0, 360), xlabel="Angle in PCA frame (deg)", ylabel="Common shape score (mm)", title=path.name)
        axes[row, 1].set(xlabel="Translation u (mm)", ylabel="Translation v (mm)", title="Canonical translation coverage")
        for ax in axes[row]:
            ax.grid(alpha=.2)
            ax.legend(fontsize=7)
    out.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out / "search_comparison.png", dpi=160)
    plt.close(fig)
    result = {"identical_input_hashes": True, "identical_shape_grids_and_frames": True,
              "identical_scoring_config": True, "runs": summaries,
              "timing_scope": "single sequentially executed run per method; search excludes fitting/rendering, elapsed includes them; process import excluded",
              "interpretation": "No ground truth: shape score and point Chamfer are fit diagnostics, not pose accuracy."}
    (out / "search_comparison.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 阶段二：联合搜索对比", "", "各次运行的四份阶段 2 输入哈希、栅格、高度与可信度、平面坐标系和评分配置一致。",
             "", "| 运行 | 原始/规范候选 | 保留法向×方向分支 | 首位角度 | 首位栅格误差 | 首位点级 Chamfer | 搜索/整次耗时 |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for s in summaries:
        lines.append(f"| {s['name']} | {s['raw_count']}/{s['canonical_count']} | {s['covered_normal_direction_branches']} | {s['top_angle_deg']:.2f}° | {s['top_shape_score_m']*1000:.3f} mm | {s['top_point_chamfer_m']*1000:.3f} mm | {s['search_elapsed_s']:.2f}/{s['elapsed_s']:.2f} s |")
    lines += ["", "## 方向与平移覆盖", "", "角度在 PCA 平面坐标系内；分支为相距 90° 的四个方向区间，区间内独立优化后角度无需严格相差 90°。平移单位为 mm。", "",
              "| 运行 | 法向 | 方向分支 | 原始/规范 | 规范角度 | 平移 (u, v) | 最佳栅格误差 |",
              "| --- | --- | ---: | ---: | --- | --- | ---: |"]
    for s in summaries:
        for b in s["branches"]:
            angles = ", ".join(f"{a:.2f}" for a in b["angles_deg"])
            shifts = "; ".join(f"({u*1000:.2f}, {v*1000:.2f})" for u, v in b["shifts_uv_m"])
            score = f"{b['best_shape_score_m']*1000:.3f} mm" if b["best_shape_score_m"] is not None else "缺失"
            lines.append(f"| {s['name']} | {b['normal_branch']} | {b['direction_branch_deg']} | {b['raw_count']}/{b['canonical_count']} | {angles} | {shifts} | {score} |")
    lines += ["", "耗时是同机依次执行的单次观测，不是统计基准；搜索耗时不含拟合、候选门限和绘图，整次耗时包含这些步骤但不含 Python 导入。",
              "没有真实位姿标签，误差下降仅表示该匹配度量改善，不证明绝对位姿更准或方向唯一。",
              "原始候选为细化后的种子输出，完整粗搜索空间另见 candidate_report.json 的 angle_profile 与 total_evaluations；近似相同的种子可在 SE(3) 去重时合并。",
              "固定数量的阶段 4 接口及人工复核未在本项中执行；旧正式输出和旧选择 ID 未替换。", "",
              "![搜索覆盖与匹配误差](search_comparison.png)"]
    (out / "search_comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--baseline", type=Path, default=ROOT / "outputs/stage_03_grid_sequential")
    p.add_argument("--joint", type=Path, nargs="+", default=[ROOT / "outputs/stage_03_grid_joint"])
    p.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage_03_search_comparison")
    args = p.parse_args()
    compare([args.baseline, *args.joint], args.output_dir)
    print(args.output_dir / "search_comparison.md")


if __name__ == "__main__":
    main()
