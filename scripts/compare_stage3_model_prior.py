#!/usr/bin/env python3
"""Rescore a frozen joint-search pool; never emit stage-4 input candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.model_prior import (combine_score, fit_supported_face,
                                                   score_supported_face)
from pointcloud_registration.shape_grid import shape_score


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(part)
    return h.hexdigest()


def frame(report, side):
    return {k: np.asarray(v, float) for k, v in report["planes"][side]["frame"].items()
            if k in ("origin", "u", "v", "n")}


def run(stage3_dir, out, config_path):
    start = time.perf_counter()
    stage3_dir, out, config_path = stage3_dir.resolve(), out.resolve(), config_path.resolve()
    if out == stage3_dir or stage3_dir in out.parents:
        raise ValueError("rescore output must be independent of the stage-3 candidate directory")
    report = read(stage3_dir / "candidate_report.json")
    if report["status"] != "success" or report["geometry_route"]["search_method"] != "joint":
        raise ValueError("input must be a successful frozen joint-search run")
    cfg = read(config_path)
    if abs(sum(cfg["score_weights"].values()) - 1) > 1e-10:
        raise ValueError("score weights must sum to one")
    inputs = {}
    for role, record in report["inputs"].items():
        path = Path(record["path"])
        actual = digest(path)
        if actual != record["sha256"]:
            raise ValueError(f"stage-2 input changed: {role}")
        inputs[role] = actual
    artifacts = {name: digest(stage3_dir / name) for name in
                 ("candidate_report.json", "raw_candidates.json", "canonical_candidates.json", "shape_grids.json")}
    exported = read(stage3_dir / "shape_grids.json")["grids"]
    grids = {name: {k: np.asarray(v) for k, v in value.items() if isinstance(v, list)}
             for name, value in exported.items()}
    if cfg["grid_size_m"] != exported["source"]["summary"]["grid_size_m"]:
        raise ValueError("model configuration grid size differs from frozen grid")
    fit, model = fit_supported_face(grids["source"], cfg)
    original = read(stage3_dir / "raw_candidates.json")["candidates"]
    canonical = read(stage3_dir / "canonical_candidates.json")["candidates"]
    canonical_ids = [c["candidate_id"] for c in canonical]
    source_frame = frame(report, "source")
    target_frame = frame(report, "target")
    shape_cfg = report["config"]["geometry"]["shape_grid"]
    rows = []
    for candidate in original:
        if candidate["status"] != "retained_raw":
            continue
        branch = candidate["provenance"]["normal_alignment_branch"]
        grid = grids[branch]
        angle = candidate["provenance"]["relative_angle_deg"]
        shift = np.asarray(candidate["generation_parameters"]["shift_uv_m"])
        old_score, old_terms = shape_score(grids["source"], grid, angle, shift, shape_cfg)
        if not np.isclose(old_score, candidate["shape_match"]["score_m"], atol=1e-12):
            raise ValueError("frozen old score does not reproduce")
        if model is None:
            model_term = {"valid": False, "reason": "source_fit_failed:" + ",".join(fit["reasons"])}
        elif branch != "observed_object_side":
            # The frozen report stores the observed target frame only.
            model_term = {"valid": False, "reason": "alternate_target_frame_not_frozen"}
        else:
            model_term = score_supported_face(model, source_frame, target_frame, grid,
                                              candidate["matrix_m"], cfg)
        new_score = combine_score(model_term, old_terms, cfg)
        rows.append({"candidate_id": candidate["candidate_id"],
                     "canonical_candidate_id": next((c["candidate_id"] for c in canonical
                         if candidate["candidate_id"] in c["member_candidate_ids"]), None),
                     "in_canonical_pool": candidate["candidate_id"] in {c["representative_raw_candidate_id"] for c in canonical},
                     "direction_branch_deg": candidate["provenance"]["discrete_increment_deg"],
                     "normal_branch": branch, "angle_deg": angle, "shift_uv_m": shift.tolist(),
                     "old_score_m": old_score, "old_terms_m": old_terms,
                     "new_score_m": new_score if new_score is not None else old_score,
                     "model_term": model_term, "fallback": new_score is None,
                     "fallback_reason": model_term.get("reason") if new_score is None else None,
                     "point_chamfer_m": candidate["coarse_checks"]["object"]["bidirectional_trimmed_chamfer_m"]})
    old_order = sorted(rows, key=lambda x: (x["old_score_m"], x["candidate_id"]))
    new_order = sorted(rows, key=lambda x: (x["new_score_m"], x["candidate_id"]))
    for ranking, key in ((old_order, "old_rank"), (new_order, "new_rank")):
        for index, row in enumerate(ranking, 1):
            row[key] = index
    representative_ids = {c["representative_raw_candidate_id"] for c in canonical}
    canonical_old = [x for x in old_order if x["candidate_id"] in representative_ids]
    canonical_new = [x for x in new_order if x["candidate_id"] in representative_ids]
    # Same frozen pool and fixed surface; dimensions only affect fit acceptance.
    size_sensitivity = []
    for nominal, sigma in ((.06, .003), (.06, .01), (.06, .015), (.06, .025), (.055, .015)):
        trial = {**cfg, "nominal_face_size_m": nominal, "face_size_sigma_m": sigma}
        assessment, _ = fit_supported_face(grids["source"], trial)
        size_sensitivity.append({"nominal_m": nominal, "sigma_m": sigma,
                                 "fit_valid": assessment["valid"], "reasons": assessment["reasons"],
                                 "size_z": assessment.get("soft_size_z")})
    weight_sensitivity = []
    for name, weights in (("model_dominant", cfg["score_weights"]),
                          ("balanced", {"model": .4, "occupancy": .2, "contour": .3, "height": .1}),
                          ("contour_heavy", {"model": .3, "occupancy": .15, "contour": .5, "height": .05})):
        trial = {**cfg, "score_weights": weights}
        ordered = sorted(rows, key=lambda x: (combine_score(x["model_term"], x["old_terms_m"], trial)
                       if x["model_term"]["valid"] else x["old_score_m"], x["candidate_id"]))
        weight_sensitivity.append({"name": name, "weights": weights,
                                   "top_candidate_id": ordered[0]["candidate_id"],
                                   "branch_order_deg": [x["direction_branch_deg"] for x in ordered]})
    elapsed = time.perf_counter() - start
    payload = {"schema": "pointcloudlab.stage3.model_prior_rescore", "version": 1,
               "scope": "frozen joint-search pool rescoring only; no search, refinement, stage-4 handoff or final transform",
               "input_directory": str(stage3_dir), "input_hashes": inputs, "frozen_artifact_sha256": artifacts,
               "config": cfg, "config_sha256": digest(config_path),
               "environment": {"python": sys.version, "platform": platform.platform(),
                               "numpy": np.__version__, "scipy": scipy.__version__},
               "model_fit": fit, "source_face_type": "single observed finite surface",
               "old_rank_ids": [x["candidate_id"] for x in old_order],
               "new_rank_ids": [x["candidate_id"] for x in new_order],
               "canonical_old_rank_ids": [x["canonical_candidate_id"] for x in canonical_old],
               "canonical_new_rank_ids": [x["canonical_candidate_id"] for x in canonical_new],
               "rows": rows, "canonical_count": len(canonical), "canonical_ids": canonical_ids,
               "four_direction_coverage_old": sorted({x["direction_branch_deg"] for x in old_order}),
               "four_direction_coverage_new": sorted({x["direction_branch_deg"] for x in new_order}),
               "size_sensitivity": size_sensitivity, "weight_sensitivity": weight_sensitivity,
               "search_elapsed_s_from_frozen_run": report["geometry_route"]["search_elapsed_s"],
               "rescore_elapsed_s_including_hash_validation": elapsed,
               "interpretation": "scores are metric match quality, not direction probabilities; no ground-truth pose available"}
    out.mkdir(parents=True, exist_ok=True)
    (out / "model_rescore.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# 阶段三：固定联合搜索候选池的模型重评分", "",
             "候选、白板拟合与栅格均从同一次已保存的联合搜索读取；本实验没有重新搜索或生成阶段 4 输入。",
             "", f"拟合有效：{fit['valid']}；原因：{', '.join(fit['reasons']) or '无'}。",
             f"已观测支持格：{fit.get('supported_cell_count', 0)}；覆盖率：{fit.get('observed_grid_coverage', 0):.3f}；残差 p90：{fit.get('residual_p90_m', 0)*1000:.3f} mm。",
             "只拟合一块正面有限表面；侧面、背面和厚度保持未知。", "",
             "权重：" + "、".join(f"{name} {value:.2f}" for name, value in cfg["score_weights"].items()) +
             "；各分项和权重完整保存在 JSON。",
             "", "| 新序 | 旧序 | 方向 | 原栅格分数 mm | 新分数 mm | 模型项 mm | 稳健距离 mm | 覆盖惩罚 mm | 占据格 mm | 外轮廓 mm | 高度 mm | 雷达支持格 | 雷达支持比例 | 源面覆盖 | 点级 Chamfer mm | 回退 |",
             "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"]
    for row in new_order:
        term = row["model_term"]
        lines.append(f"| {row['new_rank']} | {row['old_rank']} | {row['direction_branch_deg']}° | "
                     f"{row['old_score_m']*1000:.3f} | {row['new_score_m']*1000:.3f} | "
                     f"{term.get('distance_m', float('nan'))*1000:.3f} | "
                     f"{term.get('robust_supported_distance_m', float('nan'))*1000:.3f} | "
                     f"{term.get('coverage_penalty_m', float('nan'))*1000:.3f} | "
                     f"{row['old_terms_m']['occupancy_m']*1000:.3f} | {row['old_terms_m']['outer_contour_m']*1000:.3f} | "
                     f"{row['old_terms_m']['height_m']*1000:.3f} | {term.get('matched_radar_cells', 0)} | "
                     f"{term.get('radar_supported_fraction', 0):.3f} | {term.get('observed_source_face_coverage', 0):.3f} | "
                     f"{row['point_chamfer_m']*1000:.3f} | {row['fallback_reason'] or '无'} |")
    lines += ["", "## 四个规范候选的排序", "",
              "原评分：" + "、".join(f"{x['direction_branch_deg']}°" for x in canonical_old) + "。",
              "新评分：" + "、".join(f"{x['direction_branch_deg']}°" for x in canonical_new) + "。",
              "", "四个方向分支均保留。分数只比较此处定义的匹配度量，不是方向概率。",
              "同一分支的近似重复种子仍在原始池中；规范候选仍只有四个，不能直接送入要求 16 个的阶段 4。",
              "无真实位姿标签；任何分数改善均不能证明绝对姿态更准。", "",
              "## 敏感性", "", "尺寸软约束只影响拟合可靠性，不按方向施加偏好。"]
    for item in size_sensitivity:
        lines.append(f"- 标称 {item['nominal_m']*1000:.0f} mm，σ={item['sigma_m']*1000:.0f} mm：拟合 {'通过' if item['fit_valid'] else '回退'}（{', '.join(item['reasons']) or '无冲突'}）")
    for item in weight_sensitivity:
        lines.append(f"- 权重 {item['name']}：首位 {item['top_candidate_id']}；方向排序 {item['branch_order_deg']}")
    lines += ["", f"重评分及输入哈希复核耗时：{elapsed:.2f} s；原联合搜索耗时：{report['geometry_route']['search_elapsed_s']:.2f} s。",
              "原方法、规范候选和正式输出均未改写。"]
    (out / "model_rescore.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return payload


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage3-dir", type=Path, default=ROOT / "outputs/stage_03_grid_joint")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/stage_03_model_prior_rescore")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage3_model_prior.json")
    args = parser.parse_args()
    result = run(args.stage3_dir, args.output_dir, args.config)
    print(args.output_dir / "model_rescore.md")
