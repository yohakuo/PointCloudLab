"""Paired resampling audit of existing stage-3 object candidates."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.visualization import save_stage3_candidates
base = ROOT / "outputs" / "stage_02_segmentation"
stage3 = ROOT / "outputs" / "stage_03_candidates"
source = np.asarray(o3d.io.read_point_cloud(str(base / "source_object_points.ply")).points)
target = np.asarray(o3d.io.read_point_cloud(str(base / "target_object_points.pcd")).points)
candidates = json.loads((stage3 / "canonical_candidates.json").read_text(encoding="utf-8"))["candidates"]
chosen = {}
for candidate in candidates:
    direction = candidate["provenance"]["discrete_increment_deg"]
    chosen.setdefault(direction, candidate)
selected = next(c for c in candidates if c["candidate_id"] == "s3_can_4080ef322ffc")
chosen["selected"] = selected


def trimmed_mean(distances: np.ndarray, fraction: float = .8) -> float:
    count = max(1, int(np.ceil(len(distances) * fraction)))
    return float(np.mean(np.partition(distances, count - 1)[:count]))


matrix = {name: np.asarray(candidate["matrix_m"]) for name, candidate in chosen.items()}
scores = {name: [] for name in chosen}
for seed in range(25):
    rng = np.random.default_rng(91000 + seed)
    src = source[np.sort(rng.choice(len(source), 1800, replace=False))]
    tgt = target[np.sort(rng.choice(len(target), 1800, replace=False))]
    target_tree = cKDTree(tgt)
    for name, transform in matrix.items():
        moved = src @ transform[:3, :3].T + transform[:3, 3]
        forward = target_tree.query(moved, workers=1)[0]
        backward = cKDTree(moved).query(tgt, workers=1)[0]
        scores[name].append(.5 * (trimmed_mean(forward) + trimmed_mean(backward)))

print("direction candidate mean_mm sd_mm wins")
winner = [min(scores, key=lambda name: scores[name][i]) for i in range(25)]
for name, candidate in chosen.items():
    values = np.asarray(scores[name]) * 1000
    print(name, candidate["candidate_id"], round(float(values.mean()), 3),
          round(float(values.std(ddof=1)), 3), winner.count(name))

for name in (180, "selected"):
    difference = (np.asarray(scores[name]) - np.asarray(scores[270])) * 1000
    print("paired gap versus 270 (mm)", name, "mean", round(float(difference.mean()), 3),
          "p05/p95", np.percentile(difference, [5, 95]).round(3).tolist())

save_stage3_candidates(stage3 / "best_vs_current_coarse.png", [chosen[270], selected], source, target,
                       "Best geometric coarse candidate versus current selected parent")
