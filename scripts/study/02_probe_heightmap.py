"""Compare board-relative object height maps without changing candidate transforms."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.stats import binned_statistic_2d


ROOT = Path(__file__).resolve().parents[2]
stage2 = ROOT / "outputs" / "stage_02_segmentation"
stage3 = ROOT / "outputs" / "stage_03_candidates"
report = json.loads((stage3 / "candidate_report.json").read_text(encoding="utf-8"))
candidates = json.loads((stage3 / "canonical_candidates.json").read_text(encoding="utf-8"))["candidates"]
source = np.asarray(o3d.io.read_point_cloud(str(stage2 / "source_object_points.ply")).points)
target = np.asarray(o3d.io.read_point_cloud(str(stage2 / "target_object_points.pcd")).points)
frame = report["planes"]["target"]["frame"]
origin = np.asarray(frame["origin"])
basis = np.asarray([frame["u"], frame["v"]]).T
plane = np.asarray(report["planes"]["target"]["oriented_plane_model"])
edges = np.arange(-.063, .0631, .003)


def heightmap(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    uv = (points - origin) @ basis
    heights = points @ plane[:3] + plane[3]
    args = (uv[:, 0], uv[:, 1], heights)
    med = binned_statistic_2d(*args, statistic="median", bins=[edges, edges]).statistic
    count = binned_statistic_2d(*args, statistic="count", bins=[edges, edges]).statistic
    return med, count


target_map, target_count = heightmap(target)
rows = []
maps = []
for candidate in candidates:
    matrix = np.asarray(candidate["matrix_m"])
    moved = source @ matrix[:3, :3].T + matrix[:3, 3]
    source_map, source_count = heightmap(moved)
    common = (target_count >= 3) & (source_count >= 3) & np.isfinite(target_map) & np.isfinite(source_map)
    difference = np.abs(target_map[common] - source_map[common])
    correlation = np.corrcoef(target_map[common], source_map[common])[0, 1] if common.sum() >= 3 else np.nan
    rows.append((candidate["rank"], candidate["provenance"]["relative_angle_deg"], int(common.sum()),
                 float(np.median(difference)), float(np.mean(np.minimum(difference, .02))), float(correlation)))
    maps.append(source_map)

print("rank angle_deg common_cells median_abs_height_m clipped_mean_m correlation")
for row in rows:
    print(*row)

fig, axes = plt.subplots(1, 3, figsize=(12, 4))
for ax, data, title in zip(axes, [target_map, maps[0], maps[7]], ["FAST", "INSPIRE candidate 1", "INSPIRE candidate 8"]):
    ax.imshow(data.T * 1000, origin="lower", extent=[edges[0] * 1000, edges[-1] * 1000] * 2,
              vmin=0, vmax=55, cmap="viridis")
    ax.set_title(title)
    ax.set_xlabel("u (mm)")
    ax.set_ylabel("v (mm)")
fig.tight_layout()
fig.savefig(stage3 / "heightmap_probe.png", dpi=150)
