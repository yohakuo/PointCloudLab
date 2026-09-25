"""Deterministic full-circle angle/translation search on observed shape grids."""

from __future__ import annotations

import time
import numpy as np
from scipy.spatial import cKDTree


def direction_branch(angle):
    return int(np.floor(((angle + 45.) % 360.) / 90.)) * 90


def angular_distance(a, b):
    return abs((a - b + 180.) % 360. - 180.)


def symmetric_offsets(radius, step):
    """Include zero and both exact bounds even when radius is not a step multiple."""
    values = np.arange(1, int(np.floor(radius / step + 1e-9)) + 1) * step
    values = np.unique(np.minimum(np.r_[values, radius], radius))
    return np.unique(np.r_[-values[::-1], 0., values])


def validate_joint_config(cfg):
    for key in ("angle_step_deg", "translation_step_m", "nms_angle_deg", "nms_translation_m"):
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"joint_search.{key} must be finite and positive")
    if cfg["angle_step_deg"] > 45:
        raise ValueError("joint_search.angle_step_deg must be <= 45 for four-branch coverage")
    for key in ("angle_start_deg", "translation_window_m"):
        if not np.isfinite(cfg[key]):
            raise ValueError(f"joint_search.{key} must be finite")
    if cfg["translation_window_m"] < 0:
        raise ValueError("joint_search.translation_window_m must be nonnegative")
    seeds = cfg["seeds_per_branch"]
    if isinstance(seeds, bool) or not isinstance(seeds, int) or seeds < 2:
        raise ValueError("joint_search.seeds_per_branch must be an integer >= 2")
    if not cfg["refinement_levels"]:
        raise ValueError("joint_search requires at least one refinement level")
    for level in cfg["refinement_levels"]:
        for key in ("angle_radius_deg", "angle_step_deg", "translation_radius_m", "translation_step_m"):
            if not np.isfinite(level[key]) or level[key] <= 0:
                raise ValueError(f"joint_search refinement {key} must be finite and positive")


class PreparedShapeScore:
    """Cache target trees; keep forward coordinates to preserve weighted tie ordering."""

    def __init__(self, source, target, cfg):
        self.source, self.target, self.cfg = source, target, cfg
        strong = target["confidence"] >= float(cfg["minimum_reference_confidence"])
        reference = target["centers"][strong] if np.any(strong) else target["centers"]
        self.target_reference = cKDTree(reference)
        self.target_contour = cKDTree(target["contour"])
        self.target_all = cKDTree(target["centers"])
        self.source_strong = source["confidence"] >= float(cfg["minimum_reference_confidence"])
        if not np.any(self.source_strong):
            self.source_strong = np.ones(len(source["centers"]), dtype=bool)

    def trimmed(self, distances, weights):
        order = np.argsort(distances, kind="stable")
        cumulative = np.cumsum(weights[order])
        keep = cumulative <= float(self.cfg["match_trim_fraction"]) * cumulative[-1]
        keep[0] = True
        return float(np.average(distances[order][keep], weights=weights[order][keep]))

    def at_angle(self, angle):
        q = np.deg2rad(angle)
        r = np.array([[np.cos(q), -np.sin(q)], [np.sin(q), np.cos(q)]])
        s, t, cfg = self.source, self.target, self.cfg
        cells, contour = s["centers"] @ r.T, s["contour"] @ r.T
        def score(shift):
            moved_cells, moved_contour = cells + shift, contour + shift
            # Inverse queries are algebraically equivalent but perturb exact grid
            # distance ties. With weighted trimming they can alter which weights
            # cross the trim cutoff, so use the reference scorer's coordinates.
            occupancy = .5 * (
                self.trimmed(self.target_reference.query(moved_cells)[0], s["confidence"])
                + self.trimmed(cKDTree(moved_cells[self.source_strong]).query(t["centers"])[0], t["confidence"]))
            outline = .5 * (
                self.trimmed(self.target_contour.query(moved_contour)[0], s["contour_confidence"])
                + self.trimmed(cKDTree(moved_contour).query(t["contour"])[0], t["contour_confidence"]))
            distances, index = self.target_all.query(moved_cells)
            near = distances <= 1.5 * float(cfg["grid_size_m"])
            height = float(cfg["height_clip_m"])
            if np.any(near):
                dh = np.abs(s["height_p50_m"][near] - t["height_p50_m"][index[near]])
                height = float(np.average(np.minimum(dh, height), weights=s["confidence"][near]))
            return (float(cfg["occupancy_weight"]) * occupancy
                    + float(cfg["contour_weight"]) * outline + float(cfg["height_weight"]) * height)
        return score


def joint_angle_translation_search(source, target, shape_cfg, cfg, score_at_angle=None):
    validate_joint_config(cfg)
    start = time.perf_counter()
    scorer = PreparedShapeScore(source, target, shape_cfg)
    window = float(cfg["translation_window_m"])
    offsets = symmetric_offsets(window, float(cfg["translation_step_m"]))
    deltas = np.array([(u, v) for u in offsets for v in offsets])
    # PCA alignment (0 and its competing increments) supplements a full sweep.
    angles = np.unique(np.round(np.r_[
        (float(cfg["angle_start_deg"]) + np.arange(0., 360., float(cfg["angle_step_deg"]))) % 360.,
        [0., 90., 180., 270.]], 10))
    target_center = np.median(target["centers"], axis=0)
    evaluations = 0

    def anchor(angle):
        q = np.deg2rad(angle)
        r = np.array([[np.cos(q), -np.sin(q)], [np.sin(q), np.cos(q)]])
        return target_center - np.median(source["centers"] @ r.T, axis=0)

    def evaluate(angle, shifts):
        nonlocal evaluations
        score = score_at_angle(angle) if score_at_angle is not None else scorer.at_angle(angle)
        rows = [{"angle_deg": float(angle % 360.), "shift_uv_m": shift.tolist(),
                 "score_m": score(shift)} for shift in shifts]
        evaluations += len(rows)
        return sorted(rows, key=order_key)

    def order_key(item):
        return item["score_m"], item["angle_deg"], *item["shift_uv_m"]

    pools = {b: [] for b in (0, 90, 180, 270)}
    profile = []
    for angle in angles:
        rows = evaluate(angle, anchor(angle) + deltas)
        branch = direction_branch(angle)
        pools[branch].extend(rows)
        best = rows[0]
        profile.append({**best, "direction_branch_deg": branch,
                        "translation_boundary": bool(window > 0 and np.any(np.isclose(
                            np.abs(np.asarray(best["shift_uv_m"]) - anchor(angle)), window, atol=1e-10)))})
    coarse_elapsed = time.perf_counter() - start
    coarse_evaluations = evaluations
    results, seed_audit = [], []
    for branch, pool in pools.items():
        seeds = []
        for item in sorted(pool, key=order_key):
            if all(angular_distance(item["angle_deg"], other["angle_deg"]) >= cfg["nms_angle_deg"]
                   or np.linalg.norm(np.asarray(item["shift_uv_m"]) - other["shift_uv_m"]) >= cfg["nms_translation_m"]
                   for other in seeds):
                seeds.append(item)
            if len(seeds) >= cfg["seeds_per_branch"]:
                break
        for index, seed in enumerate(seeds):
            current = seed
            trace = []
            for level in cfg["refinement_levels"]:
                angle_offsets = symmetric_offsets(level["angle_radius_deg"], level["angle_step_deg"])
                translation_offsets = symmetric_offsets(level["translation_radius_m"], level["translation_step_m"])
                local_deltas = np.array([(u, v) for u in translation_offsets for v in translation_offsets])
                best = current  # A local level can never degrade the parent.
                for a in np.unique((current["angle_deg"] + angle_offsets) % 360.):
                    if direction_branch(a) != branch:
                        continue
                    shifts = np.asarray(current["shift_uv_m"]) + local_deltas
                    # The configured global translation window is a hard bound.
                    valid = np.all(np.abs(shifts - anchor(a)) <= window + 1e-12, axis=1)
                    if not np.any(valid):
                        continue
                    trial = evaluate(a, shifts[valid])[0]
                    if order_key(trial) < order_key(best):
                        best = trial
                trace.append({"before_score_m": current["score_m"], **best})
                current = best
            boundary = bool(window > 0 and np.any(np.isclose(
                np.abs(np.asarray(current["shift_uv_m"]) - anchor(current["angle_deg"])), window, atol=1e-10)))
            result = {**current, "direction_branch_deg": branch, "peak_index": index,
                      "method": "joint_search", "translation_boundary": boundary}
            results.append(result)
            seed_audit.append({"direction_branch_deg": branch, "peak_index": index,
                               "coarse_seed": seed, "refinement_trace": trace, "result": result})
    return results, {"method": "joint", "pca_role": "coordinate frame and supplemental seeds only",
                     "angle_coverage_deg": 360., "coarse_angle_count": len(angles),
                     "translations_per_coarse_angle": len(deltas), "angle_profile": profile,
                     "seeds": seed_audit, "config": cfg,
                     "coarse_evaluations": coarse_evaluations, "total_evaluations": evaluations,
                     "coarse_elapsed_s": coarse_elapsed, "search_elapsed_s": time.perf_counter() - start,
                     "enumerated_increments_deg": [0, 90, 180, 270]}
