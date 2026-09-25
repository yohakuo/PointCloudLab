from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.joint_search import (PreparedShapeScore, angular_distance,
    direction_branch, joint_angle_translation_search, symmetric_offsets, validate_joint_config)
from pointcloud_registration.shape_grid import shape_score
from pointcloud_registration.candidates import deduplicate_candidates


class JointSearchTests(unittest.TestCase):
    def setUp(self):
        cfg = json.loads((ROOT / "configs/registration.yaml").read_text())
        self.shape_cfg = cfg["coarse_registration"]["geometry"]["shape_grid"]
        self.cfg = {"angle_start_deg": 7., "angle_step_deg": 30., "translation_window_m": .008,
                    "translation_step_m": .004, "seeds_per_branch": 2, "nms_angle_deg": 8.,
                    "nms_translation_m": .004,
                    "refinement_levels": [
                        {"angle_radius_deg": 15., "angle_step_deg": 3., "translation_radius_m": .004, "translation_step_m": .002},
                        {"angle_radius_deg": 3., "angle_step_deg": 1., "translation_radius_m": .002, "translation_step_m": .001}]}
        rng = np.random.default_rng(75)
        points = rng.uniform([-.04, -.025], [.035, .02], (23, 2))
        self.source = {"centers": points, "contour": points[:12], "confidence": rng.uniform(.2, 1, len(points)),
                       "contour_confidence": rng.uniform(.3, 1, 12), "height_p50_m": rng.uniform(.004, .02, len(points))}

    def target(self, angle, shift):
        a = np.deg2rad(angle)
        r = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        return {**self.source, "centers": self.source["centers"] @ r.T + shift,
                "contour": self.source["contour"] @ r.T + shift}

    def test_prepared_score_matches_reference_with_confidence_and_height(self):
        target = self.target(37.2, [.005, -.01])
        scorer = PreparedShapeScore(self.source, target, self.shape_cfg)
        for angle, shift in [(17., [.002, -.003]), (355.2, [-.004, .008]), (92., [.2, .1])]:
            self.assertAlmostEqual(scorer.at_angle(angle)(shift),
                                   shape_score(self.source, target, angle, np.asarray(shift), self.shape_cfg)[0], places=12)

    def test_prepared_score_preserves_ties_on_regular_grid(self):
        xy = np.array([(i * .003, j * .003) for i in range(5) for j in range(6)])
        rng = np.random.default_rng(1)
        source = {"centers": xy, "contour": xy[:12], "confidence": rng.uniform(.2, 1, len(xy)),
                  "contour_confidence": rng.uniform(.2, 1, 12), "height_p50_m": rng.uniform(0, .02, len(xy))}
        target = {**source, "centers": xy + .003, "contour": xy[:12] + .003}
        scorer = PreparedShapeScore(source, target, self.shape_cfg)
        for angle in (0., 90., 180., 270.):
            for shift in (np.array([.003, -.003]), np.array([-.006, .009])):
                self.assertEqual(scorer.at_angle(angle)(shift), shape_score(source, target, angle, shift, self.shape_cfg)[0])

    def test_full_circle_recovers_known_transform_and_wraparound(self):
        # Disable confidence filtering here so exact synthetic correspondences have zero distance.
        shape_cfg = {**self.shape_cfg, "minimum_reference_confidence": 0.}
        for angle in (37., 359.):
            shift = np.array([.008, -.006])
            target = self.target(angle, shift)
            rows, audit = joint_angle_translation_search(self.source, target, shape_cfg, self.cfg)
            best = min(rows, key=lambda x: x["score_m"])
            self.assertLessEqual(angular_distance(best["angle_deg"], angle), 1.)
            self.assertLess(np.linalg.norm(np.asarray(best["shift_uv_m"]) - shift), .0015)
            self.assertEqual({r["direction_branch_deg"] for r in rows}, {0, 90, 180, 270})
            self.assertEqual(len(rows), 8)
            for seed in audit["seeds"]:
                for level in seed["refinement_trace"]:
                    self.assertLessEqual(level["score_m"], level["before_score_m"])
                    self.assertEqual(direction_branch(level["angle_deg"]), seed["direction_branch_deg"])
            self.assertEqual(audit["angle_coverage_deg"], 360.)
            self.assertTrue({0., 90., 180., 270.}.issubset({r["angle_deg"] for r in audit["angle_profile"]}))

    def test_deterministic_with_zero_translation_window(self):
        cfg = {**self.cfg, "translation_window_m": 0.}
        first, _ = joint_angle_translation_search(self.source, self.target(37., [.005, .003]), self.shape_cfg, cfg)
        second, _ = joint_angle_translation_search(self.source, self.target(37., [.005, .003]), self.shape_cfg, cfg)
        self.assertEqual(first, second)

    def test_independent_search_scorer_changes_candidate_pool(self):
        target = self.target(37., [.005, -.006])
        cfg = {**self.cfg, "translation_window_m": 0.}
        grid, _ = joint_angle_translation_search(self.source, target, self.shape_cfg, cfg)
        def model_score_at_angle(angle):
            return lambda shift: angular_distance(angle, 90.) * .001 + float(np.linalg.norm(shift)) * .001
        model, audit = joint_angle_translation_search(self.source, target, self.shape_cfg, cfg, model_score_at_angle)
        self.assertNotEqual(min(grid, key=lambda r: r["score_m"])["angle_deg"],
                            min(model, key=lambda r: r["score_m"])["angle_deg"])
        self.assertEqual(min(model, key=lambda r: r["score_m"])["angle_deg"], 90.)
        self.assertGreater(audit["total_evaluations"], 0)

    def test_bounds_and_invalid_configuration(self):
        np.testing.assert_allclose(symmetric_offsets(.01, .006), [-.01, -.006, 0., .006, .01])
        self.assertEqual([direction_branch(a) for a in (-45., 45., 315., 359., 405.)], [0, 90, 0, 0, 90])
        for key, value in [("angle_step_deg", 0.), ("angle_step_deg", 90.), ("translation_window_m", -1.),
                           ("translation_step_m", float("nan")), ("seeds_per_branch", 1), ("seeds_per_branch", 2.5)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_joint_config({**self.cfg, key: value})

    def test_canonical_limit_reserves_every_branch(self):
        rows = []
        for b in (0, 90, 180, 270):
            for k in range(3):
                a = np.deg2rad(b + k * 10)
                matrix = np.eye(4)
                matrix[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
                rows.append({"candidate_id": f"{b}_{k}", "generator": "geometry", "matrix_m": matrix.tolist(),
                             "status": "retained_raw", "coarse_rank_score": b + k,
                             "provenance": {"normal_alignment_branch": "observed", "discrete_increment_deg": b}})
        canonical, _ = deduplicate_candidates(copy.deepcopy(rows), 3., .006, 4, preserve_direction_branches=True)
        self.assertEqual({c["provenance"]["discrete_increment_deg"] for c in canonical}, {0, 90, 180, 270})
        with self.assertRaises(ValueError):
            deduplicate_candidates(copy.deepcopy(rows), 3., .006, 3, preserve_direction_branches=True)

    def test_nearby_candidates_cannot_merge_across_direction_boundary(self):
        rows = []
        for branch, angle in [(0, 44.), (90, 46.)]:
            a = np.deg2rad(angle)
            matrix = np.eye(4)
            matrix[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
            rows.append({"candidate_id": str(branch), "generator": "geometry", "matrix_m": matrix.tolist(),
                         "status": "retained_raw", "coarse_rank_score": 0.,
                         "provenance": {"normal_alignment_branch": "observed", "discrete_increment_deg": branch}})
        ordinary, _ = deduplicate_candidates(copy.deepcopy(rows), 3., .006, 4)
        protected, _ = deduplicate_candidates(copy.deepcopy(rows), 3., .006, 4, preserve_direction_branches=True)
        self.assertEqual(len(ordinary), 1)
        self.assertEqual(len(protected), 2)


if __name__ == "__main__":
    unittest.main()
