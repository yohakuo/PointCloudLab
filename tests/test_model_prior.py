from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pointcloud_registration.model_prior import (combine_score, fit_supported_face,
                                                  reliable_radar_cells, score_supported_face)


class ModelPriorTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / "configs/stage3_model_prior.json").read_text())
        uv = np.array([(i * .003, j * .003) for i in range(-6, 7) for j in range(-6, 7)], float)
        h = .05 + .002 * uv[:, 0] - .003 * uv[:, 1]
        self.source = {"centers": uv, "height_p50_m": h.copy(),
                       "confidence": np.ones(len(uv)), "crop_cell": np.zeros(len(uv), bool)}
        self.target = {"centers": uv.copy(), "height_p25_m": h - .003,
                       "height_p75_m": h + .003, "confidence": np.ones(len(uv)),
                       "crop_cell": np.zeros(len(uv), bool)}
        self.frame = {"origin": [0., 0., 0.], "u": [1., 0., 0.], "v": [0., 1., 0.],
                      "n": [0., 0., 1.]}

    def test_reliable_finite_face_and_unobservable_thickness(self):
        report, model = fit_supported_face(self.source, self.cfg)
        self.assertTrue(report["valid"])
        self.assertIsNone(report["thickness_m"])
        self.assertEqual(report["supported_cell_count"], len(self.source["centers"]))
        score = score_supported_face(model, self.frame, self.frame, self.target, np.eye(4), self.cfg)
        self.assertTrue(score["valid"])
        self.assertGreater(score["observed_source_face_coverage"], .9)
        self.assertLess(score["distance_m"], .001)

    def test_lower_outliers_do_not_create_other_faces(self):
        source = {k: np.array(v, copy=True) for k, v in self.source.items()}
        source["height_p50_m"][:25] = .012
        report, model = fit_supported_face(source, self.cfg)
        self.assertTrue(report["valid"])
        self.assertGreaterEqual(report["excluded_other_height_cell_count"], 25)
        self.assertAlmostEqual(report["model_height_m"]["intercept"], .05, places=4)
        self.assertEqual(len(model["uv"]), report["supported_cell_count"])

    def test_high_outliers_do_not_create_false_front(self):
        source = {k: np.array(v, copy=True) for k, v in self.source.items()}
        source["height_p50_m"][:25] = .09
        report, _ = fit_supported_face(source, self.cfg)
        self.assertTrue(report["valid"])
        self.assertAlmostEqual(report["model_height_m"]["intercept"], .05, places=4)
        self.assertGreaterEqual(report["excluded_other_height_cell_count"], 25)

    def test_missing_face_and_low_coverage_fall_back(self):
        source = {k: np.asarray(v)[:40] for k, v in self.source.items()}
        report, model = fit_supported_face(source, self.cfg)
        self.assertFalse(report["valid"])
        self.assertIsNone(model)
        self.assertIn("insufficient_reliable_source_cells", report["reasons"])
        old = {"occupancy_m": .003, "outer_contour_m": .004, "height_m": .005}
        self.assertIsNone(combine_score({"valid": False}, old, self.cfg))
        sparse = {k: np.asarray(v)[::2] for k, v in self.source.items()}
        report, _ = fit_supported_face(sparse, self.cfg)
        self.assertFalse(report["valid"])
        self.assertIn("observed_front_coverage_too_low", report["reasons"])

    def test_strict_size_prior_conflict_falls_back_without_inventing_thickness(self):
        cfg = {**self.cfg, "nominal_face_size_m": .06, "face_size_sigma_m": .002}
        report, model = fit_supported_face(self.source, cfg)
        self.assertFalse(report["valid"])
        self.assertIsNone(model)
        self.assertIn("soft_size_prior_conflicts_with_observed_extent", report["reasons"])
        self.assertIsNone(report["thickness_m"])

    def test_degenerate_one_dimensional_support_falls_back(self):
        uv = np.column_stack((np.linspace(-.15, .15, 110), np.zeros(110)))
        source = {"centers": uv, "height_p50_m": np.full(110, .05),
                  "confidence": np.ones(110), "crop_cell": np.zeros(110, bool)}
        report, model = fit_supported_face(source, self.cfg)
        self.assertFalse(report["valid"])
        self.assertIsNone(model)
        self.assertIn("face_parameters_degenerate", report["reasons"])

    def test_outside_finite_surface_and_few_matches_fall_back(self):
        _, model = fit_supported_face(self.source, self.cfg)
        shifted = np.eye(4)
        shifted[0, 3] = .06
        score = score_supported_face(model, self.frame, self.frame, self.target, shifted, self.cfg)
        self.assertFalse(score["valid"])
        self.assertLess(score["matched_radar_cells"], self.cfg["minimum_matched_radar_cells"])
        self.assertEqual(score["reason"], "matched_radar_support_below_minimum")
        few = {k: np.asarray(v)[:10] for k, v in self.target.items()}
        score = score_supported_face(model, self.frame, self.frame, few, np.eye(4), self.cfg)
        self.assertFalse(score["valid"])
        self.assertEqual(score["reason"], "insufficient_prequalified_radar_cells")

    def test_interior_unobserved_hole_is_unknown(self):
        uv = self.source["centers"]
        retained = (np.abs(uv[:, 0]) > .006) | (np.abs(uv[:, 1]) > .006)
        source = {k: np.asarray(v)[retained] for k, v in self.source.items()}
        report, model = fit_supported_face(source, self.cfg)
        self.assertTrue(report["valid"])
        score = score_supported_face(model, self.frame, self.frame, self.target, np.eye(4), self.cfg)
        self.assertTrue(score["valid"])
        self.assertLess(score["observed_footprint_cells"], score["eligible_radar_cells"])

    def test_radar_reliability_is_candidate_independent(self):
        mask, weights, spread = reliable_radar_cells(self.target, self.cfg)
        self.target["crop_cell"][:5] = True
        mask2, weights2, spread2 = reliable_radar_cells(self.target, self.cfg)
        self.assertTrue(mask[:5].all())
        self.assertFalse(mask2[:5].any())
        np.testing.assert_array_equal(weights, weights2)
        np.testing.assert_array_equal(spread, spread2)
        _, model = fit_supported_face(self.source, self.cfg)
        good = score_supported_face(model, self.frame, self.frame, self.target, np.eye(4), self.cfg)
        shifted = np.eye(4)
        shifted[0, 3] = .06
        bad = score_supported_face(model, self.frame, self.frame, self.target, shifted, self.cfg)
        self.assertEqual(good["eligible_radar_cells"], bad["eligible_radar_cells"])
        self.assertEqual(good["eligible_weight"], bad["eligible_weight"])


if __name__ == "__main__":
    unittest.main()
