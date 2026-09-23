from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.diagnostics import clean_points, plane_diagnostics
from pointcloud_registration.units import inspire_mm_to_m


class StageOneTests(unittest.TestCase):
    def test_inspire_scale_is_about_origin(self) -> None:
        points = np.array([[1000.0, -2000.0, 500.0], [0.0, 0.0, 0.0]])
        np.testing.assert_allclose(inspire_mm_to_m(points), [[1.0, -2.0, 0.5], [0.0, 0.0, 0.0]])

    def test_cleaning_counts_nonfinite_and_duplicates(self) -> None:
        points = np.array([[0, 0, 0], [0, 0, 0], [1, 2, 3], [np.nan, 0, 0], [np.inf, 0, 0]])
        cleaned, stats = clean_points(points, 100)
        self.assertEqual(len(cleaned), 2)
        self.assertEqual(stats["nan_row_count"], 1)
        self.assertEqual(stats["inf_row_count"], 1)
        self.assertEqual(stats["duplicate_point_count"], 1)

    def test_plane_normal_sign_and_protrusion_sides(self) -> None:
        rng = np.random.default_rng(7)
        xy = rng.uniform(-1, 1, (500, 2))
        board = np.column_stack((xy, rng.normal(0, 0.0002, 500)))
        points = np.vstack((board, [[0, 0, 0.02], [0, 0, -0.03]]))
        result = plane_diagnostics(points, threshold_m=0.001, ransac_n=3, iterations=500,
                                   protrusion_sigma_multiplier=3.0, protrusion_min_height_m=0.002)
        normal = np.asarray(result["plane_model_ax_by_cz_d"][:3])
        self.assertGreater(normal[np.argmax(np.abs(normal))], 0)
        self.assertGreaterEqual(result["positive_side_protrusion"]["count"], 1)
        self.assertGreaterEqual(result["negative_side_protrusion"]["count"], 1)


if __name__ == "__main__":
    unittest.main()
