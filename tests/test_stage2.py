from __future__ import annotations

import sys
import tempfile
import unittest
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.segmentation import (
    automatic_target_selection_allowed, canonicalize_plane, choose_object_side, clean_cloud,
    coordinate_compatibility_report, exact_overlap_report, fit_plane_ransac,
    manual_geometry_report, select_target_object_heightmap, side_candidate_indices,
    signed_distances, stage2_contract,
)
from pointcloud_registration.visualization import save_segmentation_preview

SCRIPT_SPEC = importlib.util.spec_from_file_location("stage2_script", ROOT / "scripts" / "02_segment_regions.py")
assert SCRIPT_SPEC and SCRIPT_SPEC.loader
STAGE2_SCRIPT = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(STAGE2_SCRIPT)


def grid(z: float, nx: int = 20, ny: int = 20, span: float = .1) -> np.ndarray:
    x, y = np.meshgrid(np.linspace(-span, span, nx), np.linspace(-span, span, ny))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


def heightmap_config() -> dict:
    config = json.loads((ROOT / "configs" / "registration.yaml").read_text(encoding="utf-8"))
    result = dict(config["segmentation"]["target_heightmap"])
    result["minimum_refit_points"] = 100
    return result


def synthetic_heightmap_scene(*, strong: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    board = grid(0., 31, 31, .12)
    coordinates = np.arange(-.024, .0241, .004)
    x, y = np.meshgrid(coordinates, coordinates)
    keep = np.abs(x.ravel()) > 1e-8  # A missing one-cell seam must be bridged only in the decision grid.
    xy = np.column_stack((x.ravel()[keep], y.ravel()[keep]))
    edge = np.max(np.abs(xy), axis=1) >= .020
    depth = np.where(edge, -.003, -.012 if strong else -.003)
    obj = np.column_stack((xy, depth))
    # A large high-depth sheet has more points but violates the configured physical extent.
    background = grid(-.011, 30, 30, .105) + np.array([.28, 0., 0.])
    background[:, 2] = -np.random.default_rng(7).uniform(.008, .020, len(background))
    # A compact 41-point distractor is intentionally too small in projection.
    angles = np.linspace(0., 2 * np.pi, 41, endpoint=False)
    tiny = np.column_stack((-.09 + .007 * np.cos(angles), .075 + .007 * np.sin(angles),
                            np.full(41, -.013)))
    # Weak disconnected noise has no strong seed and must not be returned.
    weak_noise = np.array([[.08 + .002 * i, -.08 + .002 * j, -.003]
                           for i in range(5) for j in range(5)], dtype=float)
    points = np.vstack((board, obj, background, tiny, weak_noise))
    object_indices = np.arange(len(board), len(board) + len(obj))
    background_indices = np.arange(object_indices[-1] + 1, object_indices[-1] + 1 + len(background))
    noise_indices = np.arange(len(points) - len(weak_noise), len(points))
    return points, object_indices, background_indices, noise_indices


class StageTwoTests(unittest.TestCase):
    def _heightmap_selection(self, points: np.ndarray, *, config: dict | None = None,
                             high_override: float | None = None) -> dict:
        initial = fit_plane_ransac(points, threshold_m=.0005, ransac_n=3, iterations=500, seed=42)
        return select_target_object_heightmap(points, initial, config or heightmap_config(),
            requested_side="negative", source_extent_m=[.048, .048],
            high_threshold_override_m=high_override, plane_threshold_m=.0005,
            plane_ransac_n=3, plane_iterations=500, seed=42)

    def test_automatic_preview_accepts_source_board_from_full_cloud(self) -> None:
        board = grid(0.0, 12, 12); obj = grid(.01, 3, 3, .02)
        panel = {"title": "source", "points": obj, "board_points": board, "origin": board.mean(axis=0),
                 "plane": np.array([0., 0., 1., 0.]), "board": np.arange(len(board)),
                 "positive": np.empty(0, dtype=int), "negative": np.empty(0, dtype=int),
                 "selected": np.arange(len(obj)), "deleted": np.empty(0, dtype=int)}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "preview.png"
            save_segmentation_preview(path, [panel])
            self.assertTrue(path.is_file()); self.assertGreater(path.stat().st_size, 0)

    def test_manual_cli_files_must_be_paired(self) -> None:
        args = SimpleNamespace(target_board_file=Path("board.pcd"), target_object_file=None, target_selection_mode=None)
        with self.assertRaisesRegex(STAGE2_SCRIPT.Stage2Failure, "must be provided together"):
            STAGE2_SCRIPT.resolve_manual_target_paths(args, {"target_selection_mode": "manual_files", "target_manual_files": {}})

    def test_manual_same_resolved_path_is_rejected_by_contract(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "same.pcd"; path.write_bytes(b"same")
            with self.assertRaisesRegex(STAGE2_SCRIPT.Stage2Failure, "same file"):
                STAGE2_SCRIPT.validate_distinct_manual_paths(path, path.parent / "." / path.name)

    def test_different_paths_with_identical_file_content_are_detected(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            board, obj = Path(folder) / "board.pcd", Path(folder) / "object.pcd"
            board.write_bytes(b"identical bytes"); obj.write_bytes(b"identical bytes")
            identity = STAGE2_SCRIPT.manual_identity_report(board, obj, np.array([[1., 2., 3.]]), np.array([[1., 2., 3.]]))
            self.assertTrue(identity["sha256_identical"])
            self.assertTrue(identity["cleaned_xyz_identical"])

    def test_empty_and_non_finite_cloud_cleaning(self) -> None:
        empty = o3d.geometry.PointCloud()
        _, points, stats = clean_cloud(empty, 1.0)
        self.assertEqual(len(points), 0); self.assertEqual(stats["clean_point_count"], 0)
        with mock.patch.object(STAGE2_SCRIPT, "read_points", return_value=(empty, np.empty((0, 3)))):
            with self.assertRaisesRegex(STAGE2_SCRIPT.Stage2Failure, "empty after"):
                STAGE2_SCRIPT._load_cloud(Path("empty.pcd"), "target_manual_board", 1.0)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([[1, 2, 3], [np.nan, 0, 0], [np.inf, 0, 0]]))
        _, points, stats = clean_cloud(cloud, 1.0)
        np.testing.assert_allclose(points, [[1, 2, 3]])
        self.assertEqual(stats["nan_row_count"], 1); self.assertEqual(stats["inf_row_count"], 1)

    def test_coordinate_unit_anomaly_is_rejected(self) -> None:
        roi = grid(0., 10, 10, .1); full = grid(0., 10, 10, 1.)
        abnormal = roi * 1000.0 + np.array([1000., 0., 0.])
        report = coordinate_compatibility_report(abnormal, roi, full, full_margin_m=.01, roi_margin_m=.02, minimum_roi_fraction=.9)
        self.assertFalse(report["compatible"])

    def test_exact_manual_overlap_is_reported_without_deletion(self) -> None:
        board = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
        obj = np.array([[1., 0., 0.], [3., 0., 0.]])
        report = exact_overlap_report(board, obj)
        self.assertEqual(report["overlap_count"], 1)
        self.assertAlmostEqual(report["board_overlap_ratio"], 1 / 3)
        self.assertAlmostEqual(report["object_overlap_ratio"], 1 / 2)
        self.assertEqual(report["points_removed"], 0)

    def test_legal_manual_geometry_is_diagnostic_only(self) -> None:
        board = grid(0., 30, 30, .12)
        obj = grid(.01, 12, 12, .035)
        roi = np.vstack((board, obj))
        plane = fit_plane_ransac(board, threshold_m=.001, ransac_n=3, iterations=200, seed=42)
        report = manual_geometry_report(board, obj, roi, plane, cluster_eps_m=.01, cluster_min_points=3,
            minimum_object_points=30, minimum_projection_extent_m=.01, maximum_boundary_fraction=.5)
        self.assertFalse(report["board_plane_degenerate"])
        self.assertFalse(report["object"]["projection_degenerate"])
        self.assertEqual(report["points_modified_or_removed_by_diagnostics"], 0)
        self.assertIn("not physical thickness", report["signed_distance_to_manual_board_plane"]["interpretation"])

    def test_manual_mode_forbids_automatic_target_selection(self) -> None:
        self.assertFalse(automatic_target_selection_allowed("manual_files"))
        self.assertTrue(automatic_target_selection_allowed("automatic"))

    def test_mm_scale_about_origin_and_attribute_alignment(self) -> None:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([[1000, -2000, 500], [1000, -2000, 500], [0, 0, 0]]))
        cloud.colors = o3d.utility.Vector3dVector([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        cleaned, points, stats = clean_cloud(cloud, .001)
        np.testing.assert_allclose(points, [[1, -2, .5], [0, 0, 0]])
        np.testing.assert_allclose(np.asarray(cleaned.colors), [[1, 0, 0], [0, 0, 1]])
        self.assertEqual(stats["exact_duplicates_removed"], 1)

    def test_signed_sides_and_normalization(self) -> None:
        plane = canonicalize_plane(np.array([0., 0., -2., 0.]))
        self.assertAlmostEqual(np.linalg.norm(plane[:3]), 1.)
        points = np.array([[0, 0, .01], [0, 0, -.02], [0, 0, 0]])
        np.testing.assert_allclose(signed_distances(points, plane), [.01, -.02, 0])
        sides = side_candidate_indices(points, plane, .005)
        np.testing.assert_array_equal(sides["positive"], [0])
        np.testing.assert_array_equal(sides["negative"], [1])

    def test_source_roi_uses_external_full_plane(self) -> None:
        rng = np.random.default_rng(2)
        board = grid(0., 25, 25) + rng.normal(0, 2e-5, (625, 3))
        external = fit_plane_ransac(board, threshold_m=.0002, ransac_n=3, iterations=300, seed=4)
        roi = grid(.009, 20, 20)
        distances = signed_distances(roi, external.model)
        self.assertGreater(float(np.median(distances)), .008)
        # A fit inside ROI would instead call its dominant object face distance approximately zero.
        roi_fit = fit_plane_ransac(roi, threshold_m=.0002, ransac_n=3, iterations=100, seed=4)
        self.assertLess(abs(float(np.median(signed_distances(roi, roi_fit.model)))), 1e-9)

    def test_ambiguous_auto_preserves_both_and_override_selects(self) -> None:
        points = np.vstack((grid(.01), grid(-.01)))
        kwargs = dict(eps_m=.012, min_points=3, min_cluster_points=100, max_extent_m=.3,
                      centrality_limit=1., auto_score_ratio=1.5)
        auto = choose_object_side(points, [0, 0, 1, 0], .005, requested_side="auto", **kwargs)
        self.assertFalse(auto["confirmed"])
        self.assertGreater(len(auto["sides"]["positive"]["best_cluster_indices"]), 0)
        self.assertGreater(len(auto["sides"]["negative"]["best_cluster_indices"]), 0)
        manual = choose_object_side(points, [0, 0, 1, 0], .005, requested_side="negative", **kwargs)
        self.assertTrue(manual["confirmed"])
        self.assertEqual(manual["selected_side"], "negative")
        self.assertEqual(manual["selection_mode"], "manual_override")

    def test_small_outlier_removed_and_largest_not_blindly_selected(self) -> None:
        central = grid(.01, 14, 14, .04)
        tiny = grid(.01, 3, 3, .005) + np.array([.13, .13, 0])
        # A larger but physically over-scale connected sheet is rejected by extent.
        huge = grid(.01, 25, 25, .25) + np.array([.8, 0, 0])
        points = np.vstack((central, tiny, huge, grid(0., 10, 10, .05)))
        result = choose_object_side(points, [0, 0, 1, 0], .005, requested_side="positive",
            eps_m=.012, min_points=3, min_cluster_points=50, max_extent_m=.15,
            centrality_limit=1.5, auto_score_ratio=1.5)
        self.assertTrue(result["confirmed"])
        self.assertEqual(len(result["selected_indices"]), len(central))
        reports = result["sides"]["positive"]["clusters"]
        self.assertTrue(any(r.get("point_count") == len(huge) and not r.get("eligible") for r in reports))

    def test_output_points_are_unmodified_input_subset(self) -> None:
        points = np.vstack((grid(0.), grid(.01)))
        result = choose_object_side(points, [0, 0, 1, 0], .005, requested_side="positive",
            eps_m=.012, min_points=3, min_cluster_points=100, max_extent_m=.3,
            centrality_limit=1., auto_score_ratio=1.5)
        selected = points[result["selected_indices"]]
        for point in selected:
            self.assertTrue(np.any(np.all(points == point, axis=1)))
        np.testing.assert_allclose(selected[:, 2], .01)

    def test_stage_contract_forbids_registration(self) -> None:
        contract = stage2_contract()
        self.assertEqual(contract["stage"], 2)
        self.assertFalse(contract["registration_performed"])
        self.assertFalse(contract["transform_estimated"])
        self.assertEqual(contract["transform_direction"], "INSPIRE_TO_FAST")

    def test_target_heightmap_merges_fragmented_square_and_grows_weak_edges(self) -> None:
        points, object_indices, _, noise_indices = synthetic_heightmap_scene()
        result = self._heightmap_selection(points)
        self.assertTrue(result["confirmed"], result["selection_reason"])
        selected = result["selected_indices"]
        # Every original object point, including the low/high edge band and both sides of the seam, is restored.
        np.testing.assert_array_equal(np.sort(selected), object_indices)
        self.assertGreater(np.count_nonzero(np.abs(points[selected, 0]) >= .020), 0)
        self.assertEqual(len(np.intersect1d(selected, noise_indices)), 0)
        self.assertEqual(result["selection_mode"], "target_heightmap")
        self.assertFalse(result["output_contract"]["coordinates_modified"])

    def test_large_background_and_41_point_cluster_cannot_beat_square(self) -> None:
        points, object_indices, background_indices, _ = synthetic_heightmap_scene()
        result = self._heightmap_selection(points)
        np.testing.assert_array_equal(np.sort(result["selected_indices"]), object_indices)
        self.assertEqual(len(np.intersect1d(result["selected_indices"], background_indices)), 0)
        self.assertGreater(len(result["selected_indices"]), 41)

    def test_no_strong_seed_requires_manual_confirmation(self) -> None:
        board = grid(0., 31, 31, .12)
        weak = grid(-.003, 13, 13, .024)
        points = np.vstack((board, weak))
        result = self._heightmap_selection(points)
        self.assertFalse(result["confirmed"])
        self.assertEqual(result["selection_reason"], "no_strong_seed_cells")
        self.assertEqual(len(result["selected_indices"]), 0)

    def test_heightmap_indices_and_coordinates_are_original_target_subset(self) -> None:
        points, _, _, _ = synthetic_heightmap_scene()
        original = points.copy()
        result = self._heightmap_selection(points)
        np.testing.assert_array_equal(points, original)
        self.assertTrue(np.issubdtype(result["selected_indices"].dtype, np.integer))
        np.testing.assert_array_equal(points[result["selected_indices"]], original[result["selected_indices"]])
        self.assertTrue(np.all((result["selected_indices"] >= 0) & (result["selected_indices"] < len(points))))

    def test_heightmap_invalid_or_missing_configuration_is_explicit(self) -> None:
        points = np.vstack((grid(0., 20, 20, .1), grid(-.01, 8, 8, .02)))
        initial = fit_plane_ransac(points, threshold_m=.0005, ransac_n=3, iterations=200, seed=42)
        base = heightmap_config()
        for mutation, message in ((lambda c: c.pop("grid_size_m"), "missing required keys"),
                                  (lambda c: c.update(grid_size_m=0), "grid_size_m must be positive")):
            config = dict(base); mutation(config)
            with self.assertRaisesRegex(ValueError, message):
                select_target_object_heightmap(points, initial, config, requested_side="negative",
                    source_extent_m=[.04, .04], plane_threshold_m=.0005,
                    plane_ransac_n=3, plane_iterations=200, seed=42)
        with self.assertRaisesRegex(ValueError, "require 0 < low"):
            self._heightmap_selection(points, high_override=.001)

    def test_target_heightmap_does_not_change_legacy_source_selection(self) -> None:
        source = np.vstack((grid(0.), grid(.01, 14, 14, .04)))
        kwargs = dict(requested_side="positive", eps_m=.012, min_points=3,
                      min_cluster_points=100, max_extent_m=.3, centrality_limit=1., auto_score_ratio=1.5)
        before = choose_object_side(source, [0, 0, 1, 0], .005, **kwargs)["selected_indices"].copy()
        target, _, _, _ = synthetic_heightmap_scene()
        self._heightmap_selection(target)
        after = choose_object_side(source, [0, 0, 1, 0], .005, **kwargs)["selected_indices"]
        np.testing.assert_array_equal(after, before)

    def test_source_selector_still_calls_legacy_choose_object_side(self) -> None:
        source = np.vstack((grid(0.), grid(.01, 14, 14, .04)))
        plane = SimpleNamespace(model=np.array([0., 0., 1., 0.]),
                                report={"mad_robust_noise_sigma_m": .0001})
        config = json.loads((ROOT / "configs" / "registration.yaml").read_text(encoding="utf-8"))["segmentation"]
        args = SimpleNamespace(source_object_threshold_m=None, source_side="positive")
        with mock.patch.object(STAGE2_SCRIPT, "choose_object_side", wraps=choose_object_side) as legacy:
            selection, _ = STAGE2_SCRIPT._select_source(source, plane, config, args)
        legacy.assert_called_once()
        self.assertEqual(selection["requested_side"], "positive")


if __name__ == "__main__":
    unittest.main()
