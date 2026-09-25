import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("stability", ROOT / "scripts/study_coarse_perturbations.py")
stability = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = stability
spec.loader.exec_module(stability)


def candidate(rotation_deg=0, translation=(0, 0, 0), score=.001):
    q = np.deg2rad(rotation_deg)
    matrix = np.eye(4)
    matrix[:3, :3] = [[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1]]
    matrix[:3, 3] = translation
    return {"matrix_m": matrix.tolist(), "coarse_rank_score": score,
            "shape_match": {"score_m": score}, "coarse_checks": {"object": {"bidirectional_trimmed_chamfer_m": .004}},
            "generation_parameters": {"translation_boundary": False}}


class CoarseStabilityTests(unittest.TestCase):
    def test_pair_manifest_is_method_independent_and_voxel_seed_repeats(self):
        config = {"seed": 10, "groups": ["spatial", "combined"], "strengths": {"low": {}, "high": {}}, "maximum_repeats": 2,
                  "combined_components": ["boundary", "spatial", "board"],
                  "boundary_direction_rule": "even_repeat_erodes_odd_repeat_dilates"}
        rows = stability.make_manifest(config)
        self.assertEqual(len(rows), 9)
        self.assertEqual(len({x["seed"] for x in rows}), len(rows))
        points = np.array([[.001*i, .002*j, 0] for i in range(10) for j in range(10)])
        self.assertEqual(stability.hash_points(stability.voxel_resample(points, .003, 42)),
                         stability.hash_points(stability.voxel_resample(points, .003, 42)))
        row = {"group": "spatial", "strength": "low", "seed": 42, "components": ["spatial"], "repeat": 0}
        cfg = {"strengths": {"low": {"source_voxel_m": .003, "target_voxel_m": .003}}}
        original = {name: points for name in stability.ROLES}
        first, _ = stability.perturb(original, {}, {}, row, cfg)
        second, _ = stability.perturb(original, {}, {}, row, cfg)
        self.assertEqual({name: stability.hash_points(first[name]) for name in stability.ROLES},
                         {name: stability.hash_points(second[name]) for name in stability.ROLES})
        self.assertEqual(stability.hash_points(first["source_board"]), stability.hash_points(points))
        self.assertNotEqual(stability.hash_points(first["source_object"]), stability.hash_points(points))
        board_row = {"group": "board", "strength": "low", "seed": 42, "components": ["board"], "repeat": 0}
        board_cfg = {"strengths": {"low": {"board_voxel_m": .003}}}
        board_only, _ = stability.perturb(original, {}, {}, board_row, board_cfg)
        self.assertEqual(stability.hash_points(board_only["target_object"]), stability.hash_points(points))
        self.assertNotEqual(stability.hash_points(board_only["target_board"]), stability.hash_points(points))

    def test_fixed_frame_uses_same_source_point(self):
        frame = {"origin": np.array([10., 0, 0]), "u": np.array([1., 0, 0]), "v": np.array([0, 1., 0])}
        matrix = np.eye(4); matrix[:3, 3] = [11, 2, 9]
        self.assertEqual(stability.project_fixed(matrix, np.array([1., 3, 4]), frame), [2., 5.])

    def test_rotation_assignment_ignores_ids_and_axis_labels(self):
        reference = [candidate(0), candidate(90), candidate(180), candidate(270)]
        observed = [candidate(270.4), candidate(180.3), candidate(0.2), candidate(90.1)]
        matches = stability.match_branches(reference, observed, np.array([0,0,1]), np.array([0,0,1]), 40)
        self.assertEqual([matches[i]["candidate_index"] for i in range(4)], [2,3,1,0])
        self.assertAlmostEqual(max(x["rotation_distance_deg"] for x in matches.values()), .4)

    def test_normal_branch_separate_and_missing_branch(self):
        reference = [candidate(0), candidate(90)]
        flipped = candidate(0)
        flipped["matrix_m"][1][1] = -1
        flipped["matrix_m"][2][2] = -1
        observed = [candidate(90), flipped]
        matches = stability.match_branches(reference, observed, np.array([0,0,1]), np.array([0,0,1]), 40)
        self.assertEqual(set(matches), {1})

    def test_near_duplicate_rotations_choose_best_scored_pose(self):
        candidates = [candidate(0, score=.002), candidate(1, translation=(.01,0,0), score=.001), candidate(90, score=.003)]
        representatives = stability.direction_representatives(candidates, "grid_joint", np.array([0,0,1]), np.array([0,0,1]), 35)
        self.assertEqual(len(representatives), 2)
        self.assertEqual(representatives[0]["coarse_rank_score"], .001)

    def test_near_tie_and_missing_count(self):
        reference = [candidate(0, score=.001), candidate(90, score=.0011)]
        result = {"canonical_candidates": reference, "raw_candidates": reference, "model_fallback_candidates": 0}
        frame = {"origin": np.zeros(3), "u": np.array([1,0,0]), "v": np.array([0,1,0])}
        analysis = stability.compact_run("grid_joint", result, reference, np.array([.01,0,0]), frame,
            np.array([0,0,1]), np.array([0,0,1]), {"maximum_rotation_distance_deg": 40, "near_tie_gap_m": .00025})
        self.assertTrue(analysis["near_tie"])
        result["canonical_candidates"] = [reference[0]]
        analysis = stability.compact_run("grid_joint", result, reference, np.zeros(3), frame,
            np.array([0,0,1]), np.array([0,0,1]), {"maximum_rotation_distance_deg": 40, "near_tie_gap_m": .00025})
        self.assertEqual(analysis["branch_missing_count"], 1)
        self.assertFalse(analysis["near_tie"])

    def test_failure_denominator_and_model_fallback(self):
        fit = {"valid": False, "reasons": ["too_few_supported_front_cells"]}
        c = candidate()
        c["shape_match"].update({"occupancy_m": .001, "outer_contour_m": .001, "height_m": .001})
        c["provenance"] = {"normal_alignment_branch": "observed_object_side"}
        assessment = stability.model_assessment(c, fit, None, {}, [], {}, {"score_weights": {}})
        self.assertTrue(assessment["fallback"])
        self.assertIn("too_few_supported_front_cells", assessment["fallback_reason"])
        self.assertEqual(assessment["score_m"], c["shape_match"]["score_m"])
        records = [{"method": "grid_joint", "group": "spatial", "strength": "low", "status": "failed", "failure_reason": "plane_fit"},
                   {"method": "grid_joint", "group": "spatial", "strength": "low", "status": "success", "repeat": 1, "elapsed_s": 1,
                    "analysis": {"winner_branch": 0, "first_second_gap_m": None, "near_tie": False,
                      "branch_missing_count": 1, "boundary_hits": 0, "model_fallback_candidates": 0,
                      "candidate_coverage": {"raw": 2, "canonical": 1, "orientation_clusters": 1},
                      "winner_switched_from_reference": False, "winner_position_shift_from_reference_m": 0,
                      "within_run_max_cross_direction_position_m": None,
                      "branches": [{"branch": 0, "present": True, "xy_m": [0,0], "offset_m": 0,
                                    "rotation_distance_deg": 0, "score_m": .001}]}}]
        summary, _ = stability.summarize(records, [{"group": "spatial", "strength": "low"}], ["grid_joint"], {"batch_size": 3})
        self.assertEqual(summary[0]["attempts"], 2)
        self.assertEqual(summary[0]["valid_runs"], 1)
        self.assertEqual(summary[0]["failure_rate"], .5)


if __name__ == "__main__": unittest.main()
