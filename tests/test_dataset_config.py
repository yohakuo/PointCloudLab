from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.dataset_config import DatasetConfigError, ensure_report_dataset, load_dataset, stage_defaults


class DatasetConfigTests(unittest.TestCase):
    def test_paths_resolve_from_project_root_and_outputs_are_derived(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            manifest = root / "dataset.json"
            manifest.write_text(json.dumps({
                "schema_version": "pointcloudlab.dataset.v1", "dataset_id": "new_scan",
                "units": {"source": "mm", "target": "m"},
                "inputs": {"source_full": "a.ply", "target_full": "b.pcd", "source_roi": "c.ply", "target_roi": "d.pcd"},
                "output_root": "outputs/new_scan",
            }), encoding="utf-8")
            dataset = load_dataset(manifest, root)
            self.assertEqual(dataset["inputs"]["source_full"], (root / "a.ply").resolve())
            defaults = stage_defaults(dataset, 3, root)
            self.assertEqual(defaults["stage2_report"], (root / "outputs/new_scan/stage_02_segmentation/segmentation_report.json").resolve())
            self.assertEqual(defaults["source_object"], (root / "outputs/new_scan/stage_02_segmentation/source_object_points.ply").resolve())

    def test_required_inputs_and_units_are_checked(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); manifest = root / "dataset.json"
            manifest.write_text(json.dumps({"schema_version": "pointcloudlab.dataset.v1", "units": {"source": "m", "target": "m"}, "inputs": {}}), encoding="utf-8")
            with self.assertRaises(DatasetConfigError):
                load_dataset(manifest, root)

    def test_cross_dataset_report_is_rejected_but_legacy_report_is_allowed(self) -> None:
        ensure_report_dataset({"stage": 2}, "new_scan", 2)
        with self.assertRaises(DatasetConfigError):
            ensure_report_dataset({"stage": 2, "dataset": {"id": "old_scan"}}, "new_scan", 2)


if __name__ == "__main__":
    unittest.main()
