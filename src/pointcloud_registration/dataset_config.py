"""Single-source dataset paths shared by every pipeline stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_DATASET_FILE = Path("configs/dataset.json")


class DatasetConfigError(ValueError):
    """Raised when the dataset manifest is missing or internally inconsistent."""


def _resolve(project_root: Path, value: str | Path | None) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (project_root / path).resolve()


def load_dataset(path: Path, project_root: Path) -> dict[str, Any]:
    """Load a JSON dataset manifest and resolve all paths against the project root."""
    manifest_path = path.expanduser()
    if not manifest_path.is_absolute():
        manifest_path = project_root / manifest_path
    manifest_path = manifest_path.resolve()
    if not manifest_path.is_file():
        raise DatasetConfigError(f"dataset manifest does not exist: {manifest_path}")
    try:
        raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetConfigError(f"cannot read dataset manifest {manifest_path}: {exc}") from exc
    if raw.get("schema_version") != "pointcloudlab.dataset.v1":
        raise DatasetConfigError("dataset schema_version must be pointcloudlab.dataset.v1")
    inputs = raw.get("inputs")
    if not isinstance(inputs, dict):
        raise DatasetConfigError("dataset inputs must be an object")
    required = ("source_full", "target_full", "source_roi", "target_roi")
    missing = [key for key in required if not inputs.get(key)]
    if missing:
        raise DatasetConfigError(f"dataset inputs missing: {', '.join(missing)}")
    units = raw.get("units", {})
    if units.get("source") != "mm" or units.get("target") != "m":
        raise DatasetConfigError("dataset units must be source=mm and target=m")
    output_root = _resolve(project_root, raw.get("output_root", "outputs"))
    assert output_root is not None
    resolved = dict(raw)
    resolved["manifest_path"] = manifest_path
    resolved["inputs"] = {key: _resolve(project_root, value) for key, value in inputs.items()}
    resolved["output_root"] = output_root
    return resolved


def stage_dir(dataset: dict[str, Any], stage: int) -> Path:
    names = {
        1: "stage_01_inspection",
        2: "stage_02_segmentation",
        3: "stage_03_candidates",
        4: "stage_04_refinement",
        5: "stage_05_export",
    }
    return dataset["output_root"] / names[stage]


def stage_defaults(dataset: dict[str, Any], stage: int, project_root: Path) -> dict[str, Any]:
    """Return argparse destination defaults for one stage."""
    inputs = dataset["inputs"]
    s1, s2, s3, s4 = (stage_dir(dataset, n) for n in (1, 2, 3, 4))
    common_clouds = {
        "source_board": s2 / "source_board_points.ply",
        "source_object": s2 / "source_object_points.ply",
        "target_board": s2 / "target_board_points.pcd",
        "target_object": s2 / "target_object_points.pcd",
    }
    if stage == 1:
        return {
            "source": inputs["source_full"], "target": inputs["target_full"],
            "source_roi": inputs["source_roi"], "target_roi": inputs["target_roi"],
            "data_dir": (project_root / "data").resolve(), "output_dir": s1,
        }
    if stage == 2:
        return {
            "source_full": inputs["source_full"], "target_full": inputs["target_full"],
            "source_roi": inputs["source_roi"], "target_roi": inputs["target_roi"],
            "target_board_file": inputs.get("target_board_manual"),
            "target_object_file": inputs.get("target_object_manual"),
            "target_selection_mode": dataset.get("target_selection_mode"), "output_dir": s2,
        }
    if stage == 3:
        return {**common_clouds, "stage2_report": s2 / "segmentation_report.json", "output_dir": s3}
    if stage == 4:
        return {**common_clouds, "stage2_report": s2 / "segmentation_report.json",
                "stage3_report": s3 / "candidate_report.json",
                "canonical_candidates": s3 / "canonical_candidates.json", "candidate_matrices": s3 / "candidate_matrices",
                "output_dir": s4}
    if stage == 5:
        selection = dataset.get("selection", {})
        return {"source_full": inputs["source_full"], "target_full": inputs["target_full"], **common_clouds,
                "stage1_report": s1 / "inspection_report.json", "stage2_report": s2 / "segmentation_report.json",
                "stage3_report": s3 / "candidate_report.json",
                "stage4_report": s4 / "refinement_report.json", "refined_candidates": s4 / "refined_candidates.json",
                "refinement_summary": s4 / "refinement_summary.csv", "refined_matrix_dir": s4 / "refined_matrices",
                "segmentation_preview": s2 / "segmentation_preview.png", "candidate_comparison": s3 / "candidate_comparison.png",
                "parent_candidate_id": selection.get("parent_candidate_id"),
                "selected_refined_candidate_id": selection.get("refined_candidate_id"), "output_dir": stage_dir(dataset, 5)}
    raise DatasetConfigError(f"unsupported stage: {stage}")


def parse_with_dataset(parser: argparse.ArgumentParser, argv: Sequence[str] | None,
                       project_root: Path, stage: int) -> tuple[argparse.Namespace, dict[str, Any]]:
    """Parse twice so the manifest supplies defaults while explicit CLI values win."""
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("--dataset", type=Path, default=project_root / DEFAULT_DATASET_FILE)
    known, _ = probe.parse_known_args(argv)
    dataset = load_dataset(known.dataset, project_root)
    parser.set_defaults(**stage_defaults(dataset, stage, project_root))
    args = parser.parse_args(argv)
    args.dataset = dataset["manifest_path"]
    args.dataset_id = dataset.get("dataset_id")
    args.dataset_approvals = dict(dataset.get("approvals", {}))
    return args, dataset


def add_dataset_argument(parser: argparse.ArgumentParser, project_root: Path) -> None:
    parser.add_argument("--dataset", type=Path, default=project_root / DEFAULT_DATASET_FILE,
                        help="统一数据清单；相对路径按项目根目录解析，显式 CLI 参数优先")


def ensure_report_dataset(report: dict[str, Any], dataset_id: str | None, stage: int) -> None:
    """Reject cross-dataset artifacts while accepting reports made before manifests existed."""
    recorded = report.get("dataset")
    if recorded is not None and recorded.get("id") != dataset_id:
        raise DatasetConfigError(
            f"stage {stage} report belongs to dataset {recorded.get('id')!r}, expected {dataset_id!r}"
        )
