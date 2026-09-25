"""Inventory existing experimental outputs without inventing historical provenance."""

from __future__ import annotations

import json
from pathlib import Path
import shutil

from run_experiment import RECORDS, ROOT, archive_outputs, display_path, file_record, output_index, resolve_path, sha256_file, write_json


HISTORICAL = {
    "stage_03_candidates": "candidate_report.json",
    "stage_03_sampled_baseline": "candidate_report.json",
    "stage_03_occupancy_grid": "candidate_report.json",
    "stage_03_grid_sequential": "candidate_report.json",
    "stage_03_grid_joint": "candidate_report.json",
    "stage_03_grid_joint_fast": "candidate_report.json",
    "stage_03_model_prior_rescore": "model_rescore.json",
    "stage_03_search_comparison": "search_comparison.json",
    "coarse_stability_pilot": "summary.json",
    "coarse_stability": "summary.json",
}


def observed_file(path_text: str | None, expected_sha256: str | None) -> dict:
    item = {"path": path_text, "reported_sha256": expected_sha256}
    if path_text:
        path = resolve_path(path_text)
        if path.is_file():
            item["current_sha256"] = sha256_file(path)
            item["current_matches_report"] = item["current_sha256"] == expected_sha256
        else:
            item["current_matches_report"] = False
    return item


def backfill(name: str, primary_name: str) -> None:
    output_dir = ROOT / "outputs" / name
    primary = output_dir / primary_name
    if not primary.is_file():
        raise ValueError(f"missing historical result: {primary}")
    record_dir = RECORDS / f"historical_{name}"
    if record_dir.exists():
        raise ValueError(f"record already exists: {record_dir}")
    report = json.loads(primary.read_text(encoding="utf-8"))
    configs = {}
    inputs = {}
    result_dir = record_dir / "results"
    result_dir.mkdir(parents=True)
    for filename in (primary_name, "paired_manifest.json", "report.md", "model_rescore.md", "search_comparison.md"):
        source = output_dir / filename
        if source.is_file() and source.stat().st_size <= 1024 * 1024:
            shutil.copyfile(source, result_dir / filename)
    if name.startswith("stage_03_") and primary_name == "candidate_report.json":
        if "config" in report:
            configs["registration.coarse_registration"] = {"source": "embedded in candidate_report.json", "effective": report["config"]}
        configs["cli_overrides"] = {"source": "embedded in candidate_report.json", "effective": report.get("cli_overrides")}
        for role, value in report.get("inputs", {}).items():
            inputs[role] = observed_file(value.get("path"), value.get("sha256"))
        summary = {"status": report.get("status"), "candidate_counts": report.get("candidate_counts"),
                   "elapsed_s": report.get("elapsed_s")}
    elif name.startswith("coarse_stability"):
        paired = json.loads((output_dir / "paired_manifest.json").read_text(encoding="utf-8"))
        configs["stability"] = {"source": "embedded in paired_manifest.json", "effective": paired["config"],
                                 "reported_sha256": paired.get("config_sha256")}
        configs["registration"] = {"reported_sha256": paired.get("registration_config_sha256")}
        configs["model_prior"] = {"reported_sha256": paired.get("model_config_sha256")}
        for role, expected in paired.get("input_file_sha256", {}).items():
            inputs[role] = observed_file(paired["config"]["input_paths"].get(role), expected)
        summary = {key: report.get(key) for key in ("total_attempts", "valid_runs", "failed_runs", "overall_failure_rate")}
        configs["implementation_file_hashes"] = {"source": "paired_manifest.json", "sha256": paired.get("implementation_sha256")}
    elif primary_name == "model_rescore.json":
        configs["model_prior"] = {"source": "embedded in model_rescore.json", "effective": report.get("config"),
                                  "reported_sha256": report.get("config_sha256")}
        for role, expected in report.get("input_hashes", {}).items():
            path = ROOT / "outputs/stage_02_segmentation" / f"{role}_points.{ 'ply' if role.startswith('source') else 'pcd'}"
            inputs[role] = observed_file(str(path), expected)
        for filename, expected in report.get("frozen_artifact_sha256", {}).items():
            inputs[f"frozen.{filename}"] = observed_file(str(ROOT / "outputs/stage_03_grid_joint" / filename), expected)
        summary = {key: report.get(key) for key in ("canonical_count", "canonical_old_rank_ids", "canonical_new_rank_ids")}
    else:
        runs = report.get("runs", [])
        for row in runs:
            if isinstance(row, dict):
                path_text = row.get("path") or row.get("directory")
                if path_text:
                    inputs[f"run.{len(inputs)}"] = {"path": path_text, "verification": "see comparison report"}
        summary = {"identical_input_hashes": report.get("identical_input_hashes"), "runs": len(runs)}
    manifest = {
        "schema": "pointcloudlab.experiment.v1", "id": record_dir.name,
        "historical": True, "status": "backfilled_from_existing_outputs",
        "code_commit": None, "code_version_note": "Run predates the first post-initialization commit; exact Git commit is unknown.",
        "command": None, "command_note": "Exact historical command is not recoverable from these reports.",
        "configs": configs, "inputs": inputs, "result_summary": summary,
        "output_directory": display_path(output_dir), "outputs": output_index(output_dir),
        "primary_result": {**file_record(primary), "snapshot": display_path(result_dir / primary_name)},
        "archive": archive_outputs(output_dir, record_dir.name),
    }
    write_json(record_dir / "manifest.json", manifest)
    print(record_dir.name)


def main() -> None:
    for name, primary in HISTORICAL.items():
        backfill(name, primary)


if __name__ == "__main__":
    main()
