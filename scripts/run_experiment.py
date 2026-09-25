"""Run one experiment with a reproducible, Git-trackable record.

Usage examples are in docs/experiment_records.md. This runner never reuses an
output directory or experiment ID.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time
import zipfile


ROOT = Path(__file__).resolve().parents[1]
RECORDS = ROOT / "experiments"
ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{2,79}$")
ENV_NAMES = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "PYTHONHASHSEED", "CUDA_VISIBLE_DEVICES")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def display_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def resolve_path(value: str) -> Path:
    path = Path(value)
    return (path if path.is_absolute() else ROOT / path).resolve()


def file_record(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"required file missing: {path}")
    return {"path": display_path(path), "size_bytes": path.stat().st_size, "sha256": sha256_file(path)}


def named_files(values: list[str], kind: str) -> dict[str, Path]:
    result = {}
    for value in values:
        name, separator, raw_path = value.partition("=")
        if not separator or not re.fullmatch(r"[A-Za-z0-9_.-]+", name) or not raw_path:
            raise ValueError(f"{kind} must use NAME=PATH: {value}")
        if name in result:
            raise ValueError(f"duplicate {kind} name: {name}")
        result[name] = resolve_path(raw_path)
    return result


def git(*args: str) -> str:
    result = subprocess.run(["git", *args], cwd=ROOT, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def ensure_code_version() -> str:
    if not git("rev-parse", "--show-toplevel").replace("\\", "/").casefold() == ROOT.as_posix().casefold():
        raise ValueError("runner must be used from this Git repository")
    for line in git("status", "--porcelain", "--untracked-files=all").splitlines():
        name = line[3:].replace("\\", "/")
        if not name.startswith("experiments/"):
            raise ValueError(f"commit code/config changes before an experiment: {name}")
    return git("rev-parse", "HEAD")


def output_index(directory: Path) -> list[dict]:
    return [file_record(path) for path in sorted(directory.rglob("*")) if path.is_file()]


def archive_outputs(directory: Path, experiment_id: str) -> dict:
    archive = ROOT / "archives" / "experiments" / f"{experiment_id}.zip"
    archive.parent.mkdir(parents=True, exist_ok=True)
    if archive.exists():
        raise ValueError(f"archive already exists: {archive}")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_STORED) as bundle:
        for path in sorted(directory.rglob("*")):
            if path.is_file():
                bundle.write(path, path.relative_to(directory).as_posix())
    return file_record(archive)


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    if not ID_PATTERN.fullmatch(args.id) or args.id in (".", ".."):
        raise ValueError("experiment ID must contain 3–80 letters, digits, dots, dashes or underscores")
    record_dir = RECORDS / args.id
    output_dir = resolve_path(args.output_dir)
    if not output_dir.is_relative_to(ROOT / "outputs") or output_dir == ROOT / "outputs":
        raise ValueError("output directory must be a unique path under outputs/")
    if record_dir.exists() or output_dir.exists():
        raise ValueError("experiment ID or output directory already exists; choose a new one")
    code_commit = ensure_code_version()
    configs = named_files(args.config, "config")
    inputs = named_files(args.input, "input")
    for value in args.result_file:
        relative = Path(value)
        if relative.is_absolute() or ".." in relative.parts or not value:
            raise ValueError("result file must be relative to the output directory")
    if not args.result_file:
        raise ValueError("at least one --result-file is required")
    if args.dataset:
        dataset_path = resolve_path(args.dataset)
        if "dataset" in configs:
            raise ValueError("dataset was specified twice")
        configs["dataset"] = dataset_path
        dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
        for role, value in dataset.get("inputs", {}).items():
            if value:
                inputs.setdefault(f"dataset.{role}", resolve_path(value))
    for name, path in configs.items():
        if path.suffix.lower() in (".json", ".yaml"):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
            except (UnicodeError, json.JSONDecodeError):
                continue
            if isinstance(value, dict) and isinstance(value.get("input_paths"), dict):
                for role, raw_path in value["input_paths"].items():
                    if raw_path:
                        inputs.setdefault(f"{name}.{role}", resolve_path(raw_path))
    configs.setdefault("requirements_lock", ROOT / "requirements-lock.txt")
    config_records = {name: file_record(path) for name, path in configs.items()}
    input_records = {name: file_record(path) for name, path in inputs.items()}
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    if not command or not any("{output_dir}" in part for part in command):
        raise ValueError("command must include {output_dir} so results enter the unique output directory")
    command = [part.replace("{output_dir}", str(output_dir)) for part in command]
    output_dir.mkdir(parents=True)
    record_dir.mkdir(parents=True)
    snapshot_dir = record_dir / "configs"
    snapshot_dir.mkdir()
    for name, path in configs.items():
        suffix = path.suffix or ".txt"
        snapshot = snapshot_dir / f"{name}{suffix}"
        shutil.copyfile(path, snapshot)
        config_records[name]["snapshot"] = display_path(snapshot)
    started = dt.datetime.now(dt.timezone.utc)
    start = time.monotonic()
    log_path = output_dir / "runner.log"
    return_code = None
    error = None
    try:
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding="utf-8", errors="replace", env=os.environ.copy())
            assert process.stdout is not None
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end="", flush=True)
            return_code = process.wait()
    except OSError as exc:
        error = f"{type(exc).__name__}: {exc}"
    result_records = {}
    result_summary = {}
    result_dir = record_dir / "results"
    for value in args.result_file:
        relative = Path(value)
        source = output_dir / relative
        if source.is_file():
            result_records[value] = file_record(source)
            if source.suffix.lower() == ".json":
                try:
                    payload = json.loads(source.read_text(encoding="utf-8"))
                    if isinstance(payload, dict):
                        result_summary[value] = {key: payload[key] for key in
                            ("status", "candidate_counts", "total_attempts", "valid_runs", "failed_runs", "elapsed_s")
                            if key in payload}
                except (UnicodeError, json.JSONDecodeError):
                    pass
            if source.stat().st_size <= 1024 * 1024:
                target = result_dir / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
                result_records[value]["snapshot"] = display_path(target)
        else:
            result_records[value] = {"missing": True}
    missing_result = any(item.get("missing") for item in result_records.values())
    after_configs = {name: sha256_file(path) for name, path in configs.items()}
    after_inputs = {name: sha256_file(path) for name, path in inputs.items()}
    unchanged = (all(after_configs[name] == item["sha256"] for name, item in config_records.items())
                 and all(after_inputs[name] == item["sha256"] for name, item in input_records.items()))
    reported_failure = any(summary.get("status") in ("failed", "error") for summary in result_summary.values())
    status = "success" if return_code == 0 and not missing_result and unchanged and not reported_failure else "failed"
    outputs = output_index(output_dir)
    archive = archive_outputs(output_dir, args.id)
    manifest = {
        "schema": "pointcloudlab.experiment.v1", "id": args.id, "status": status,
        "started_at_utc": started.isoformat(),
        "finished_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "elapsed_s": round(time.monotonic() - start, 3),
        "code_commit": code_commit, "command": command, "cwd": display_path(ROOT),
        "environment": {"python": sys.version.split()[0], "platform": platform.platform(),
                        "variables": {name: os.environ.get(name) for name in ENV_NAMES}},
        "configs": config_records, "inputs": input_records,
        "output_directory": display_path(output_dir), "outputs": outputs,
        "results": result_records, "result_summary": result_summary,
        "inputs_and_configs_unchanged_during_run": unchanged,
        "exit_code": return_code, "error": error,
        "archive": archive,
    }
    write_json(record_dir / "manifest.json", manifest)
    print(f"record: {display_path(record_dir / 'manifest.json')}")
    return 0 if status == "success" else (return_code or 3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--dataset", help="dataset manifest; non-null input paths are hashed automatically")
    parser.add_argument("--config", action="append", default=[], metavar="NAME=PATH")
    parser.add_argument("--input", action="append", default=[], metavar="ROLE=PATH")
    parser.add_argument("--result-file", action="append", default=[], metavar="RELATIVE_PATH")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    try:
        return run(parser.parse_args())
    except (ValueError, OSError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        print(f"experiment setup failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
