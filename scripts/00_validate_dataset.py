#!/usr/bin/env python3
"""Validate and display the unified dataset entry points without running registration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pointcloud_registration.dataset_config import DEFAULT_DATASET_FILE, load_dataset, stage_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查统一点云数据清单及各阶段入口")
    parser.add_argument("--dataset", type=Path, default=ROOT / DEFAULT_DATASET_FILE)
    args = parser.parse_args(argv)
    try:
        dataset = load_dataset(args.dataset, ROOT)
        errors: list[str] = []
        expected_suffix = {
            "source_full": ".ply", "source_roi": ".ply", "target_full": ".pcd", "target_roi": ".pcd",
            "target_board_manual": ".pcd", "target_object_manual": ".pcd",
        }
        print(f"数据集: {dataset.get('dataset_id', '<未命名>')}")
        print(f"清单: {dataset['manifest_path']}")
        print("输入:")
        for key, path in dataset["inputs"].items():
            if path is None:
                print(f"  {key}: <未配置>")
                continue
            status = "存在" if path.is_file() else "缺失"
            print(f"  {key}: {path} [{status}]")
            if not path.is_file():
                errors.append(f"{key} 文件不存在: {path}")
            suffix = expected_suffix.get(key)
            if suffix and path.suffix.lower() != suffix:
                errors.append(f"{key} 应为 {suffix}: {path}")
        print("阶段输出:")
        for stage in (1, 2, 3, 4, 5):
            print(f"  stage {stage}: {stage_dir(dataset, stage)}")
        if errors:
            print("检查失败:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
            return 2
        if dataset.get("target_selection_mode") == "manual_files" and (
            dataset["inputs"].get("target_board_manual") is None or dataset["inputs"].get("target_object_manual") is None
        ):
            print("检查失败: manual_files 模式需要两份 target 手工点云", file=sys.stderr)
            return 2
        print("检查通过。")
        return 0
    except Exception as exc:
        print(f"检查失败: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
