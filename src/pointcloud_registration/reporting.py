"""JSON and logging helpers for inspection reports."""

from __future__ import annotations

import json
import logging
import math
import os
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


def json_safe(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return [json_safe(v) for v in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    fd, name = tempfile.mkstemp(prefix=path.stem + ".", suffix=path.suffix, dir=path.parent)
    os.close(fd)
    temporary = Path(name)
    try:
        temporary.write_text(text, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def configure_logging(path: Path, logger_name: str = "pointcloud_registration", file_mode: str = "a") -> logging.Logger:
    path.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger(logger_name)
    logger.handlers.clear()
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    for handler in (logging.StreamHandler(), logging.FileHandler(path, mode=file_mode, encoding="utf-8")):
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    return logger
