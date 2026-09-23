"""Unit conversion with an explicit INSPIRE -> FAST direction."""

from __future__ import annotations

import numpy as np

INSPIRE_MM_TO_M = 0.001


def inspire_mm_to_m(points_mm: np.ndarray) -> np.ndarray:
    """Scale XYZ about the fixed origin (0, 0, 0); never about a centroid."""
    points = np.asarray(points_mm, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"expected an (N, 3) point array, got {points.shape}")
    return points * INSPIRE_MM_TO_M

