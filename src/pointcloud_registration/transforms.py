"""Rigid-transform and plane-frame helpers for stage 4 (INSPIRE -> FAST)."""

from __future__ import annotations

from typing import Any

import numpy as np


def validate_rigid_transform(matrix: np.ndarray, atol: float = 1e-6) -> dict[str, Any]:
    t = np.asarray(matrix, dtype=float)
    reasons: list[str] = []
    if t.shape != (4, 4):
        return {"valid": False, "reasons": ["matrix_shape_not_4x4"]}
    if not np.isfinite(t).all(): reasons.append("matrix_contains_nan_or_inf")
    if not np.allclose(t[3], [0, 0, 0, 1], atol=atol): reasons.append("invalid_last_row")
    r = t[:3, :3]
    det = float(np.linalg.det(r)) if np.isfinite(r).all() else None
    orth_error = float(np.linalg.norm(r.T @ r - np.eye(3))) if np.isfinite(r).all() else None
    if det is None or abs(det - 1.0) > atol: reasons.append("rotation_determinant_not_one")
    if orth_error is None or orth_error > atol: reasons.append("rotation_not_orthonormal")
    if det is not None and det < 0: reasons.append("reflection")
    return {"valid": not reasons, "reasons": reasons, "determinant": det,
            "orthonormality_error_fro": orth_error, "last_row_valid": "invalid_last_row" not in reasons,
            "no_reflection": det is not None and det > 0, "no_scale": not reasons or "rotation_not_orthonormal" not in reasons}


def apply_transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Apply p_target = T_target_from_source @ p_source."""
    p = np.asarray(points, dtype=float); t = np.asarray(matrix, dtype=float)
    return p @ t[:3, :3].T + t[:3, 3]


def rotation_between_normals(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, str]:
    a = np.asarray(source, dtype=float); b = np.asarray(target, dtype=float)
    a /= np.linalg.norm(a); b /= np.linalg.norm(b)
    dot = float(np.clip(a @ b, -1.0, 1.0))
    if dot > 1.0 - 1e-12: return np.eye(3), "parallel"
    if dot < -1.0 + 1e-12:
        axis_seed = np.eye(3)[int(np.argmin(np.abs(a)))]
        axis = np.cross(a, axis_seed); axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3), "antiparallel"
    cross = np.cross(a, b); s = np.linalg.norm(cross); k = np.array(
        [[0, -cross[2], cross[1]], [cross[2], 0, -cross[0]], [-cross[1], cross[0], 0]])
    return np.eye(3) + k + k @ k * ((1.0 - dot) / (s * s)), "general"


def oriented_plane(model: np.ndarray, object_points: np.ndarray, weak_epsilon_m: float) -> tuple[np.ndarray, dict[str, Any]]:
    plane = np.asarray(model, dtype=float).copy(); plane /= np.linalg.norm(plane[:3])
    signed = np.asarray(object_points) @ plane[:3] + plane[3]
    median = float(np.median(signed)); p25, p75 = np.percentile(signed, [25, 75])
    weak = abs(median) <= weak_epsilon_m
    if median < 0: plane *= -1
    return plane, {"rule": "orient normal so the observed object signed-distance median is non-negative",
                   "pre_orientation_object_signed_distance_m": {"median": median, "p25": float(p25), "p75": float(p75)},
                   "sign_evidence_weak": bool(weak), "weak_epsilon_m": float(weak_epsilon_m),
                   "opposite_alignment_branch_required": bool(weak)}


def right_handed_plane_frame(plane: np.ndarray, object_points: np.ndarray) -> dict[str, Any]:
    n = np.asarray(plane[:3], dtype=float); n /= np.linalg.norm(n)
    origin = np.median(np.asarray(object_points), axis=0)
    origin = origin - (origin @ n + float(plane[3])) * n
    projected = np.asarray(object_points) - np.outer(np.asarray(object_points) @ n + float(plane[3]), n)
    xy = projected - origin
    cov = xy.T @ xy / max(len(xy), 1)
    values, vectors = np.linalg.eigh(cov); order = np.argsort(values)[::-1]
    u = vectors[:, order[0]]; u -= n * (u @ n); u /= np.linalg.norm(u)
    pivot = int(np.argmax(np.abs(u)))
    if u[pivot] < 0: u *= -1
    v = np.cross(n, u); v /= np.linalg.norm(v)
    basis = np.column_stack((u, v, n))
    uv = np.column_stack(((projected-origin) @ u, (projected-origin) @ v))
    eig2 = np.linalg.eigvalsh(np.cov(uv.T))[::-1]
    return {"origin": origin, "u": u, "v": v, "n": n, "basis": basis, "uv": uv,
            "determinant": float(np.linalg.det(basis)), "pca_eigenvalues": eig2,
            "pca_relative_gap": float(abs(eig2[0]-eig2[1]) / max(eig2[0], np.finfo(float).eps))}


def compose_plane_transform(source_frame: dict[str, Any], target_frame: dict[str, Any], angle_deg: float,
                            inplane_shift_uv: np.ndarray) -> np.ndarray:
    angle = np.deg2rad(angle_deg); c, s = np.cos(angle), np.sin(angle)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
    r = target_frame["basis"] @ rz @ source_frame["basis"].T
    shift = target_frame["u"] * inplane_shift_uv[0] + target_frame["v"] * inplane_shift_uv[1]
    # Both recorded origins lie on their fitted plane.  Mapping source origin to
    # target origin therefore solves normal translation without treating cropped
    # board bounds as corresponding physical edges; only the explicit object-
    # projection hypothesis contributes an in-plane shift.
    t = target_frame["origin"] - r @ source_frame["origin"] + shift
    out = np.eye(4); out[:3, :3] = r; out[:3, 3] = t
    return out


def rotation_difference_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = np.asarray(a)[:3, :3].T @ np.asarray(b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(relative)-1.0)/2.0, -1.0, 1.0))))


def transform_difference(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    return rotation_difference_deg(a, b), float(np.linalg.norm(np.asarray(a)[:3, 3]-np.asarray(b)[:3, 3]))
