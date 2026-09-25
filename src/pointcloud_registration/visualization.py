"""Deterministic, non-mutating stage 2 visualizations."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    n = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
    reference = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, reference); u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def save_segmentation_preview(path: Path, panels: list[dict[str, Any]], seed: int = 42) -> None:
    fig, axes = plt.subplots(1, len(panels), figsize=(8 * len(panels), 7), squeeze=False)
    rng = np.random.default_rng(seed)
    colors = {"board": "#a9adb4", "positive": "#ef476f", "negative": "#118ab2",
              "selected": "#06d6a0", "deleted": "#ffd166"}
    for ax, panel in zip(axes[0], panels):
        normal = np.asarray(panel["plane"][:3]); u, v = plane_basis(normal)
        origin = np.asarray(panel.get("origin", np.asarray(panel["points"]).mean(axis=0)))
        def draw(indices: np.ndarray, key: str, label: str, size: float, limit: int = 30000) -> None:
            points = np.asarray(panel.get(f"{key}_points", panel["points"]))
            idx = np.asarray(indices, dtype=np.int64)
            if len(idx) > limit: idx = rng.choice(idx, limit, replace=False)
            if len(idx):
                q = points[idx] - origin
                ax.scatter(q @ u, q @ v, s=size, c=colors[key], label=f"{label} (n={len(indices)})", alpha=.75, linewidths=0)
        draw(panel["board"], "board", "board", 1.0)
        draw(panel["positive"], "positive", "+ candidate", 2.0)
        draw(panel["negative"], "negative", "- candidate", 2.0)
        draw(panel["deleted"], "deleted", "deleted outlier", 3.0)
        draw(panel["selected"], "selected", "selected object", 3.0)
        ax.set_title(panel["title"]); ax.set_xlabel("plane u (m)"); ax.set_ylabel("plane v (m)")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2); ax.legend(loc="best", fontsize=8)
    fig.suptitle("Stage 2 segmentation — no registration / fixed plane-local views", fontsize=14)
    fig.tight_layout(); path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight"); plt.close(fig)


def save_manual_segmentation_preview(path: Path, *, source_board: np.ndarray, source_object: np.ndarray,
                                     source_plane: np.ndarray,
                                     target_board: np.ndarray, target_object: np.ndarray,
                                     target_plane: np.ndarray, target_plane_inliers: np.ndarray,
                                     seed: int = 42) -> None:
    """Six fixed plane-local views for manual target review; inputs are never modified."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 11), squeeze=False)
    rng = np.random.default_rng(seed)
    colors = {"board": "#7f8c9a", "object": "#e63946", "outlier": "#f4a261", "plane": "#457b9d"}

    def project(points: np.ndarray, plane: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        u, v = plane_basis(np.asarray(plane[:3]))
        origin = np.mean(points, axis=0)
        q = points - origin
        return q @ u, q @ v, q @ (np.asarray(plane[:3]) / np.linalg.norm(plane[:3]))

    def sample(points: np.ndarray, limit: int = 35000) -> np.ndarray:
        if len(points) <= limit:
            return points
        return points[rng.choice(len(points), limit, replace=False)]

    def combined(ax: Any, board: np.ndarray, obj: np.ndarray, plane: np.ndarray, title: str) -> None:
        origin = np.mean(np.vstack((board, obj)), axis=0)
        u, v = plane_basis(np.asarray(plane[:3]))
        for points, key, label, size in ((board, "board", "board", 1.2), (obj, "object", "object", 3.0)):
            shown = sample(points)
            q = shown - origin
            ax.scatter(q @ u, q @ v, s=size, c=colors[key], alpha=.75, linewidths=0,
                       label=f"{label} (n={len(points)})")
        ax.set_title(title); ax.set_xlabel("fixed plane u (m)"); ax.set_ylabel("fixed plane v (m)")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2); ax.legend(fontsize=8)

    combined(axes[0, 0], source_board, source_object, source_plane, "Source: board + central common object")
    combined(axes[0, 1], target_board, target_object, target_plane, "Target: preserved manual board + object")

    board_inlier_mask = np.zeros(len(target_board), dtype=bool)
    board_inlier_mask[np.asarray(target_plane_inliers, dtype=np.int64)] = True
    for ax, points, title, key in (
        (axes[0, 2], target_board, "Target raw manual board", "board"),
        (axes[1, 0], target_object, "Target raw manual object", "object"),
    ):
        x, y, _ = project(points, target_plane)
        ax.scatter(x, y, s=2, c=colors[key], alpha=.8, linewidths=0, label=f"preserved points (n={len(points)})")
        ax.set_title(title); ax.set_xlabel("fixed plane u (m)"); ax.set_ylabel("fixed plane v (m)")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2); ax.legend(fontsize=8)

    bx, by, _ = project(target_board, target_plane)
    axes[1, 1].scatter(bx[~board_inlier_mask], by[~board_inlier_mask], s=3, c=colors["outlier"],
                       alpha=.7, linewidths=0, label=f"RANSAC outliers (n={np.count_nonzero(~board_inlier_mask)})")
    axes[1, 1].scatter(bx[board_inlier_mask], by[board_inlier_mask], s=2, c=colors["plane"],
                       alpha=.75, linewidths=0, label=f"plane inliers (n={np.count_nonzero(board_inlier_mask)})")
    axes[1, 1].set_title("Target manual board: robust plane coverage")
    axes[1, 1].set_xlabel("fixed plane u (m)"); axes[1, 1].set_ylabel("fixed plane v (m)")
    axes[1, 1].set_aspect("equal", adjustable="box"); axes[1, 1].grid(alpha=.2); axes[1, 1].legend(fontsize=8)

    u, _ = plane_basis(np.asarray(target_plane[:3]))
    origin = np.mean(target_board, axis=0)
    for points, key, label, size in ((target_board, "board", "manual board", 2), (target_object, "object", "manual object", 5)):
        shown = sample(points)
        q = shown - origin
        signed = shown @ (target_plane[:3] / np.linalg.norm(target_plane[:3])) + target_plane[3] / np.linalg.norm(target_plane[:3])
        axes[1, 2].scatter(q @ u, signed, s=size, c=colors[key], alpha=.75, linewidths=0,
                           label=f"{label} (n={len(points)})")
    axes[1, 2].axhline(0, color=colors["plane"], linewidth=1, label="fitted board plane")
    axes[1, 2].set_title("Target side view: signed distance diagnostic")
    axes[1, 2].set_xlabel("fixed plane u (m)"); axes[1, 2].set_ylabel("signed distance to board plane (m)")
    axes[1, 2].grid(alpha=.2); axes[1, 2].legend(fontsize=8)

    fig.suptitle("Stage 2 manual target review — fixed plane-local camera; no registration", fontsize=15)
    fig.tight_layout(); path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight"); plt.close(fig)


def save_target_heightmap_preview(path: Path, *, points: np.ndarray, selection: dict[str, Any],
                                  seed: int = 42) -> None:
    """Six target-only diagnostics for the automatic plane-local heightmap selection."""
    diagnostics = selection["_diagnostics"]
    uv = np.asarray(diagnostics["uv"])
    signed = np.asarray(diagnostics["signed_depth"])
    ij = np.asarray(diagnostics["grid_indices"])
    grid = selection["grid_statistics"]
    origin = np.asarray(grid["grid_origin_uv_m"])
    cell = float(grid["grid_size_m"])
    requested = selection["requested_side"]
    display_side = selection.get("selected_side") or (requested if requested in {"positive", "negative"} else "negative")
    depth = signed if display_side == "positive" else -signed
    side_debug = diagnostics["side_debug"][display_side]
    strong = np.asarray(side_debug["_strong_mask"])
    grown = np.asarray(side_debug["_grown_mask"])
    labels = np.asarray(side_debug["_labels"])
    cell_depth = np.asarray(side_debug["_cell_depth"])
    selected = np.asarray(selection["selected_indices"], dtype=np.int64)
    rng = np.random.default_rng(seed)

    def sampled_indices(limit: int = 45000) -> np.ndarray:
        index = np.arange(len(points))
        return index if len(index) <= limit else np.sort(rng.choice(index, limit, replace=False))

    def cells_xy(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rc = np.argwhere(mask)
        xy = origin + (rc[:, [0, 1]] + .5) * cell if len(rc) else np.empty((0, 2))
        return xy[:, 0], xy[:, 1]

    fig, axes = plt.subplots(2, 3, figsize=(18, 11), squeeze=False)
    shown = sampled_indices()
    axes[0, 0].scatter(uv[shown, 0], uv[shown, 1], s=1, color="#7f8c9a", alpha=.5, linewidths=0)
    axes[0, 0].set_title(f"1. Target raw u-v projection (n={len(points)})")

    clip = max(float(selection["thresholds"]["high_threshold_m"]) * 3., np.percentile(np.abs(depth), 95))
    colored = axes[0, 1].scatter(uv[shown, 0], uv[shown, 1], c=np.clip(depth[shown], -clip, clip),
                                 s=2, cmap="coolwarm", vmin=-clip, vmax=clip, linewidths=0)
    fig.colorbar(colored, ax=axes[0, 1], label=f"{display_side} protrusion depth (m)")
    axes[0, 1].set_title("2. Plane-local protrusion depth")

    sx, sy = cells_xy(strong)
    axes[0, 2].scatter(uv[shown, 0], uv[shown, 1], s=.5, color="#c9cdd2", alpha=.25, linewidths=0)
    axes[0, 2].scatter(sx, sy, marker="s", s=20, color="#d62828", alpha=.85,
                       label=f"strong cells (n={len(sx)})")
    axes[0, 2].set_title("3. Strong-seed cells")
    axes[0, 2].legend(fontsize=8)

    gx, gy = cells_xy(grown)
    axes[1, 0].scatter(gx, gy, marker="s", s=14, color="#f4a261", alpha=.55,
                       label=f"low-threshold grown mask (n={len(gx)})")
    chosen_component = selection.get("selected_component")
    if chosen_component and chosen_component["side"] == display_side:
        chosen_mask = labels == int(chosen_component["label"])
        cx, cy = cells_xy(chosen_mask)
        axes[1, 0].scatter(cx, cy, marker="s", s=10, color="#2a9d8f", alpha=.9,
                           label=f"best component #{chosen_component['component_id']}")
    axes[1, 0].set_title("4. 8-neighbor low/high region growth")
    axes[1, 0].legend(fontsize=8)

    axes[1, 1].scatter(uv[shown, 0], uv[shown, 1], s=.5, color="#c9cdd2", alpha=.2, linewidths=0)
    if len(selected):
        axes[1, 1].scatter(uv[selected, 0], uv[selected, 1], s=3, color="#06d6a0", alpha=.8,
                           linewidths=0, label=f"original retained points (n={len(selected)})")
    else:
        axes[1, 1].text(.5, .5, f"NOT CONFIRMED\n{selection['selection_reason']}", transform=axes[1, 1].transAxes,
                        ha="center", va="center", color="#b00020", fontsize=11)
    axes[1, 1].set_title("5. Final retained original points")
    if len(selected): axes[1, 1].legend(fontsize=8)

    axes[1, 2].scatter(uv[shown, 0], uv[shown, 1], s=.4, color="#d7d9dc", alpha=.18, linewidths=0)
    rejected = [item for item in selection["candidate_components"] if not item["eligible"] and item["side"] == display_side]
    for number, candidate in enumerate(rejected):
        mask = labels == int(candidate["label"])
        rx, ry = cells_xy(mask)
        axes[1, 2].scatter(rx, ry, marker="s", s=10, alpha=.45, linewidths=0)
        center = candidate.get("projection_center_uv_m")
        if center is not None:
            reasons = ", ".join(candidate["rejection_reasons"][:2])
            axes[1, 2].annotate(f"#{candidate['component_id']}: {reasons}", xy=center, fontsize=6,
                                xytext=(3, 3), textcoords="offset points")
    if not rejected:
        axes[1, 2].text(.5, .5, "No rejected candidates on displayed side", transform=axes[1, 2].transAxes,
                        ha="center", va="center")
    axes[1, 2].set_title("6. Rejected components and reasons")

    for ax in axes.ravel():
        ax.set_xlabel("plane u (m)"); ax.set_ylabel("plane v (m)")
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2)
    fig.suptitle("Stage 2 FAST-LIVO2 target heightmap diagnostics — original coordinates; no registration", fontsize=15)
    fig.tight_layout(); path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight"); plt.close(fig)


def save_stage3_candidates(path: Path, candidates: list[dict[str, Any]], source_object: np.ndarray,
                           target_object: np.ndarray, title: str, max_panels: int = 12) -> None:
    """Fixed world-coordinate diagnostic views; only local array copies are transformed."""
    from .transforms import apply_transform
    shown = sorted(candidates, key=lambda c: (c.get("rank", 10**9), c.get("coarse_rank_score", 10**9), c["candidate_id"]))[:max_panels]
    n = max(1, len(shown)); cols = min(4, n); rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.2*cols, 4.6*rows), squeeze=False)
    rng = np.random.default_rng(42)
    target = target_object if len(target_object) <= 1800 else target_object[np.sort(rng.choice(len(target_object), 1800, replace=False))]
    source = source_object if len(source_object) <= 1800 else source_object[np.sort(rng.choice(len(source_object), 1800, replace=False))]
    for ax in axes.ravel(): ax.set_visible(False)
    for ax, candidate in zip(axes.ravel(), shown):
        ax.set_visible(True); moved = apply_transform(source, np.asarray(candidate["matrix_m"], dtype=float))
        ax.scatter(target[:, 1], target[:, 2], s=4, color="#277da1", alpha=.45, linewidths=0, label="FAST target object")
        ax.scatter(moved[:, 1], moved[:, 2], s=3, color="#f94144", alpha=.45, linewidths=0, label="transformed INSPIRE diagnostic copy")
        label=candidate["candidate_id"]
        if len(label)>24: label=label[:24]
        ax.set_title(f"{label}\n{candidate.get('generator','')} | coarse={candidate.get('coarse_rank_score',0):.4g}",fontsize=9)
        ax.set_xlabel("world Y (m)");ax.set_ylabel("world Z (m)");ax.set_aspect("equal",adjustable="box");ax.grid(alpha=.2)
    if shown: axes.ravel()[0].legend(fontsize=7,loc="best")
    fig.suptitle(f"Stage 4 coarse candidates — {title}\nFixed Y–Z camera; metre units; no ICP/refinement/final selection",fontsize=13)
    fig.tight_layout();path.parent.mkdir(parents=True,exist_ok=True);fig.savefig(path,dpi=170,bbox_inches="tight");plt.close(fig)


def save_shape_grid_preview(path: Path, source: dict[str, Any], target: dict[str, Any]) -> None:
    """Show observed support, outer contour, and height without painting unknown cells."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))
    for ax, name, grid in zip(axes, ("INSPIRE source", "FAST target"), (source, target)):
        centers = grid["centers"]
        scatter = ax.scatter(centers[:, 0], centers[:, 1], c=grid["height_p50_m"] * 1000,
                             s=35, marker="s", cmap="viridis", vmin=0, vmax=55,
                             alpha=np.clip(grid["confidence"], .15, 1), linewidths=0)
        contour = grid["contour"]
        ax.scatter(contour[:, 0], contour[:, 1], s=15, marker="o", facecolors="none",
                   edgecolors="#ef476f", linewidths=.8, label="supported outer contour")
        cropped = centers[grid["crop_cell"]]
        if len(cropped):
            ax.scatter(cropped[:, 0], cropped[:, 1], s=24, marker="x", color="#d00000", label="ROI edge downweighted")
        ax.set(title=f"{name}: {grid['summary']['observed_cells']} observed cells",
               xlabel="local board u (m)", ylabel="local board v (m)")
        ax.set_aspect("equal", adjustable="box");ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.colorbar(scatter, ax=axes, label="median distance from board (mm)", shrink=.75)
    fig.suptitle("Stage 3 shape grid: blank cells are unknown, color is observed cell height")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_stage4_comparison(path: Path, candidates: list[dict[str, Any]], source_board: np.ndarray,
                           source_object: np.ndarray, target_board: np.ndarray, target_object: np.ndarray,
                           seed: int = 42) -> dict[str, Any]:
    """Show formal parents beside accepted or diagnostic-only attempted refinements."""
    from .transforms import apply_transform
    from matplotlib.lines import Line2D
    ordered=sorted(candidates,key=lambda c:c["parent_candidate_id"])
    rng=np.random.default_rng(seed)
    sample=lambda x,n: x if len(x)<=n else x[np.sort(rng.choice(len(x),n,False))]
    so,tobj,sb,tb=sample(source_object,1200),sample(target_object,1200),sample(source_board,1200),sample(target_board,1200)
    n=np.linalg.svd(tb-np.mean(tb,axis=0),full_matrices=False)[2][-1];n/=np.linalg.norm(n);u,v=plane_basis(n);origin=np.mean(tb,axis=0)
    front=lambda x: np.c_[(x-origin)@u,(x-origin)@v]
    side=lambda x: np.c_[(x-origin)@u,(x-origin)@n]
    all_front=np.vstack([front(tobj)]+[front(apply_transform(so,np.asarray(c[k]))) for c in ordered for k in ("parent_matrix_m","visualization_matrix_m")])
    lo=np.min(all_front,axis=0);hi=np.max(all_front,axis=0);pad=max(.002,.03*np.max(hi-lo));rows=max(1,len(ordered))
    fig,axes=plt.subplots(rows,3,figsize=(14,3.0*rows),squeeze=False)
    for row,c in enumerate(ordered):
        transformed=[]
        status_label,status_color=_stage4_status_presentation(c["status"])
        panels=(("stage-4 parent object front",c["parent_matrix_m"],"#f9c74f"),
                ("actual refinement attempt object front",c["visualization_matrix_m"],status_color))
        for col,(label,matrix,color) in enumerate(panels):
            ax=axes[row,col];q=apply_transform(so,np.asarray(matrix));transformed.append(q);a,b=front(tobj),front(q)
            ax.scatter(a[:,0],a[:,1],s=3,color="#277da1",alpha=.45,linewidths=0,label="FAST object")
            ax.scatter(b[:,0],b[:,1],s=2,color=color,alpha=.50,linewidths=0,label=label)
            ax.set(xlim=(lo[0]-pad,hi[0]+pad),ylim=(lo[1]-pad,hi[1]+pad),xlabel="board u (m)",ylabel="board v (m)")
            ax.set_aspect("equal",adjustable="box");ax.grid(alpha=.2)
            ax.set_title(f"{c['parent_candidate_id']}\n{label}" if col==0 else _stage4_candidate_title(c),fontsize=8,color=status_color if col else "black")
        cp=np.mean(front(transformed[0]),axis=0);cf=np.mean(front(transformed[1]),axis=0)
        axes[row,1].annotate("",xy=cf,xytext=cp,arrowprops={"arrowstyle":"->","color":"#111111","lw":1.8})
        ax=axes[row,2];ts=side(tb);ps=side(apply_transform(sb,np.asarray(c["parent_matrix_m"])));fs=side(apply_transform(sb,np.asarray(c["visualization_matrix_m"])))
        ax.scatter(ts[:,0],ts[:,1],s=3,color="#277da1",alpha=.35,linewidths=0,label="FAST board")
        ax.scatter(ps[:,0],ps[:,1],s=2,color="#f9c74f",alpha=.30,linewidths=0,label="parent board")
        board_label="accepted refinement" if c["status"]=="refined_valid" else ("rejected diagnostic proposal" if c["status"]=="no_safe_refinement" else "failed diagnostic proposal")
        ax.scatter(fs[:,0],fs[:,1],s=2,color=status_color,alpha=.45,linewidths=0,label=board_label)
        ax.set_xlabel("board u (m)");ax.set_ylabel("board normal (m)");ax.grid(alpha=.2)
        ax.set_title(f"board side | {status_label}\nactual attempt: {_stage4_source_text(c)}",fontsize=8,color=status_color)
    legend=[Line2D([0],[0],marker="o",color="w",markerfacecolor="#277da1",label="FAST target",markersize=7),
        Line2D([0],[0],marker="o",color="w",markerfacecolor="#f9c74f",label="stage-4 parent",markersize=7),
        Line2D([0],[0],marker="o",color="w",markerfacecolor="#2ca02c",label="PASS — accepted refinement",markersize=7),
        Line2D([0],[0],marker="o",color="w",markerfacecolor="#f28e2b",label="REJECTED — diagnostic only",markersize=7),
        Line2D([0],[0],marker="o",color="w",markerfacecolor="#d62728",label="FAILED — diagnostic only",markersize=7)]
    fig.legend(handles=legend,loc="upper center",ncol=5,fontsize=8,bbox_to_anchor=(.5,.982))
    fig.suptitle("Stage 5 constrained refinement — parent versus actual attempted refinement\nRejected proposals are shown for diagnosis and are not exported transforms.",fontsize=13,y=.999)
    fig.tight_layout(rect=(0,0,1,.985),h_pad=1.5);path.parent.mkdir(parents=True,exist_ok=True);fig.savefig(path,dpi=150);plt.close(fig)
    return {"views":["object_plane_front_parent","object_plane_front_visualization_matrix","board_side_with_target_parent_and_actual_attempt"],
            "centroid_motion_arrow":True,"delta_and_rejection_annotation":True,"unit":"m","fixed_colors":{"FAST_target":"#277da1",
                "stage3_parent":"#f9c74f","accepted_refinement":"#2ca02c","rejected_diagnostic":"#f28e2b","failed_diagnostic":"#d62728"},
            "diagnostic_only_note":"Rejected proposals are shown for diagnosis and are not exported transforms.",
            "legend_labels":[x.get_label() for x in legend],"candidate_titles":[_stage4_candidate_title(c) for c in ordered],
            "displayed_parent_candidate_ids":[c["parent_candidate_id"] for c in ordered],"undisplayed_parent_candidate_ids":[]}


def _stage4_status_presentation(status: str) -> tuple[str, str]:
    if status=="refined_valid":return "PASS — accepted refinement","#2ca02c"
    if status=="no_safe_refinement":return "REJECTED — diagnostic only","#f28e2b"
    return "FAILED — diagnostic only","#d62728"


def _stage4_source_text(candidate: dict[str, Any]) -> str:
    algorithm=candidate.get("visualization_source_algorithm") or candidate.get("visualization_source_kind") or "unknown"
    level=candidate.get("visualization_source_level");iteration=candidate.get("visualization_source_iteration")
    suffix="" if level is None else f" L{level}";suffix+="" if iteration is None else f" iter {iteration}"
    return f"{algorithm}{suffix} [{candidate.get('visualization_source_kind','matrix')}]"


def _stage4_candidate_title(candidate: dict[str, Any]) -> str:
    status,_=_stage4_status_presentation(candidate["status"]);source=_stage4_source_text(candidate)
    reasons=candidate.get("visualization_rejection_reasons",[]);reason=reasons[0] if reasons else "none"
    if len(reason)>52:reason=reason[:49]+"..."
    return (f"{status} | formal={candidate['status']}\nsource={source} | "
        f"ΔR={candidate['visualization_from_parent_rotation_deg']:.3f}°  "
        f"Δt={1000*candidate['visualization_from_parent_translation_m']:.2f} mm\nreason={reason}")

