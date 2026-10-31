"""Fuse Cosmos detections with table depth into UF0 TCP targets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from calibration import geometry as calib
from screw_policy.config import ScrewTaskConfig
from screw_policy.geometry import (
    base_to_uf0_mm,
    cluster_screw_candidates,
    depth_points_table,
    scale_rgb_uv_to_native,
)


@dataclass
class ScrewTarget:
    x_mm: float
    y_mm: float
    yaw_deg: float
    confidence: float
    source: str


def _norm_to_table_base_m(nx: float, ny: float, image_wh: tuple[int, int]) -> np.ndarray:
    cams = calib.load()
    cam = cams["table"]
    u, v = scale_rgb_uv_to_native(
        nx * image_wh[0], ny * image_wh[1], tuple(cam["size"]), image_wh
    )
    return calib.pixel_to_base(cam, (u, v), z=calib.SURFACE_Z + 0.008)


def localize_from_obs(
    obs: dict,
    cfg: ScrewTaskConfig,
    cosmos_parsed: dict | None,
    *,
    image_wh: tuple[int, int],
) -> tuple[list[ScrewTarget], tuple[float, float] | None, dict]:
    """Return screw targets in UF0 mm, optional bin centre (x,y) mm, and perception stats."""
    screws: list[ScrewTarget] = []
    bin_xy = cfg.bin_xy_mm
    cosmos_count = 0
    depth_count = 0

    depth = (obs.get("depth") or {}).get("table")
    if depth is not None:
        pts, px = depth_points_table(depth, stride=6)
        for c in cluster_screw_candidates(pts, px):
            uf0 = base_to_uf0_mm(np.array([c["centre_base_m"]]))[0]
            screws.append(
                ScrewTarget(
                    x_mm=float(uf0[0]),
                    y_mm=float(uf0[1]),
                    yaw_deg=float(c["yaw_deg"]),
                    confidence=float(c["confidence"]),
                    source="depth_cluster",
                )
            )
            depth_count += 1

    if cosmos_parsed:
        for s in cosmos_parsed.get("screws") or []:
            cx, cy = s["center"]
            base = _norm_to_table_base_m(cx, cy, image_wh)
            uf0 = base_to_uf0_mm(base.reshape(1, 3))[0]
            screws.append(
                ScrewTarget(
                    x_mm=float(uf0[0]),
                    y_mm=float(uf0[1]),
                    yaw_deg=float(s.get("yaw_deg", 0.0)),
                    confidence=float(s.get("confidence", 0.5)),
                    source="cosmos",
                )
            )
            cosmos_count += 1
        b = cosmos_parsed.get("bin")
        if b and b.get("center"):
            bx, by = b["center"]
            bbase = _norm_to_table_base_m(bx, by, image_wh)
            buf0 = base_to_uf0_mm(bbase.reshape(1, 3))[0]
            bin_xy = (float(buf0[0]), float(buf0[1]))

    used_fallback = not screws
    if not screws:
        for x, y, yaw in cfg.fallback_screws_mm:
            screws.append(
                ScrewTarget(
                    x_mm=float(x),
                    y_mm=float(y),
                    yaw_deg=float(yaw),
                    confidence=0.25,
                    source="fallback",
                )
            )

    # De-duplicate nearby targets, keep highest confidence.
    merged: list[ScrewTarget] = []
    for t in sorted(screws, key=lambda s: -s.confidence):
        if any(
            (t.x_mm - m.x_mm) ** 2 + (t.y_mm - m.y_mm) ** 2 < 400.0 for m in merged
        ):
            continue
        merged.append(t)
    merged = [t for t in merged if t.confidence >= cfg.min_confidence * 0.5]
    source_counts: dict[str, int] = {}
    for t in merged:
        source_counts[t.source] = source_counts.get(t.source, 0) + 1
    stats = {
        "merged_target_count": len(merged),
        "raw_cosmos_count": cosmos_count,
        "raw_depth_cluster_count": depth_count,
        "used_fallback_grid": used_fallback,
        "source_counts": source_counts,
        "mean_confidence": (
            float(sum(t.confidence for t in merged) / len(merged)) if merged else 0.0
        ),
        "bin_xy_mm": list(bin_xy),
    }
    return merged, bin_xy, stats
