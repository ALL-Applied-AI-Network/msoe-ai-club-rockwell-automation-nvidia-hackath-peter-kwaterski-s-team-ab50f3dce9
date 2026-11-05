"""RGB/depth fusion helpers (organizer calibration only)."""

from __future__ import annotations

import numpy as np

from calibration import geometry as calib


def base_to_uf0_mm(xyz_base_m: np.ndarray) -> np.ndarray:
    """(N,3) base metres -> UF0 mm (x,y,z)."""
    pts = np.asarray(xyz_base_m, float).reshape(-1, 3)
    out = np.empty_like(pts)
    out[:, 0] = pts[:, 0] * 1000.0
    out[:, 1] = pts[:, 1] * 1000.0
    out[:, 2] = pts[:, 2] * 1000.0 + calib.UF0_Z_MM
    return out


def scale_rgb_uv_to_native(
    u: float, v: float, native_wh: tuple[int, int], resized_wh: tuple[int, int]
) -> tuple[float, float]:
    """Map pixel from policy IMAGE_SIZE back to native calibrated colour size."""
    nw, nh = native_wh
    rw, rh = resized_wh
    return u * nw / rw, v * nh / rh


def depth_points_table(
    depth_mm: np.ndarray, stride: int = 4
) -> tuple[np.ndarray, np.ndarray]:
    """Table depth -> base points (m) and native depth pixels (u,v)."""
    cams = calib.load()
    cam = cams["table_depth"]
    pts, px = calib.depth_to_base(cam, depth_mm, stride=stride, with_pixels=True)
    return pts, px


def project_base_to_table_rgb(
    xyz_base_m: np.ndarray, native: bool = True
) -> np.ndarray:
    cams = calib.load()
    cam = cams["table"]
    uv = calib.project(cam, xyz_base_m)
    return uv


def cluster_screw_candidates(
    pts_base: np.ndarray,
    px_depth: np.ndarray,
    table_z: float = calib.SURFACE_Z,
    z_band_m: float = 0.025,
) -> list[dict]:
    """Simple height filter + 2D grid clustering for screw-sized blobs."""
    if len(pts_base) == 0:
        return []
    pts = np.asarray(pts_base, float)
    px = np.asarray(px_depth, float)
    z_ok = np.abs(pts[:, 2] - (table_z + 0.008)) < z_band_m
    pts, px = pts[z_ok], px[z_ok]
    if len(pts) == 0:
        return []
    xy = pts[:, :2]
    cell = 0.015
    keys = np.floor(xy / cell).astype(int)
    buckets: dict[tuple[int, int], list[int]] = {}
    for i, k in enumerate(map(tuple, keys)):
        buckets.setdefault(k, []).append(i)
    out = []
    for idxs in buckets.values():
        if len(idxs) < 8:
            continue
        cluster_pts = pts[idxs]
        centre = cluster_pts.mean(axis=0)
        dxy = cluster_pts[:, :2] - centre[:2]
        if len(dxy) >= 3:
            cov = dxy.T @ dxy / len(dxy)
            _, vecs = np.linalg.eigh(cov)
            yaw = float(np.degrees(np.arctan2(vecs[1, -1], vecs[0, -1])))
        else:
            yaw = 0.0
        out.append(
            {
                "centre_base_m": centre.tolist(),
                "yaw_deg": yaw,
                "confidence": min(1.0, len(idxs) / 40.0),
                "depth_pixels": px[idxs].mean(axis=0).tolist(),
            }
        )
    return out
