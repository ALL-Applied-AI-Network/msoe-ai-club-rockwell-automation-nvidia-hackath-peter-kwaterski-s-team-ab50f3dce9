"""Validate absolute TCP targets against the public cell envelope."""

from __future__ import annotations

import numpy as np

from screw_policy.config import ScrewTaskConfig


def clip_gripper(g: float) -> float:
    return float(np.clip(g, 0.0, 1.0))


def validate_tcp_row(row, cfg: ScrewTaskConfig) -> np.ndarray:
    a = np.asarray(row, dtype=float).reshape(7)
    if not np.all(np.isfinite(a)):
        raise ValueError("tcp row contains NaN or inf")
    x, y, z = a[0], a[1], a[2]
    if not (cfg.tcp_x[0] <= x <= cfg.tcp_x[1]):
        raise ValueError(f"tcp x={x} outside [{cfg.tcp_x[0]}, {cfg.tcp_x[1]}]")
    if not (cfg.tcp_y[0] <= y <= cfg.tcp_y[1]):
        raise ValueError(f"tcp y={y} outside [{cfg.tcp_y[0]}, {cfg.tcp_y[1]}]")
    if z < cfg.table_z_floor_mm:
        raise ValueError(f"tcp z={z} below table-safe floor {cfg.table_z_floor_mm}")
    if not (cfg.tcp_z[0] <= z <= cfg.tcp_z[1]):
        raise ValueError(f"tcp z={z} outside [{cfg.tcp_z[0]}, {cfg.tcp_z[1]}]")
    if np.abs(a[3:6]).max() > 360:
        raise ValueError("tcp w,p,r outside +-360 deg")
    out = a.copy()
    out[6] = clip_gripper(out[6])
    return out


def validate_chunk(rows, cfg: ScrewTaskConfig) -> np.ndarray:
    a = np.asarray(rows, dtype=float)
    if a.ndim == 1:
        a = a.reshape(1, 7)
    if a.shape[1] != 7:
        raise ValueError(f"expected (N,7) tcp chunk, got {a.shape}")
    if not (1 <= len(a) <= 100):
        raise ValueError("chunk length must be 1..100")
    return np.stack([validate_tcp_row(r, cfg) for r in a], axis=0)
