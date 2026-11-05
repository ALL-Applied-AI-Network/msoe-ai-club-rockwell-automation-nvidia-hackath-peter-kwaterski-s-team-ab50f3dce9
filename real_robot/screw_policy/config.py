"""Versioned task profile for eight-screw default scene (UF0 millimetres)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_PROFILE = HERE.parent / "config" / "screw_task.yaml"


@dataclass(frozen=True)
class ScrewTaskConfig:
    expected_screws: int = 8
    max_retries_per_screw: int = 2
    action_dt: float = 0.15
    hover_z_mm: float = -280.0
    approach_z_mm: float = -310.0
    grasp_z_mm: float = -332.0
    lift_z_mm: float = -280.0
    bin_hover_z_mm: float = -260.0
    bin_drop_z_mm: float = -300.0
    safe_hover: tuple[float, float, float, float, float, float] = (
        400.0,
        0.0,
        -260.0,
        -180.0,
        0.0,
        0.0,
    )
    bin_xy_mm: tuple[float, float] = (510.0, 110.0)
    min_confidence: float = 0.35
    tcp_x: tuple[float, float] = (150.0, 650.0)
    tcp_y: tuple[float, float] = (-450.0, 450.0)
    tcp_z: tuple[float, float] = (-334.0, 300.0)
    table_z_floor_mm: float = -334.0
    # Fallback grid when Cosmos/heuristic mock has no detections (UF0 mm).
    fallback_screws_mm: tuple[tuple[float, float, float], ...] = (
        (380.0, -120.0, 0.0),
        (400.0, -80.0, 45.0),
        (420.0, -40.0, 90.0),
        (440.0, 0.0, 135.0),
        (420.0, 40.0, 180.0),
        (400.0, 80.0, -135.0),
        (380.0, 40.0, -90.0),
        (360.0, 0.0, -45.0),
    )


def load_config(path: Path | None = None) -> ScrewTaskConfig:
    path = path or DEFAULT_PROFILE
    if not path.is_file():
        return ScrewTaskConfig()
    raw = yaml.safe_load(path.read_text()) or {}
    safe = raw.get("safe_hover", {})
    fb = raw.get("fallback_screws_mm", [])
    return ScrewTaskConfig(
        expected_screws=int(raw.get("expected_screws", 8)),
        max_retries_per_screw=int(raw.get("max_retries_per_screw", 2)),
        action_dt=float(raw.get("action_dt", 0.15)),
        hover_z_mm=float(raw.get("hover_z_mm", -280.0)),
        approach_z_mm=float(raw.get("approach_z_mm", -310.0)),
        grasp_z_mm=float(raw.get("grasp_z_mm", -332.0)),
        lift_z_mm=float(raw.get("lift_z_mm", -280.0)),
        bin_hover_z_mm=float(raw.get("bin_hover_z_mm", -260.0)),
        bin_drop_z_mm=float(raw.get("bin_drop_z_mm", -300.0)),
        safe_hover=(
            float(safe.get("x", 400.0)),
            float(safe.get("y", 0.0)),
            float(safe.get("z", -260.0)),
            float(safe.get("w", -180.0)),
            float(safe.get("p", 0.0)),
            float(safe.get("r", 0.0)),
        ),
        bin_xy_mm=(
            float(raw.get("bin_x_mm", 510.0)),
            float(raw.get("bin_y_mm", 110.0)),
        ),
        min_confidence=float(raw.get("min_confidence", 0.35)),
        fallback_screws_mm=tuple(tuple(row) for row in fb)
        if fb
        else ScrewTaskConfig().fallback_screws_mm,
    )
