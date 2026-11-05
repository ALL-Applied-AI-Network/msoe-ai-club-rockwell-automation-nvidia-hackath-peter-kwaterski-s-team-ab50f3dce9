"""Optional grasp faults to exercise stepwise live missed-grasp recovery."""

from __future__ import annotations

from typing import Any, Optional, Tuple

from recovery_policy import with_grasp

# pick_index, mode ("offset" | "drop"), dx_mm, dy_mm, dyaw_deg
FaultSpec = Tuple[int, str, float, float, float]
INJECT_ACCIDENTAL_DROP = "_inject_accidental_drop"


def parse_inject_missed_grasp(text: str) -> FaultSpec:
    """Parse ``PICK_INDEX:drop`` or ``PICK_INDEX:DX_MM:DY_MM:DYAW_DEG``."""
    try:
        parts = text.split(":")
        pick_index = int(parts[0])
        if pick_index < 0:
            raise ValueError("pick index must be nonnegative")
        if len(parts) == 2 and parts[1].lower() == "drop":
            return (pick_index, "drop", 0.0, 0.0, 0.0)
        if len(parts) == 4:
            return (
                pick_index,
                "offset",
                float(parts[1]),
                float(parts[2]),
                float(parts[3]),
            )
    except ValueError as exc:
        raise ValueError(
            f"invalid inject value {text!r}; use PICK_INDEX:drop or "
            "PICK_INDEX:DX_MM:DY_MM:DYAW_DEG"
        ) from exc
    raise ValueError(
        f"invalid inject value {text!r}; use PICK_INDEX:drop or "
        "PICK_INDEX:DX_MM:DY_MM:DYAW_DEG"
    )


def maybe_biased_pick(
    pick: dict,
    pick_index: int,
    fault: Optional[FaultSpec],
    *,
    retry_count: int,
) -> dict:
    """Apply a one-shot test fault before live recovery retries the same pick."""
    if fault is None or retry_count != 0:
        return pick
    fault_index, mode, dx_mm, dy_mm, dyaw_deg = fault
    if pick_index != fault_index:
        return pick
    if mode == "drop":
        updated = dict(pick)
        updated[INJECT_ACCIDENTAL_DROP] = True
        return updated
    xy = [
        pick["xy"][0] + dx_mm / 1000.0,
        pick["xy"][1] + dy_mm / 1000.0,
    ]
    return with_grasp(pick, xy, pick["yaw_deg"] + dyaw_deg)


def inject_fault_label(fault: FaultSpec) -> str:
    index, mode, dx_mm, dy_mm, dyaw_deg = fault
    if mode == "drop":
        return f"accidental drop on pick {index}"
    return f"grasp offset on pick {index} ({dx_mm:g},{dy_mm:g},{dyaw_deg:g} mm/deg)"


def fault_to_dict(fault: FaultSpec) -> dict:
    index, mode, dx_mm, dy_mm, dyaw_deg = fault
    payload: dict[str, Any] = {
        "pick_index": index,
        "mode": mode,
        "formats": ["PICK_INDEX:drop", "PICK_INDEX:DX_MM:DY_MM:DYAW_DEG"],
    }
    if mode == "offset":
        payload.update(dx_mm=dx_mm, dy_mm=dy_mm, dyaw_deg=dyaw_deg)
    return payload
