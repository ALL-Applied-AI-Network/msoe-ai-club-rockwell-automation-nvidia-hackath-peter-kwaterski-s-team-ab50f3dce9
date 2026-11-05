"""Write Isaac table-camera calibration JSON matching a rendered snapshot."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Sequence, Tuple

import scene_config as C


def write_isaac_camera_json(
    stage,
    cam_path: str,
    resolution: Sequence[int],
    out_path: Path,
    *,
    camera_name: str = "table",
) -> None:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(cam_path)
    gcam = UsdGeom.Camera(prim)
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    width, height = int(resolution[0]), int(resolution[1])
    payload = {
        "camera": camera_name,
        "resolution": [width, height],
        "focal_length_mm": float(gcam.GetFocalLengthAttr().Get()),
        "horizontal_aperture_mm": float(gcam.GetHorizontalApertureAttr().Get()),
        "vertical_aperture_mm": float(gcam.GetVerticalApertureAttr().Get()),
        "camera_to_world": [[float(xf[r][c]) for c in range(4)] for r in range(4)],
        "table_top_z": float(C.TABLE_TOP_Z + C.SURFACE_DZ),
        "robot_base_xy": [0.0, 0.0],
    }
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def table_camera_resolution(task: str) -> Tuple[int, int]:
    """Match capture_scene.py table resolution for the active task."""
    width, height = C.TABLE_CAM["resolution"]
    if task == "cube_stack":
        return (2 * width, 2 * height)
    return (width, height)
