#!/usr/bin/env python3
"""Build and optionally save/render the dual-arm LEGO 77243 workcell."""

import argparse
from pathlib import Path

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--save", type=Path)
parser.add_argument("--render", type=Path)
args = parser.parse_args()

app = SimulationApp({"headless": args.headless})
try:
    import cv2
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    import lego_cell

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
    stage = omni.usd.get_context().get_stage()
    lego_cell.build(stage)
    world.reset()
    for _ in range(120):
        world.step(render=bool(args.render))
    if args.render:
        args.render.parent.mkdir(parents=True, exist_ok=True)
        camera = Camera(prim_path=lego_cell.OVERVIEW_CAM_PATH, resolution=(1920, 1080))
        camera.initialize()
        for _ in range(16):
            world.step(render=True)
        rgba = camera.get_rgba()
        if rgba is None:
            raise RuntimeError("LEGO overview camera returned no image")
        cv2.imwrite(str(args.render), cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR))
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        omni.usd.get_context().save_as_stage(str(args.save))
finally:
    app.close()

