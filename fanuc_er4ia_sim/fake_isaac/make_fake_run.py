"""Build a synthetic camera/planner run for fake-Isaac recovery tests.

Run from fanuc_er4ia_sim:
  python fake_isaac/make_fake_run.py runs/fake_01
  python fake_isaac/make_fake_run.py runs/fake_fault --fault 2:2:0:0

The optional fault is PICK:DX_MM:DY_MM:DYAW_DEG and offsets one planned grasp
to exercise image-based remeasurement. It does not create data from a real robot
or Isaac physics.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))

import scene_config as C  # noqa: E402
from plan_sort import build_plan  # noqa: E402
from recovery_policy import with_grasp  # noqa: E402

if __package__:
    from .fakes import TABLE_MODEL, render_table, yaw_quat
else:
    from fakes import TABLE_MODEL, render_table, yaw_quat


def detection(cam, xy, colour, yaw):
    theta = np.deg2rad(yaw)
    rotation = np.array(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
    )
    endpoints = [
        (
            cam.project(
                [
                    *(np.array(xy) + rotation @ np.array([x, -C.CUBE_EDGE / 2])),
                    cam.table_z + C.CUBE_EDGE,
                ]
            )
            / [cam.W, cam.H]
        ).tolist()
        for x in (-C.CUBE_EDGE / 2, C.CUBE_EDGE / 2)
    ]
    centre = cam.project([*xy, cam.table_z + C.CUBE_EDGE / 2]) / [
        cam.W,
        cam.H,
    ]
    return {
        "category": "cube",
        "colour": colour,
        "top_edge_endpoints": endpoints,
        "pose_confidence": 0.95,
        "center": centre.tolist(),
    }


def make_run(out, fault=None):
    out = Path(out)
    if out.exists() and any(out.iterdir()):
        raise ValueError(f"{out} is not empty")
    out.mkdir(parents=True, exist_ok=True)

    cubes = [
        (x, y, colour, yaw)
        for (x, y, colour), yaw in zip(C.SORT_CUBES, C.SORT_CUBE_YAWS_DEG)
    ]
    objects = {
        f"SortCube{index}": {
            "pos": np.array([x, y, C.TABLE_TOP_Z + C.SURFACE_DZ]),
            "yaw": yaw,
            "colour": colour,
        }
        for index, (x, y, colour, yaw) in enumerate(cubes)
    }
    image = render_table(objects)
    if not cv2.imwrite(str(out / "world_camera.png"), image):
        raise OSError(f"Could not write synthetic image to {out}")
    (out / "camera.json").write_text(
        json.dumps(TABLE_MODEL.j, indent=2) + "\n", encoding="utf-8"
    )
    scene_state = {
        "objects": {
            name: {
                "position": obj["pos"].tolist(),
                "orientation": yaw_quat(obj["yaw"]).tolist(),
            }
            for name, obj in objects.items()
        }
    }
    (out / "scene_state.json").write_text(
        json.dumps(scene_state, indent=2) + "\n", encoding="utf-8"
    )
    ground_truth = {
        "objects": [
            {"name": name, "category": "cube", "colour": obj["colour"]}
            for name, obj in objects.items()
        ]
    }
    (out / "ground_truth.json").write_text(
        json.dumps(ground_truth, indent=2) + "\n", encoding="utf-8"
    )
    detections = [
        detection(TABLE_MODEL, (x, y), colour, yaw)
        for x, y, colour, yaw in cubes
    ]
    plan = build_plan(
        {"objects": detections, "coordinate_convention": "unit"},
        TABLE_MODEL,
        image,
    )
    if len(plan["picks"]) != 6:
        raise RuntimeError(f"Planner produced {len(plan['picks'])} picks")
    if fault:
        index, dx_mm, dy_mm, dyaw_deg = fault
        pick = plan["picks"][index]
        xy = [
            pick["xy"][0] + dx_mm / 1000,
            pick["xy"][1] + dy_mm / 1000,
        ]
        plan["picks"][index] = with_grasp(
            pick, xy, pick["yaw_deg"] + dyaw_deg
        )
    (out / "sort_plan.json").write_text(
        json.dumps(plan, indent=2) + "\n", encoding="utf-8"
    )
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--fault", metavar="PICK:DX_MM:DY_MM:DYAW_DEG")
    args = parser.parse_args(argv)
    fault = None
    if args.fault:
        try:
            index, dx, dy, yaw = args.fault.split(":")
            fault = (int(index), float(dx), float(dy), float(yaw))
        except ValueError as exc:
            parser.error(f"invalid --fault value: {exc}")
    run = make_run(args.out, fault)
    print(f"Created synthetic run {run}")


if __name__ == "__main__":
    main()
