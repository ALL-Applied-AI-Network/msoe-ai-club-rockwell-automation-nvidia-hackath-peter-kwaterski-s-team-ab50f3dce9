"""Make a run folder like capture -> perceive -> plan would, without Isaac or Cosmos.

    .venv/bin/python fake_isaac/make_fake_run.py runs/fake_01
    .venv/bin/python fake_isaac/make_fake_run.py runs/fake_02 --fault 2:0:0:25

Uses the real cube layout from scene_config.py, renders the table camera image,
and builds sort_plan.json with the real plan_sort.py (image pose fit included).
--fault PICK:DX_MM:DY_MM:DYAW_DEG then corrupts one planned grasp, simulating a
perception error that slipped through (PICK = position in sort_plan.json).
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))

import scene_config as C  # noqa: E402
from fakes import TABLE_MODEL, render_table, yaw_quat  # noqa: E402
from plan_sort import build_plan  # noqa: E402
from recovery_policy import with_grasp  # noqa: E402


def detection(cam, xy, colour, yaw):
    """What Cosmos + the image front end report for one cube (as in the repo's tests)."""
    t = np.deg2rad(yaw)
    r = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    ends = [(cam.project([*(np.array(xy) + r @ np.array([x, -C.CUBE_EDGE / 2])),
                          cam.table_z + C.CUBE_EDGE]) / [cam.W, cam.H]).tolist()
            for x in (-C.CUBE_EDGE / 2, C.CUBE_EDGE / 2)]
    return dict(category="cube", colour=colour, top_edge_endpoints=ends, pose_confidence=0.95,
                center=(cam.project([*xy, cam.table_z + C.CUBE_EDGE / 2]) / [cam.W, cam.H]).tolist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--fault", metavar="PICK:DX_MM:DY_MM:DYAW_DEG")
    a = ap.parse_args()
    out = Path(a.out)
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty")
    out.mkdir(parents=True, exist_ok=True)

    cubes = [(x, y, colour, yaw) for (x, y, colour), yaw in zip(C.SORT_CUBES, C.SORT_CUBE_YAWS_DEG)]
    objects = {f"SortCube{i}": dict(pos=np.array([x, y, C.TABLE_TOP_Z + C.SURFACE_DZ]), yaw=yaw, colour=c)
               for i, (x, y, c, yaw) in enumerate(cubes)}
    img = render_table(objects)
    cv2.imwrite(str(out / "world_camera.png"), img)
    (out / "camera.json").write_text(json.dumps(TABLE_MODEL.j, indent=2))
    (out / "scene_state.json").write_text(json.dumps({"objects": {
        n: {"position": o["pos"].tolist(), "orientation": yaw_quat(o["yaw"]).tolist()}
        for n, o in objects.items()}}, indent=2))
    (out / "ground_truth.json").write_text(json.dumps({"objects": [
        {"name": n, "category": "cube", "colour": o["colour"]} for n, o in objects.items()]}, indent=2))

    dets = [detection(TABLE_MODEL, (x, y), c, yaw) for x, y, c, yaw in cubes]
    plan = build_plan({"objects": dets, "coordinate_convention": "unit"}, TABLE_MODEL, img)
    if len(plan["picks"]) != 6:
        sys.exit(f"planner produced {len(plan['picks'])} picks: {plan.get('skipped')}")
    if a.fault:
        k, dx, dy, dyaw = a.fault.split(":")
        k = int(k)
        p = plan["picks"][k]
        plan["picks"][k] = with_grasp(
            p, [p["xy"][0] + float(dx) / 1000, p["xy"][1] + float(dy) / 1000],
            p["yaw_deg"] + float(dyaw))
        plan["fake_fault"] = a.fault
    (out / "sort_plan.json").write_text(json.dumps(plan, indent=2))
    print(f"made {out}" + (f" with fault {a.fault}" if a.fault else ""))


if __name__ == "__main__":
    main()
