"""Copy a finished run's inputs into a new run folder, optionally corrupting one grasp.

    .venv/bin/python make_fault_run.py runs/stack_01 runs/matthew_exp_yaw2 --fault 2:0:0:25

--fault PICK:DX_MM:DY_MM:DYAW_DEG simulates a perception error on one planned
grasp (PICK = position in sort_plan.json). Without --fault the plan is unchanged.
Only experiment inputs are copied; the source run is never modified.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

from recovery_policy import with_grasp

INPUTS = ("camera.json", "world_camera.png", "scene_state.json", "ground_truth.json", "detections.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source")
    ap.add_argument("out")
    ap.add_argument("--fault", metavar="PICK:DX_MM:DY_MM:DYAW_DEG")
    a = ap.parse_args()
    src, out = Path(a.source), Path(a.out)
    if not (src / "sort_plan.json").is_file():
        sys.exit(f"{src}/sort_plan.json missing")
    if out.exists():
        sys.exit(f"{out} already exists")
    out.mkdir(parents=True)
    for name in INPUTS:
        if (src / name).is_file():
            shutil.copy(src / name, out / name)
    plan = json.loads((src / "sort_plan.json").read_text())
    plan.pop("recovery_history", None)
    plan.pop("recovery_rungs_used", None)
    if a.fault:
        k, dx, dy, dyaw = a.fault.split(":")
        k = int(k)
        p = plan["picks"][k]
        plan["picks"][k] = with_grasp(p, [p["xy"][0] + float(dx) / 1000, p["xy"][1] + float(dy) / 1000],
                                      p["yaw_deg"] + float(dyaw))
        plan["fake_fault"] = a.fault
    (out / "sort_plan.json").write_text(json.dumps(plan, indent=2))
    print(f"made {out}" + (f" (fault {a.fault})" if a.fault else " (no fault)"))


if __name__ == "__main__":
    main()
