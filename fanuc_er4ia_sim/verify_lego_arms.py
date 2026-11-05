#!/usr/bin/env python3
"""Isaac gate G2: prove two ER-4iA articulations and jaws are independent."""

import argparse
import json
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, default=Path("runs/lego_arm_gate.json"))
args = parser.parse_args()

app = SimulationApp({"headless": True})
report = {"gate": "G2", "passed": False, "arms": []}
try:
    import omni.usd
    from pxr import UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    import cell
    import lego_cell
    import lego_demo_config as D

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
    stage = omni.usd.get_context().get_stage()
    lego_cell.build(stage)
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith("/World/LegoParts/") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
    cell.enable_ccd(world)
    world.reset()

    arms = []
    for spec in D.ARM_SPECS:
        arm = SingleArticulation(prim_path=spec["prim_path"], name=spec["name"])
        arm.initialize()
        names = list(arm.dof_names)
        arms.append(
            {
                "spec": spec,
                "arm": arm,
                "names": names,
                "arm_idx": [names.index(f"J{i}") for i in range(1, 7)],
                "jaw_idx": [names.index("jaw_l"), names.index("jaw_r")],
            }
        )

    targets_deg = (
        np.array([-12.0, 10.0, -8.0, 6.0, -10.0, 8.0]),
        np.array([12.0, 8.0, -6.0, -5.0, -8.0, -7.0]),
    )

    def command(entry, q, jaw):
        positions = entry["arm"].get_joint_positions().copy()
        positions[entry["arm_idx"]] = q
        for name, value in cell.jaw_targets(jaw).items():
            positions[entry["names"].index(name)] = value
        entry["arm"].apply_action(ArticulationAction(joint_positions=positions))

    for _ in range(180):
        command(arms[0], np.deg2rad(targets_deg[0]), D.PLATE_HEIGHT / 2)
        command(arms[1], np.deg2rad(targets_deg[1]), cell.C.JAW_OPEN)
        world.step(render=False)

    first_measurements = []
    for entry, target in zip(arms, targets_deg):
        measured = entry["arm"].get_joint_positions()
        first_measurements.append(measured.copy())
        arm_error = np.rad2deg(measured[entry["arm_idx"]]) - target
        first_measurements[-1] = measured.copy()
        report["arms"].append(
            {
                "name": entry["spec"]["name"],
                "prim_path": entry["spec"]["prim_path"],
                "joint_names": entry["names"],
                "target_deg": target.tolist(),
                "measured_deg": np.rad2deg(measured[entry["arm_idx"]]).tolist(),
                "max_joint_error_deg": float(np.max(np.abs(arm_error))),
                "phase1_jaws_m": measured[entry["jaw_idx"]].tolist(),
            }
        )

    # Swap jaw ownership without changing either arm pose.
    for _ in range(120):
        command(arms[0], np.deg2rad(targets_deg[0]), cell.C.JAW_OPEN)
        command(arms[1], np.deg2rad(targets_deg[1]), D.PLATE_HEIGHT / 2)
        world.step(render=False)

    for entry, row in zip(arms, report["arms"]):
        measured = entry["arm"].get_joint_positions()
        row["phase2_jaws_m"] = measured[entry["jaw_idx"]].tolist()
    report["independent_prim_paths"] = len({a["spec"]["prim_path"] for a in arms}) == 2
    report["all_six_joints_moved"] = all(
        np.all(np.abs(np.asarray(row["measured_deg"])) > 1.0) for row in report["arms"]
    )
    report["max_joint_error_deg"] = max(row["max_joint_error_deg"] for row in report["arms"])
    report["jaw_swap_observed"] = (
        abs(report["arms"][0]["phase1_jaws_m"][0]) > abs(report["arms"][0]["phase2_jaws_m"][0])
        and abs(report["arms"][1]["phase2_jaws_m"][0]) > abs(report["arms"][1]["phase1_jaws_m"][0])
    )
    report["passed"] = bool(
        report["independent_prim_paths"]
        and report["all_six_joints_moved"]
        and report["max_joint_error_deg"] < 1.0
        and report["jaw_swap_observed"]
    )
finally:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    app.close()

if not report["passed"]:
    raise SystemExit("dual-arm gate G2 failed")

