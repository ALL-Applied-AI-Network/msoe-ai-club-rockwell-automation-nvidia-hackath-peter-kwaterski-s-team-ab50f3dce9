"""Execute the locked LEGO 77243 slice with two physical ER-4iA arms.

The keyed feeders provide known initial poses. Parts are never teleported
during execution. Rev C closes on both physical pads before its carry-retention
constraint can engage, and that constraint is removed before connector lock.
A connector may lock only after strict pose, stud/socket, contact-duration and
low-speed gates pass. The second arm physically holds the fixture handle.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np

import fk
import ik
import lego_demo_config as D
import scene_config as C
from lego_plan import load_plan, run_manifest
from lego_insertion import DEFAULT_INSERTION_CONFIG
from lego_operation_profiles import (
    body_target_offset as profile_body_target_offset,
    connector_position_tolerance as profile_connector_position_tolerance,
    handling_profile,
)

# Deterministic elastic/contact compensation, measured in the simulator and
# applied to the robot target before contact.  It is logged per event and the
# connector gate still evaluates the unmodified rigid-body pose.
TRACKING_POSITION_TOLERANCE_M = 0.0025
TRACKING_ANGLE_TOLERANCE_DEG = 1.0


def _ease(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def _rpy_matrix(rpy_deg):
    rx, ry, rz = np.deg2rad(rpy_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        float,
    )


def _quat_matrix(quat):
    w, x, y, z = np.asarray(quat, float)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _matrix_quat(rotation):
    """Return an Isaac/Gf scalar-first quaternion from a rotation matrix."""
    r = np.asarray(rotation, float)
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
        elif i == 1:
            s = math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
        else:
            s = math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


def _rotation_error_deg(actual, desired):
    relative = np.asarray(desired) @ np.asarray(actual).T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _interpolate_rotation(start, finish, fraction):
    delta = np.asarray(finish) @ np.asarray(start).T
    angle = math.acos(float(np.clip((np.trace(delta) - 1) / 2, -1, 1)))
    if angle < 1e-9:
        return np.asarray(start)
    if abs(math.sin(angle)) < 1e-6:
        # The axis extracted from a rotation matrix is singular at pi. Use
        # quaternion SLERP only for that branch; retain the calibrated matrix
        # interpolation path for ordinary motions.
        q0, q1 = _matrix_quat(start), _matrix_quat(finish)
        dot = float(np.dot(q0, q1))
        if dot < 0:
            q1, dot = -q1, -dot
        theta = math.acos(float(np.clip(dot, -1, 1)))
        if theta < 1e-9:
            return np.asarray(start)
        q = (
            math.sin((1 - fraction) * theta) * q0
            + math.sin(fraction * theta) * q1
        ) / math.sin(theta)
        return _quat_matrix(q)
    axis = np.array([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
    axis /= 2 * math.sin(angle)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    theta = angle * fraction
    increment = np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * (k @ k)
    return increment @ start


def _atomic_json(path: Path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--physics-only", action="store_true")
    parser.add_argument("--max-tasks", type=int, default=None, help="development gate; omitted for an accountable full run")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    if args.fps != 30:
        parser.error("30 fps is required: two 1/60 s physics steps per video frame")

    run = Path(args.run).resolve()
    if run.exists() and any(run.iterdir()):
        raise SystemExit(f"run directory is not empty: {run}")
    run.mkdir(parents=True, exist_ok=True)
    frames = run / "frames"
    side_frames = run / "side_frames"
    if not args.physics_only:
        frames.mkdir()
        side_frames.mkdir()

    plan = load_plan()
    tasks = plan["tasks"][: args.max_tasks]
    try:
        git_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parent, text=True
        ).strip()
    except Exception:
        git_hash = "unknown"
    manifest = run_manifest(plan, git_hash=git_hash)
    manifest.update(executed_task_count=len(tasks), requested_task_count=len(plan["tasks"]))
    _atomic_json(run / "run_manifest.json", manifest)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    report = {
        "task": plan["name"],
        "task_complete": False,
        "verified_pickups": 0,
        "verified_placements": 0,
        "connector_locks": 0,
        "ik_failures": 0,
        "teleports_during_execution": 0,
        "tracking_gate_misses": [],
        "arm_roles": {spec["name"]: spec["role"] for spec in D.ARM_SPECS},
        "events": [],
        "fixture_transitions": [],
    }
    try:
        import cv2
        import omni.usd
        from pxr import (
            Gf,
            PhysicsSchemaTools,
            PhysxSchema,
            Sdf,
            UsdGeom,
            UsdPhysics,
        )
        from omni.physx import get_physx_simulation_interface
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera

        import cell
        import lego_cell
        from tweezer_plunger import (
            CLEARANCE_SLIDE_STROKE_M,
            PLUNGER_FACE_RETRACTED_FROM_TIP_M,
            PLUNGER_MAX_FORCE_N,
            PLUNGER_STROKE_M,
        )

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        stage = omni.usd.get_context().get_stage()
        lego_cell.build(stage)
        cell.enable_ccd(world)

        tool_contact_records = []
        assembly_contact_records = []
        physics_step = 0
        worker_tool_contact_paths = (
            D.ARM_SPECS[0]["prim_path"] + "/JawL",
            D.ARM_SPECS[0]["prim_path"] + "/JawLClearance",
            D.ARM_SPECS[0]["prim_path"] + "/JawR",
            D.ARM_SPECS[0]["prim_path"] + "/Plunger",
        )
        support_tool_contact_paths = (
            D.ARM_SPECS[1]["prim_path"] + "/J6_link",
            D.ARM_SPECS[1]["prim_path"] + "/JawL",
            D.ARM_SPECS[1]["prim_path"] + "/JawR",
        )
        tool_contact_paths = worker_tool_contact_paths + support_tool_contact_paths
        for tool_contact_path in tool_contact_paths:
            contact_api = PhysxSchema.PhysxContactReportAPI.Apply(
                stage.GetPrimAtPath(tool_contact_path)
            )
            contact_api.CreateThresholdAttr().Set(0.0)
        part_contact_paths = tuple(
            "/World/LegoParts/" + part["id"] for part in plan["parts"]
        )
        for part_contact_path in part_contact_paths:
            contact_api = PhysxSchema.PhysxContactReportAPI.Apply(
                stage.GetPrimAtPath(part_contact_path)
            )
            contact_api.CreateThresholdAttr().Set(0.0)

        def on_contact_report(contact_headers, contact_data):
            for header in contact_headers:
                if header.num_contact_data <= 0:
                    continue
                collider0 = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
                collider1 = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
                involves_tool = any(
                    path.startswith(tool_path)
                    for path in (collider0, collider1)
                    for tool_path in tool_contact_paths
                )
                involves_part = any(
                    path.startswith(part_path)
                    for path in (collider0, collider1)
                    for part_path in part_contact_paths
                )
                if not involves_tool and not involves_part:
                    continue
                maximum_impulse = 0.0
                start = header.contact_data_offset
                stop = start + header.num_contact_data
                for index in range(start, stop):
                    impulse = contact_data[index].impulse
                    maximum_impulse = max(
                        maximum_impulse,
                        math.sqrt(sum(float(value) ** 2 for value in impulse)),
                    )
                if maximum_impulse <= 1.0e-9:
                    continue
                record = {
                    "collider0": collider0,
                    "collider1": collider1,
                    "maximum_impulse_Ns": maximum_impulse,
                    "physics_step": physics_step,
                }
                if involves_tool:
                    if len(tool_contact_records) < 100000:
                        tool_contact_records.append(dict(record))
                    else:
                        report["contact_telemetry_truncated"] = True
                if involves_part:
                    if len(assembly_contact_records) < 250000:
                        assembly_contact_records.append(dict(record))
                    else:
                        report["contact_telemetry_truncated"] = True

        contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            on_contact_report
        )
        report["contact_telemetry_enabled"] = True

        wrappers = {}
        for part in plan["parts"]:
            path = "/World/LegoParts/" + part["id"]
            prim = stage.GetPrimAtPath(path)
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
            wrappers[part["id"]] = SingleXFormPrim(path)

        world.reset()
        cameras = []
        if not args.physics_only:
            cameras = [
                Camera(lego_cell.OVERVIEW_CAM_PATH, resolution=(1280, 720)),
                Camera(lego_cell.SIDE_CAM_PATH, resolution=(1280, 720)),
            ]
            for camera in cameras:
                camera.initialize()

        arms = {}
        limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
        ik.set_tool_axis(C.TOOL_AXIS)
        ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        tool = tuple(
            np.asarray(C.TOOL_AXIS) * cell.tip_offset(D.WORKER_FINGER_STYLE)
        )
        for spec in D.ARM_SPECS:
            arm = SingleArticulation(spec["prim_path"], name=spec["name"])
            arm.initialize()
            names = list(arm.dof_names)
            arms[spec["name"]] = {
                "spec": spec,
                "arm": arm,
                "joint_indices": [names.index(f"J{i}") for i in range(1, 7)],
                "indices": {name: index for index, name in enumerate(names)},
                "q": np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)]),
                "jaw": C.JAW_OPEN,
                "plunger": 0.0,
                "clearance_slide": 0.0,
                "j6": SingleXFormPrim(spec["prim_path"] + "/J6_link"),
                "tool": tuple(
                    np.asarray(C.TOOL_AXIS)
                    * cell.tip_offset(
                        D.WORKER_FINGER_STYLE if spec["name"] == "arm_a" else "stock"
                    )
                ),
            }
        if "JawL_retract" not in arms["arm_a"]["indices"]:
            raise RuntimeError("Rev C JawL_retract DOF is missing")
        worker_controller = arms["arm_a"]["arm"].get_articulation_controller()
        worker_kps, worker_kds = worker_controller.get_gains()
        worker_max_efforts = worker_controller.get_max_efforts()
        # Rev B adds a long tool and moving plunger payload. J4 needs calibrated
        # compensation continuously. J2/J3 are added only for the proven tall-
        # part free-space carry profile. Seating retains the same contact-safe
        # gains; clearance comes from the independent insert slide, not a
        # stiffer wrist that can feed collision force into the assembly.
        contact_axis_names = ["J4"]
        seating_axis_names = ["J4"]
        carry_axis_names = ["J2", "J3", "J4", "J5"]
        contact_indices = [
            arms["arm_a"]["indices"][name] for name in contact_axis_names
        ]
        carry_indices = [
            arms["arm_a"]["indices"][name] for name in carry_axis_names
        ]
        seating_indices = [
            arms["arm_a"]["indices"][name] for name in seating_axis_names
        ]
        tuned_effort_indices = sorted(set(carry_indices + seating_indices))
        contact_tuning_factor = 3.0
        carry_tuning_factor = 3.0
        worker_drive_mode = None

        def set_worker_drive_mode(mode):
            nonlocal worker_drive_mode
            if mode == worker_drive_mode:
                return
            tuned_kps = worker_kps.copy()
            tuned_kds = worker_kds.copy()
            tuned_efforts = worker_max_efforts.copy()
            indices = (
                carry_indices
                if mode == "free_space_carry"
                else seating_indices if mode == "seating" else contact_indices
            )
            factor = (
                carry_tuning_factor
                if mode == "free_space_carry"
                else contact_tuning_factor
            )
            tuned_kps[indices] *= factor
            tuned_kds[indices] *= math.sqrt(factor)
            tuned_efforts[indices] *= factor
            worker_controller.set_gains(
                kps=tuned_kps, kds=tuned_kds, save_to_usd=False
            )
            worker_controller.set_max_efforts(
                values=np.asarray(tuned_efforts[tuned_effort_indices], np.float32),
                joint_indices=np.asarray(tuned_effort_indices, dtype=np.int32),
            )
            worker_drive_mode = mode

        set_worker_drive_mode("contact")
        report["worker_runtime_drive_tuning"] = {
            "contact_joints": contact_axis_names,
            "seating_joints": seating_axis_names,
            "free_space_carry_joints": carry_axis_names,
            "contact_stiffness_factor": contact_tuning_factor,
            "contact_damping_factor": math.sqrt(contact_tuning_factor),
            "contact_max_effort_factor": contact_tuning_factor,
            "free_space_carry_stiffness_factor": carry_tuning_factor,
            "free_space_carry_damping_factor": math.sqrt(carry_tuning_factor),
            "free_space_carry_max_effort_factor": carry_tuning_factor,
            "persistent_usd_edit": False,
        }
        support_controller = arms["arm_b"]["arm"].get_articulation_controller()
        support_kps, support_kds = support_controller.get_gains()
        support_max_efforts = support_controller.get_max_efforts()
        support_axis_names = ["J2", "J3", "J4", "J5"]
        support_tuned_indices = [
            arms["arm_b"]["indices"][name] for name in support_axis_names
        ]
        support_tuning_factor = 5.0
        support_kps[support_tuned_indices] *= support_tuning_factor
        support_kds[support_tuned_indices] *= math.sqrt(support_tuning_factor)
        support_max_efforts[support_tuned_indices] *= support_tuning_factor
        support_controller.set_gains(
            kps=support_kps,
            kds=support_kds,
            save_to_usd=False,
        )
        support_controller.set_max_efforts(
            values=np.asarray(
                support_max_efforts[support_tuned_indices], np.float32
            ),
            joint_indices=np.asarray(support_tuned_indices, dtype=np.int32),
        )
        report["support_runtime_drive_tuning"] = {
            "joints": support_axis_names,
            "stiffness_factor": support_tuning_factor,
            "damping_factor": math.sqrt(support_tuning_factor),
            "max_effort_factor": support_tuning_factor,
            "persistent_usd_edit": False,
        }
        worker_grasp_body = SingleXFormPrim(D.ARM_SPECS[0]["prim_path"] + "/JawL")
        support_grasp_body = SingleXFormPrim(
            D.ARM_SPECS[1]["prim_path"] + "/JawL"
        )
        report["worker_fingertip"] = {
            "asset_id": D.WORKER_FINGER_ASSET_ID,
            "style": D.WORKER_FINGER_STYLE,
            "source": D.WORKER_FINGER_SOURCE,
            "required_mount_face_spacing_m": D.WORKER_FINGER_MOUNT_FACE_SPACING_M,
            "simulated_adapter_per_side_m": D.WORKER_FINGER_SIM_ADAPTER_PER_SIDE_M,
            "real_mount_verified": D.WORKER_FINGER_REAL_MOUNT_VERIFIED,
            "independent_jaw_retraction": {
                "joint": "JawL_retract",
                "stroke_m": CLEARANCE_SLIDE_STROKE_M,
                "moving_group": "JawL right_finger_insert + right_finger_pad",
                "geometry_status": "user_supplied_rev_c_simulation_concept",
            },
        }
        def command_one(state):
            arm = state["arm"]
            full = arm.get_joint_positions().copy()
            full[state["joint_indices"]] = state["q"]
            for joint, value in cell.jaw_targets(state["jaw"]).items():
                full[state["indices"][joint]] = value
            if "plunger" in state["indices"]:
                full[state["indices"]["plunger"]] = state["plunger"]
            if "JawL_retract" in state["indices"]:
                full[state["indices"]["JawL_retract"]] = state[
                    "clearance_slide"
                ]
            arm.apply_action(ArticulationAction(joint_positions=full))

        def actual_tool_pose(state):
            """Measured physical J6/tool pose directly from the USD rigid link."""
            position, quat = state["j6"].get_world_pose()
            rotation = _quat_matrix(quat)
            return rotation, np.asarray(position, float) + rotation @ np.asarray(
                state["tool"], float
            )

        frame = 0

        def step(count=1, capture=False):
            nonlocal frame, physics_step
            for _ in range(count):
                for state in arms.values():
                    command_one(state)
                world.step(render=not args.physics_only)
                physics_step += 1
            if capture and cameras:
                for camera, directory in zip(cameras, (frames, side_frames)):
                    rgba = camera.get_rgba()
                    if rgba is None:
                        raise RuntimeError("camera returned no image")
                    cv2.imwrite(
                        str(directory / f"f{frame:05d}.png"),
                        cv2.cvtColor(rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR),
                    )
                frame += 1

        def ramp_jaw_only(state, target, samples=10):
            """Change only the jaw command while holding the current arm pose."""
            start = float(state["jaw"])
            for index in range(samples):
                state["jaw"] = start + (float(target) - start) * _ease(
                    (index + 1) / samples
                )
                step(2, capture=not args.physics_only)

        def ramp_clearance_slide(state, target, samples=10):
            """Retract/extend the independent JawL insert at fixed arm pose."""
            if "JawL_retract" not in state["indices"]:
                raise RuntimeError("worker tool has no JawL_retract joint")
            start = float(state["clearance_slide"])
            for index in range(samples):
                state["clearance_slide"] = start + (float(target) - start) * _ease(
                    (index + 1) / samples
                )
                step(2, capture=not args.physics_only)

        def solve_pose(state, position_world, rotation_world):
            base = np.asarray(state["spec"]["base_position"], float)
            target = np.asarray(position_world, float) - base
            approach = rotation_world @ np.asarray(C.TOOL_AXIS)
            spread = rotation_world @ np.asarray(C.FINGER_SPREAD_AXIS)
            # Damped least-squares can converge to a clipped wrist branch even
            # when the same exact Cartesian pose is reachable from another
            # deterministic seed.  Retry only after the continuous seed fails;
            # move_pose still interpolates every physical waypoint from the
            # current measured branch.
            canonical = np.deg2rad(
                [C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)]
            )
            seeds = [np.asarray(state["q"], float), canonical]
            for base_seed in (np.asarray(state["q"], float), canonical):
                for delta in (-math.pi, math.pi):
                    seed = base_seed.copy()
                    seed[5] = np.clip(
                        seed[5] + delta, limits[5, 0], limits[5, 1]
                    )
                    seeds.append(seed)
            candidates = []
            attempts = []
            for seed_index, seed in enumerate(seeds):
                q, ok = ik.solve(
                    seed,
                    target,
                    approach,
                    limits,
                    tool_offset=state["tool"],
                    spread_dir=spread,
                    iters=500,
                    damping=0.02,
                )
                actual_r, actual_p = fk.tool_pose(q, state["tool"])
                error = float(np.linalg.norm(actual_p - target))
                angle_error = _rotation_error_deg(actual_r, rotation_world)
                attempts.append(
                    {
                        "seed_index": seed_index,
                        "position_error_m": error,
                        "angle_error_deg": angle_error,
                        "solver_ok": bool(ok),
                    }
                )
                # The runtime tracking loop owns the final 1 degree measured
                # gate.  Accept a numerically valid IK seed within 2 degrees
                # here so sub-degree solver/physics settling is not rejected
                # before the articulation gets a chance to track it.
                if ok and error <= 0.001 and angle_error <= 2.0:
                    joint_distance = float(
                        np.linalg.norm(q - np.asarray(state["q"], float))
                    )
                    candidates.append((joint_distance, error, q))
                    if seed_index == 0:
                        break
            if not candidates:
                report["ik_failures"] += 1
                report.setdefault("ik_failure_details", []).append(
                    {
                        "arm": state["spec"]["name"],
                        "target_m": np.asarray(position_world, float).tolist(),
                        "rotation_world": np.asarray(
                            rotation_world, float
                        ).tolist(),
                        "commanded_joint_deg": np.rad2deg(state["q"]).tolist(),
                        "attempts": attempts,
                    }
                )
                best_error = min(item["position_error_m"] for item in attempts)
                raise RuntimeError(
                    f"IK gate failed at {position_world}: "
                    f"{best_error * 1000:.2f} mm"
                )
            _, _, selected_q = min(candidates, key=lambda item: (item[0], item[1]))
            return selected_q

        def move_pose(
            state,
            position_world,
            rotation_world,
            samples=10,
            jaw=None,
            settle=True,
            position_tolerance_m=TRACKING_POSITION_TOLERANCE_M,
            angle_tolerance_deg=TRACKING_ANGLE_TOLERANCE_DEG,
        ):
            start_r, start_p = actual_tool_pose(state)
            target_p = np.asarray(position_world, float)
            target_r = np.asarray(rotation_world, float)
            start_jaw = state["jaw"]
            finish_jaw = start_jaw if jaw is None else float(jaw)
            final_q = solve_pose(state, target_p, target_r)
            max_joint_delta_deg = float(
                np.max(np.abs(np.rad2deg(final_q - state["q"])))
            )
            samples = max(samples, int(math.ceil(max_joint_delta_deg / 0.5)))
            for index in range(samples):
                t = _ease((index + 1) / samples)
                p = start_p + (target_p - start_p) * t
                r = _interpolate_rotation(start_r, target_r, t)
                state["q"] = solve_pose(state, p, r)
                state["jaw"] = start_jaw + (finish_jaw - start_jaw) * t
                step(2, capture=not args.physics_only)
                for _ in range(24):
                    measured_q = state["arm"].get_joint_positions()[
                        state["joint_indices"]
                    ]
                    if np.max(np.abs(np.rad2deg(measured_q - state["q"]))) <= 0.5:
                        break
                    step(1, capture=not args.physics_only)

            # The trajectory above updates commands at video-rate, while the
            # physical articulation follows through drive dynamics. Do not use
            # the target FK as feedback: wait for the measured wrist to arrive.
            if not settle:
                return
            settled = False
            for _ in range(240):
                actual_r, actual_p = actual_tool_pose(state)
                if (
                    np.linalg.norm(actual_p - target_p) <= position_tolerance_m
                    and _rotation_error_deg(actual_r, target_r)
                    <= angle_tolerance_deg
                ):
                    settled = True
                    break
                step(1, capture=not args.physics_only)
            if not settled:
                actual_r, actual_p = actual_tool_pose(state)
                actual_q = state["arm"].get_joint_positions()[state["joint_indices"]]
                arm_prefix = state["spec"]["prim_path"] + "/"
                recent_contacts = [
                    record
                    for record in tool_contact_records[-200:]
                    if record["collider0"].startswith(arm_prefix)
                    or record["collider1"].startswith(arm_prefix)
                ]
                report["tracking_gate_misses"].append(
                    {
                        "arm": state["spec"]["name"],
                        "target_m": target_p.tolist(),
                        "position_error_m": float(np.linalg.norm(actual_p - target_p)),
                        "angle_error_deg": _rotation_error_deg(actual_r, target_r),
                        "actual_position_m": actual_p.tolist(),
                        "commanded_joint_deg": np.rad2deg(state["q"]).tolist(),
                        "actual_joint_deg": np.rad2deg(actual_q).tolist(),
                        "max_joint_error_deg": float(
                            np.max(np.abs(np.rad2deg(actual_q - state["q"])))
                        ),
                        "recent_contact_pairs": recent_contacts[-50:],
                    }
                )
                raise RuntimeError(
                    "tool tracking gate failed: "
                    f"arm={state['spec']['name']} "
                    f"{np.linalg.norm(actual_p-target_p)*1000:.2f} mm, "
                    f"{_rotation_error_deg(actual_r,target_r):.2f} deg"
                )

        def park_arm(state, target_p, samples=14):
            """Park outside the shared zone without canonicalizing jaw roll."""
            measured_r, _ = actual_tool_pose(state)
            move_pose(state, target_p, measured_r, samples)

        # Tool local +X points down; local +Y (jaw spread) points world +Y.
        down_r = np.column_stack(
            (np.array((0.0, 0.0, -1.0)), np.array((0.0, 1.0, 0.0)), np.array((1.0, 0.0, 0.0)))
        )
        # Keep both arms in separated half-spaces until arm B owns the fixture.
        home_world = {
            "arm_a": np.array((0.24, -0.16, 1.18)),
            "arm_b": np.array((0.24, 0.17, 1.18)),
        }
        for name, state in arms.items():
            state["q"] = solve_pose(state, home_world[name], down_r)
            full = state["arm"].get_joint_positions().copy()
            full[state["joint_indices"]] = state["q"]
            for joint, value in cell.jaw_targets(C.JAW_OPEN).items():
                full[state["indices"][joint]] = value
            if "plunger" in state["indices"]:
                full[state["indices"]["plunger"]] = 0.0
            if "JawL_retract" in state["indices"]:
                full[state["indices"]["JawL_retract"]] = 0.0
            state["arm"].set_joint_positions(full)
        step(60, capture=not args.physics_only)

        # Arm B closes on the real collision-enabled support handle and remains
        # commanded there for every subsequent physics step.
        support = arms["arm_b"]
        support_xy = D.FIXTURE_HANDLE_XY
        support_tip = np.array((support_xy[0], support_xy[1], D.SURFACE_Z + 0.004))
        move_pose(support, support_tip + (0, 0, 0.08), down_r, 10)
        move_pose(support, support_tip, down_r, 8, settle=False)
        support_target = max(C.JAW_CLOSED, D.FIXTURE_HANDLE_SIZE[1] / 2 - C.FINGER_INNER_TIP - 0.0003)
        move_pose(support, support_tip, down_r, 6, jaw=support_target, settle=False)
        step(30, capture=not args.physics_only)
        report["fixture_support"] = {
            "arm": "arm_b", "handle": "/World/LegoFixture/SupportHandle",
            "jaw_target_per_side_m": support_target, "held_for_all_tasks": True,
        }

        worker = arms["arm_a"]
        part_by_id = {part["id"]: part for part in plan["parts"]}
        task_by_part = {task["part"]: task for task in plan["tasks"]}
        def pose(part_id):
            position, quat = wrappers[part_id].get_world_pose()
            return np.asarray(position, float), _quat_matrix(quat), np.asarray(quat, float)

        def set_connector_guide(task_id, enabled):
            root_path = f"/World/LegoConnectorGuides/{task_id}"
            root = stage.GetPrimAtPath(root_path)
            if not root.IsValid():
                raise RuntimeError(f"missing declared connector guide: {root_path}")
            toggled = []
            for prim in root.GetChildren():
                collision = prim.GetAttribute("physics:collisionEnabled")
                if collision.IsValid():
                    collision.Set(bool(enabled))
                    toggled.append(str(prim.GetPath()))
            if not toggled:
                raise RuntimeError(f"connector guide has no colliders: {root_path}")
            return toggled

        def connector_lock(task, position, rotation):
            path = f"/World/LegoConnectorLocks/{task['id']}"
            joint = UsdPhysics.FixedJoint.Get(stage, path)
            if not joint.GetPrim().IsValid():
                raise RuntimeError(f"missing pre-authored connector lock: {path}")
            joint.GetJointEnabledAttr().Set(False)
            child_path = "/World/LegoParts/" + task["part"]
            parent_id = task.get("connector_parent")
            locked_body_paths = [child_path]
            if parent_id is not None:
                locked_body_paths.append("/World/LegoParts/" + parent_id)
            for body_path in locked_body_paths:
                body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(
                    stage.GetPrimAtPath(body_path)
                )
                body_api.CreateSolverPositionIterationCountAttr().Set(64)
                body_api.CreateSolverVelocityIterationCountAttr().Set(8)
            joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
            if parent_id is None:
                local_p, local_r = position, rotation
            else:
                parent_p, parent_r, _ = pose(parent_id)
                local_p = parent_r.T @ (position - parent_p)
                local_r = parent_r.T @ rotation
                joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/LegoParts/" + parent_id)])
            local_q = _matrix_quat(local_r)
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_p))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(local_q[0]), Gf.Vec3f(*local_q[1:])))
            joint.CreateBreakForceAttr().Set(float("inf"))
            joint.CreateBreakTorqueAttr().Set(float("inf"))
            # Match the proven grasp-joint activation sequence: give PhysX one
            # disabled step to consume both measured frames before enabling.
            # Otherwise it can retain the pre-authored identity frames and the
            # child starts correcting toward its parent's origin after support
            # is removed.
            step(1, capture=not args.physics_only)
            joint.GetJointEnabledAttr().Set(True)
            return path, parent_id or "world"

        turnover_state = {
            "active": False,
            "inverted": False,
            "home_part_position": None,
            "home_part_rotation": None,
            "part_from_tool_position": None,
            "part_from_tool_rotation": None,
            # Keep the horizontal camera housing clear of the mat across the
            # full measured J6 settling range.  The earlier 1.02 m root pose
            # was marginal; 30 mm more height leaves the assembly accessible
            # to vertical Arm A while preserving the 1.12 m roll waypoint.
            "work_part_position": np.asarray((0.22, -0.02, 1.05), float),
            "worker_lift_joint_active": False,
        }
        # Arm B approaches the bracket wall horizontally from +Y. Rolling its
        # wrist 180 degrees about the tool's local X axis therefore produces a
        # world-Y turnover while keeping the 293 mm TCP horizontal and clear
        # of the table.
        turnover_rotation_world = np.diag((-1.0, 1.0, -1.0))
        turnover_rotation_tool = np.diag((1.0, -1.0, -1.0))

        def _turnover_joint():
            joint = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoAssemblyTurnover"
            )
            if not joint.GetPrim().IsValid():
                raise RuntimeError("missing pre-authored LEGO turnover joint")
            return joint

        def _turnover_contact_sides(start_index):
            bracket_prefix = "/World/LegoParts/p74_bracket"
            sides = set()
            for record in assembly_contact_records[start_index:]:
                paths = (record["collider0"], record["collider1"])
                if not any(path.startswith(bracket_prefix) for path in paths):
                    continue
                for side in ("JawL", "JawR"):
                    if any(
                        path.startswith(
                            D.ARM_SPECS[1]["prim_path"] + "/" + side
                        )
                        for path in paths
                    ):
                        sides.add(side)
            return sorted(sides)

        def _worker_raise_for_turnover():
            """Arm A raises the assembly before Arm B's horizontal handoff."""
            bracket_p, bracket_r, _ = pose("p74_bracket")
            turnover_state["home_part_position"] = bracket_p.copy()
            turnover_state["home_part_rotation"] = bracket_r.copy()
            grasp_local = np.asarray((0.0, 0.0048, 0.011), float)
            grasp_tip = bracket_p + bracket_r @ grasp_local
            spread = bracket_r @ np.asarray((1.0, 0.0, 0.0), float)
            spread[2] = 0.0
            spread /= np.linalg.norm(spread)
            if spread[int(np.argmax(np.abs(spread)))] < 0.0:
                spread = -spread
            grasp_r = np.column_stack(
                (
                    -np.asarray((0.0, 0.0, 1.0)),
                    spread,
                    np.cross(-np.asarray((0.0, 0.0, 1.0)), spread),
                )
            )
            hover = grasp_tip + np.asarray((0.0, 0.0, 0.07))
            move_pose(worker, hover, grasp_r, 12)
            move_pose(worker, grasp_tip, grasp_r, 8, settle=False)
            worker_grip_target = (
                2 * D.LEGO_PITCH
                - D.WORKER_FINGER_CLOSED_PAD_GAP_M
                - 0.0007
            ) / 2
            contact_start = len(assembly_contact_records)
            move_pose(
                worker,
                grasp_tip,
                grasp_r,
                8,
                jaw=worker_grip_target,
                settle=False,
            )
            step(20, capture=not args.physics_only)
            bracket_prefix = "/World/LegoParts/p74_bracket"
            contact_sides = set()
            for record in assembly_contact_records[contact_start:]:
                paths = (record["collider0"], record["collider1"])
                if not any(path.startswith(bracket_prefix) for path in paths):
                    continue
                if any(
                    path.startswith(D.ARM_SPECS[0]["prim_path"] + "/JawL")
                    or path.startswith(
                        D.ARM_SPECS[0]["prim_path"] + "/JawLClearance"
                    )
                    for path in paths
                ):
                    contact_sides.add("JawL")
                if any(
                    path.startswith(D.ARM_SPECS[0]["prim_path"] + "/JawR")
                    for path in paths
                ):
                    contact_sides.add("JawR")
            if sorted(contact_sides) != ["JawL", "JawR"]:
                raise RuntimeError(
                    "Arm A turnover lift grasp gate failed: "
                    f"contact sides={sorted(contact_sides)}"
                )

            jaw_p, jaw_q = worker_grasp_body.get_world_pose()
            jaw_p = np.asarray(jaw_p, float)
            jaw_r = _quat_matrix(jaw_q)
            held_p, held_r, _ = pose("p74_bracket")
            local_p = jaw_r.T @ (held_p - jaw_p)
            local_r = jaw_r.T @ held_r
            local_q = _matrix_quat(local_r)
            lift_joint = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoGraspConstraints/t00"
            )
            lift_joint.GetJointEnabledAttr().Set(False)
            lift_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*local_p))
            lift_joint.GetLocalRot0Attr().Set(
                Gf.Quatf(float(local_q[0]), Gf.Vec3f(*local_q[1:]))
            )
            lift_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            lift_joint.GetLocalRot1Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            step(1, capture=not args.physics_only)
            lift_joint.GetJointEnabledAttr().Set(True)
            step(4, capture=not args.physics_only)

            seed_lock = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoConnectorLocks/t00"
            )
            before_p, before_r, _ = pose("p74_bracket")
            seed_lock.GetJointEnabledAttr().Set(False)
            step(4, capture=not args.physics_only)
            after_p, after_r, _ = pose("p74_bracket")
            position_jump = float(np.linalg.norm(after_p - before_p))
            angle_jump = _rotation_error_deg(after_r, before_r)
            if position_jump > 0.0002 or angle_jump > 1.5:
                raise RuntimeError(
                    "Arm A turnover lift handoff jump gate failed: "
                    f"{position_jump*1000:.3f} mm/{angle_jump:.3f} deg"
                )
            lift_r, lift_p = actual_tool_pose(worker)
            carried_p, carried_r, _ = pose("p74_bracket")
            part_from_tcp_p = lift_r.T @ (carried_p - lift_p)
            part_from_tcp_r = lift_r.T @ carried_r
            work_part_p = turnover_state["work_part_position"]
            work_part_r = turnover_state["home_part_rotation"]
            work_tool_r = work_part_r @ part_from_tcp_r.T
            work_tool_p = work_part_p - work_tool_r @ part_from_tcp_p
            move_pose(
                worker,
                work_tool_p,
                work_tool_r,
                28,
            )
            raised_p, raised_r, _ = pose("p74_bracket")
            raised_position_error = float(
                np.linalg.norm(raised_p - work_part_p)
            )
            raised_angle_error = _rotation_error_deg(raised_r, work_part_r)
            if raised_position_error > 0.003 or raised_angle_error > 1.5:
                raise RuntimeError(
                    "Arm A turnover presentation gate failed: "
                    f"{raised_position_error*1000:.2f} mm/"
                    f"{raised_angle_error:.2f} deg"
                )
            turnover_state["worker_lift_joint_active"] = True
            report["assembly_turnover"] = {
                "arm_a_lift_contacting_jaw_sides": sorted(contact_sides),
                "arm_a_lift_m": float(
                    raised_p[2] - turnover_state["home_part_position"][2]
                ),
                "arm_a_presentation_position_error_m": raised_position_error,
                "arm_a_presentation_angle_error_deg": raised_angle_error,
                "work_part_position_m": work_part_p.tolist(),
                "world_lock_disabled_after_arm_a_grasp": True,
                "arm_a_handoff_position_jump_m": position_jump,
                "arm_a_handoff_angle_jump_deg": angle_jump,
                "hidden_part_pose_write": False,
            }

        def _rotate_turnover_assembly(inverted):
            if not turnover_state["active"]:
                raise RuntimeError("turnover rotation requested before Arm B handoff")
            if bool(turnover_state["inverted"]) == bool(inverted):
                return
            # The held subassembly grows after every placement. Park Arm A
            # before any turnover so its post-release withdrawal pose cannot
            # occupy the enlarged swept volume.
            park_arm(worker, home_world["arm_a"])
            current_part_r = (
                turnover_rotation_world @ turnover_state["home_part_rotation"]
                if turnover_state["inverted"]
                else turnover_state["home_part_rotation"]
            )
            clearance_part_p = turnover_state["work_part_position"].copy()
            clearance_part_p[2] = 1.12
            part_from_tool_p = turnover_state["part_from_tool_position"]
            part_from_tool_r = turnover_state["part_from_tool_rotation"]
            clearance_tool_r = current_part_r @ part_from_tool_r.T
            clearance_tool_p = (
                clearance_part_p - clearance_tool_r @ part_from_tool_p
            )
            move_pose(support, clearance_tool_p, clearance_tool_r, 24)
            current_r, current_p = actual_tool_pose(support)
            target_r = current_r @ turnover_rotation_tool
            before_p, before_r, _ = pose("p74_bracket")
            # Cartesian IK intentionally treats the parallel-jaw spread as an
            # unoriented line. A 180-degree roll about tool X therefore looks
            # equivalent and can return the unchanged wrist solution. Drive
            # the physical J6 roll explicitly while holding J1-J5 fixed; the
            # TCP offset lies on that axis, so its position remains fixed.
            start_q = support["q"].copy()
            target_q = start_q.copy()
            target_q[5] += -math.pi if inverted else math.pi
            if not limits[5, 0] <= target_q[5] <= limits[5, 1]:
                raise RuntimeError(
                    "Arm B turnover J6 target is outside joint limits: "
                    f"{math.degrees(target_q[5]):.2f} deg"
                )
            for index in range(120):
                fraction = _ease((index + 1) / 120)
                support["q"] = start_q + (target_q - start_q) * fraction
                step(2, capture=not args.physics_only)
            settled = False
            for _ in range(180):
                measured = support["arm"].get_joint_positions()[
                    support["joint_indices"]
                ]
                if abs(math.degrees(measured[5] - target_q[5])) <= 0.5:
                    settled = True
                    break
                step(1, capture=not args.physics_only)
            if not settled:
                arm_b_prefix = D.ARM_SPECS[1]["prim_path"] + "/"
                report.setdefault("assembly_turnover", {})[
                    "j6_tracking_failure_contacts"
                ] = [
                    record
                    for record in tool_contact_records[-200:]
                    if record["collider0"].startswith(arm_b_prefix)
                    or record["collider1"].startswith(arm_b_prefix)
                ][-50:]
                raise RuntimeError(
                    "Arm B turnover J6 tracking gate failed: "
                    f"target={math.degrees(target_q[5]):.2f} deg, "
                    f"actual={math.degrees(measured[5]):.2f} deg"
                )
            rolled_r, rolled_p = actual_tool_pose(support)
            roll_position_change = float(np.linalg.norm(rolled_p - current_p))
            roll_angle_error = _rotation_error_deg(rolled_r, target_r)
            if roll_position_change > 0.002 or roll_angle_error > 1.0:
                raise RuntimeError(
                    "Arm B turnover wrist-roll gate failed: "
                    f"{roll_position_change*1000:.2f} mm/"
                    f"{roll_angle_error:.2f} deg"
                )
            step(8, capture=not args.physics_only)
            desired_part_r = (
                turnover_rotation_world @ turnover_state["home_part_rotation"]
                if inverted
                else turnover_state["home_part_rotation"]
            )
            desired_part_p = turnover_state["work_part_position"]
            part_from_tool_p = turnover_state["part_from_tool_position"]
            part_from_tool_r = turnover_state["part_from_tool_rotation"]
            desired_tool_r = desired_part_r @ part_from_tool_r.T
            desired_tool_p = desired_part_p - desired_tool_r @ part_from_tool_p
            move_pose(support, desired_tool_p, desired_tool_r, 18)
            step(8, capture=not args.physics_only)
            after_p, after_r, _ = pose("p74_bracket")
            measured_delta = after_r @ before_r.T
            turnover_state["inverted"] = bool(inverted)
            report["fixture_transitions"].append(
                {
                    "action": (
                        "present_underside" if inverted else "restore_upright_hold"
                    ),
                    "mechanism": "arm_b_measured_grasp_turnover",
                    "clearance_part_position_m": clearance_part_p.tolist(),
                    "bracket_position_change_m": float(
                        np.linalg.norm(after_p - before_p)
                    ),
                    "rotation_command_deg": 180.0,
                    "j6_target_deg": math.degrees(target_q[5]),
                    "j6_measured_deg": math.degrees(measured[5]),
                    "tcp_position_change_during_roll_m": roll_position_change,
                    "wrist_roll_angle_error_deg": roll_angle_error,
                    "rotation_error_deg": _rotation_error_deg(
                        measured_delta, turnover_rotation_world
                    ),
                    "hidden_part_pose_write": False,
                }
            )

        def _acquire_and_present_underside():
            if turnover_state["active"]:
                _rotate_turnover_assembly(True)
                return

            # Arm B's long stock tool cannot take a horizontal grasp at table
            # height. Arm A first lifts the real assembly, then the two arms
            # perform a measured overlap handoff before Arm A retreats.
            _worker_raise_for_turnover()
            ramp_jaw_only(support, C.JAW_OPEN, samples=10)
            support_handle = stage.GetPrimAtPath(
                "/World/LegoFixture/SupportHandle"
            )
            support_handle.GetAttribute("physics:collisionEnabled").Set(False)
            UsdGeom.Imageable(support_handle).CreateVisibilityAttr().Set(
                "invisible"
            )
            report["fixture_transitions"].append(
                {
                    "action": "retract_support_handle",
                    "after_arm_b_release": True,
                    "collision_enabled": False,
                }
            )
            report["fixture_support"].update(
                {
                    "held_for_all_tasks": False,
                    "initial_handle_hold_verified": True,
                    "released_for_assembly_turnover": True,
                    "subsequent_support": "arm_b_root_bracket_grasp",
                }
            )
            move_pose(
                support,
                home_world["arm_b"],
                down_r,
                14,
            )

            bracket_p, bracket_r, _ = pose("p74_bracket")
            # Arm A owns the tall +Y wall during the overlap handoff. Arm B
            # pinches the 15.7 mm-wide base shell below the tile instead, so
            # the two real finger envelopes occupy disjoint height bands.
            grasp_local = np.asarray((0.0, 0.0, 0.0015), float)
            grasp_tip = bracket_p + bracket_r @ grasp_local
            # The stock wrist reaches this side grasp only in the equivalent
            # negative-X jaw-spread hemisphere.  A jaw spread is an unoriented
            # line physically, but this wrist branch avoids the J4/J5 limits
            # hit by the positive-X representation.
            spread = -(bracket_r @ np.asarray((1.0, 0.0, 0.0), float))
            spread[2] = 0.0
            spread /= np.linalg.norm(spread)
            approach_axis = np.asarray((0.0, -1.0, 0.0), float)
            grasp_r = np.column_stack(
                (approach_axis, spread, np.cross(approach_axis, spread))
            )
            grasp_hover = grasp_tip - approach_axis * 0.07
            move_pose(support, grasp_hover, grasp_r, 12)
            move_pose(support, grasp_tip, grasp_r, 8, settle=False)
            bracket_width = 2 * D.LEGO_PITCH - 0.0003
            turnover_jaw = max(
                C.JAW_CLOSED,
                min(
                    C.JAW_OPEN,
                    bracket_width / 2 - C.FINGER_INNER_TIP - 0.0003,
                ),
            )
            contact_start = len(assembly_contact_records)
            move_pose(
                support,
                grasp_tip,
                grasp_r,
                8,
                jaw=turnover_jaw,
                settle=False,
            )
            step(20, capture=not args.physics_only)
            contact_sides = _turnover_contact_sides(contact_start)
            if contact_sides != ["JawL", "JawR"]:
                raise RuntimeError(
                    "Arm B turnover grasp gate failed: "
                    f"contact sides={contact_sides}"
                )

            jaw_p, jaw_q = support_grasp_body.get_world_pose()
            jaw_p = np.asarray(jaw_p, float)
            jaw_r = _quat_matrix(jaw_q)
            held_p, held_r, _ = pose("p74_bracket")
            part_from_jaw_p = jaw_r.T @ (held_p - jaw_p)
            part_from_jaw_r = jaw_r.T @ held_r
            held_tool_r, held_tool_p = actual_tool_pose(support)
            part_from_tool_p = held_tool_r.T @ (held_p - held_tool_p)
            part_from_tool_r = held_tool_r.T @ held_r
            local_q = _matrix_quat(part_from_jaw_r)
            joint = _turnover_joint()
            joint.GetJointEnabledAttr().Set(False)
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*part_from_jaw_p))
            joint.GetLocalRot0Attr().Set(
                Gf.Quatf(float(local_q[0]), Gf.Vec3f(*local_q[1:]))
            )
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.GetLocalRot1Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            step(1, capture=not args.physics_only)
            joint.GetJointEnabledAttr().Set(True)
            # Give the two coincident grasp constraints time to converge before
            # Arm A releases the now-heavier elevated subassembly.
            step(20, capture=not args.physics_only)

            # Re-zero Arm B's fixed frame after the physical pads and both
            # articulations have settled.  Otherwise a small residual strain
            # between the two valid grasp frames is released as a visible
            # rotation when Arm A lets go.
            joint.GetJointEnabledAttr().Set(False)
            step(1, capture=not args.physics_only)
            settled_jaw_p, settled_jaw_q = support_grasp_body.get_world_pose()
            settled_jaw_p = np.asarray(settled_jaw_p, float)
            settled_jaw_r = _quat_matrix(settled_jaw_q)
            settled_held_p, settled_held_r, _ = pose("p74_bracket")
            settled_local_p = settled_jaw_r.T @ (
                settled_held_p - settled_jaw_p
            )
            settled_local_r = settled_jaw_r.T @ settled_held_r
            settled_local_q = _matrix_quat(settled_local_r)
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*settled_local_p))
            joint.GetLocalRot0Attr().Set(
                Gf.Quatf(
                    float(settled_local_q[0]),
                    Gf.Vec3f(*settled_local_q[1:]),
                )
            )
            step(1, capture=not args.physics_only)
            joint.GetJointEnabledAttr().Set(True)
            step(20, capture=not args.physics_only)

            held_tool_r, held_tool_p = actual_tool_pose(support)
            held_p, held_r, _ = pose("p74_bracket")
            part_from_tool_p = held_tool_r.T @ (held_p - held_tool_p)
            part_from_tool_r = held_tool_r.T @ held_r

            lift_joint = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoGraspConstraints/t00"
            )
            before_handoff_p, before_handoff_r, _ = pose("p74_bracket")
            lift_joint.GetJointEnabledAttr().Set(False)
            ramp_jaw_only(worker, C.JAW_OPEN, samples=10)
            step(4, capture=not args.physics_only)
            after_handoff_p, after_handoff_r, _ = pose("p74_bracket")
            handoff_position_jump = float(
                np.linalg.norm(after_handoff_p - before_handoff_p)
            )
            handoff_angle_jump = _rotation_error_deg(
                after_handoff_r, before_handoff_r
            )
            # A suspended load can elastically settle when one arm releases.
            # Reject a genuine slip here, then remeasure the loaded relation
            # and require the original strict pose gate at the work pose.
            if handoff_position_jump > 0.002 or handoff_angle_jump > 5.0:
                raise RuntimeError(
                    "Arm B turnover catastrophic-slip gate failed: "
                    f"{handoff_position_jump*1000:.3f} mm/"
                    f"{handoff_angle_jump:.3f} deg"
                )
            worker_release_r, worker_release_p = actual_tool_pose(worker)
            move_pose(
                worker,
                worker_release_p + np.asarray((0.0, 0.0, 0.060)),
                worker_release_r,
                10,
            )
            park_arm(worker, home_world["arm_a"])

            loaded_tool_r, loaded_tool_p = actual_tool_pose(support)
            loaded_part_p, loaded_part_r, _ = pose("p74_bracket")
            part_from_tool_p = loaded_tool_r.T @ (
                loaded_part_p - loaded_tool_p
            )
            part_from_tool_r = loaded_tool_r.T @ loaded_part_r

            turnover_state.update(
                {
                    "active": True,
                    "inverted": False,
                    "part_from_tool_position": part_from_tool_p,
                    "part_from_tool_rotation": part_from_tool_r,
                    "worker_lift_joint_active": False,
                }
            )
            work_part_p = turnover_state["work_part_position"]
            work_part_r = turnover_state["home_part_rotation"]
            work_tool_r = work_part_r @ part_from_tool_r.T
            work_tool_p = work_part_p - work_tool_r @ part_from_tool_p
            move_pose(
                support,
                work_tool_p,
                work_tool_r,
                32,
            )
            presentation_corrections = []
            for _ in range(30):
                presented_p, presented_r, _ = pose("p74_bracket")
                position_error_vector = work_part_p - presented_p
                angle_error = _rotation_error_deg(presented_r, work_part_r)
                presentation_corrections.append(
                    {
                        "position_error_m": position_error_vector.tolist(),
                        "angle_error_deg": angle_error,
                    }
                )
                if (
                    np.linalg.norm(position_error_vector)
                    <= D.CONNECTOR_POSITION_TOLERANCE_M
                    and angle_error <= D.CONNECTOR_ANGLE_TOLERANCE_DEG
                ):
                    break
                current_tool_r, current_tool_p = actual_tool_pose(support)
                full_corrected_r = (
                    work_part_r @ presented_r.T
                ) @ current_tool_r
                rotation_fraction = min(0.5, 5.0 / max(angle_error, 1e-9))
                corrected_tool_r = _interpolate_rotation(
                    current_tool_r, full_corrected_r, rotation_fraction
                )
                correction = 0.5 * position_error_vector
                correction_norm = float(np.linalg.norm(correction))
                if correction_norm > 0.004:
                    correction *= 0.004 / correction_norm
                move_pose(
                    support,
                    current_tool_p + correction,
                    corrected_tool_r,
                    6,
                    settle=False,
                )
                step(8, capture=not args.physics_only)
            presented_p, presented_r, _ = pose("p74_bracket")
            presentation_position_error = float(
                np.linalg.norm(presented_p - work_part_p)
            )
            presentation_angle_error = _rotation_error_deg(
                presented_r, work_part_r
            )
            if (
                presentation_position_error
                > D.CONNECTOR_POSITION_TOLERANCE_M
                or presentation_angle_error
                > D.CONNECTOR_ANGLE_TOLERANCE_DEG
            ):
                raise RuntimeError(
                    "Arm B loaded presentation gate failed: "
                    f"{presentation_position_error*1000:.3f} mm/"
                    f"{presentation_angle_error:.3f} deg"
                )
            # The removable support insert is withdrawn after Arm B owns the
            # assembly. Keep the shallow locator walls through the remaining
            # elevated placements; they are retracted only immediately before
            # the enlarged assembly returns to the fixture.
            base_prim = stage.GetPrimAtPath("/World/LegoFixture/Base")
            base_prim.GetAttribute("physics:collisionEnabled").Set(False)
            report["fixture_transitions"].append(
                {
                    "action": "withdraw_fixture_insert",
                    "after_arm_b_handoff": True,
                    "collision_enabled": False,
                }
            )
            report["assembly_turnover"].update(
                {
                    "arm_b_grasp_part": "p74_bracket",
                    "arm_b_grasp_local_m": grasp_local.tolist(),
                    "arm_b_jaw_target_per_side_m": turnover_jaw,
                    "arm_b_contacting_jaw_sides": contact_sides,
                    "arm_a_released_after_arm_b_grasp": True,
                    "arm_b_handoff_position_jump_m": handoff_position_jump,
                    "arm_b_handoff_angle_jump_deg": handoff_angle_jump,
                    "arm_b_loaded_presentation_position_error_m": (
                        presentation_position_error
                    ),
                    "arm_b_loaded_presentation_angle_error_deg": (
                        presentation_angle_error
                    ),
                    "arm_b_loaded_presentation_corrections": (
                        presentation_corrections
                    ),
                    "work_part_position_m": work_part_p.tolist(),
                    "turnover_axis": "arm_b_tool_local_x/world_y",
                    "hidden_part_pose_write": False,
                }
            )
            _rotate_turnover_assembly(True)

        def _restore_upright_and_dock():
            if not turnover_state["active"]:
                raise RuntimeError("upright docking requested without turnover grasp")
            _rotate_turnover_assembly(False)

            # Arm B's 293 mm camera-equipped tool can turn the subassembly at
            # clearance height, but it cannot descend horizontally to the mat.
            # Recreate the proven high-wall grasp with vertical Arm A and keep
            # both constraints active during the overlap handback.
            bracket_p, bracket_r, _ = pose("p74_bracket")
            grasp_local = np.asarray((0.0, 0.0048, 0.011), float)
            grasp_tip = bracket_p + bracket_r @ grasp_local
            spread = bracket_r @ np.asarray((1.0, 0.0, 0.0), float)
            spread[2] = 0.0
            spread /= np.linalg.norm(spread)
            if spread[int(np.argmax(np.abs(spread)))] < 0.0:
                spread = -spread
            grasp_r = np.column_stack(
                (
                    -np.asarray((0.0, 0.0, 1.0)),
                    spread,
                    np.cross(-np.asarray((0.0, 0.0, 1.0)), spread),
                )
            )
            hover = grasp_tip + np.asarray((0.0, 0.0, 0.07))
            move_pose(worker, hover, grasp_r, 12)
            move_pose(worker, grasp_tip, grasp_r, 8, settle=False)
            worker_grip_target = (
                2 * D.LEGO_PITCH
                - D.WORKER_FINGER_CLOSED_PAD_GAP_M
                - 0.0007
            ) / 2
            contact_start = len(assembly_contact_records)
            move_pose(
                worker,
                grasp_tip,
                grasp_r,
                8,
                jaw=worker_grip_target,
                settle=False,
            )
            step(20, capture=not args.physics_only)
            bracket_prefix = "/World/LegoParts/p74_bracket"
            contact_sides = set()
            for record in assembly_contact_records[contact_start:]:
                paths = (record["collider0"], record["collider1"])
                if not any(path.startswith(bracket_prefix) for path in paths):
                    continue
                if any(
                    path.startswith(D.ARM_SPECS[0]["prim_path"] + "/JawL")
                    or path.startswith(
                        D.ARM_SPECS[0]["prim_path"] + "/JawLClearance"
                    )
                    for path in paths
                ):
                    contact_sides.add("JawL")
                if any(
                    path.startswith(D.ARM_SPECS[0]["prim_path"] + "/JawR")
                    for path in paths
                ):
                    contact_sides.add("JawR")
            if sorted(contact_sides) != ["JawL", "JawR"]:
                raise RuntimeError(
                    "Arm A upright handback grasp gate failed: "
                    f"contact sides={sorted(contact_sides)}"
                )

            jaw_p, jaw_q = worker_grasp_body.get_world_pose()
            jaw_p = np.asarray(jaw_p, float)
            jaw_r = _quat_matrix(jaw_q)
            held_p, held_r, _ = pose("p74_bracket")
            local_p = jaw_r.T @ (held_p - jaw_p)
            local_r = jaw_r.T @ held_r
            local_q = _matrix_quat(local_r)
            lift_joint = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoGraspConstraints/t00"
            )
            lift_joint.GetJointEnabledAttr().Set(False)
            lift_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*local_p))
            lift_joint.GetLocalRot0Attr().Set(
                Gf.Quatf(float(local_q[0]), Gf.Vec3f(*local_q[1:]))
            )
            lift_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            lift_joint.GetLocalRot1Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            step(1, capture=not args.physics_only)
            lift_joint.GetJointEnabledAttr().Set(True)
            step(4, capture=not args.physics_only)

            before_handoff_p, before_handoff_r, _ = pose("p74_bracket")
            turnover_joint = _turnover_joint()
            turnover_joint.GetJointEnabledAttr().Set(False)
            step(4, capture=not args.physics_only)
            after_handoff_p, after_handoff_r, _ = pose("p74_bracket")
            handback_position_jump = float(
                np.linalg.norm(after_handoff_p - before_handoff_p)
            )
            handback_angle_jump = _rotation_error_deg(
                after_handoff_r, before_handoff_r
            )
            if handback_position_jump > 0.0002 or handback_angle_jump > 1.5:
                raise RuntimeError(
                    "Arm B-to-A upright handback jump gate failed: "
                    f"{handback_position_jump*1000:.3f} mm/"
                    f"{handback_angle_jump:.3f} deg"
                )

            ramp_jaw_only(support, C.JAW_OPEN, samples=10)
            support_release_r, support_release_p = actual_tool_pose(support)
            move_pose(
                support,
                support_release_p - support_release_r[:, 0] * 0.08,
                support_release_r,
                12,
            )
            # The first jumper spans two parents and needs the full shared
            # workspace for its corrective seating motion.  An 80 mm release
            # retreat clears the root, but a horizontal 293 mm camera/tool
            # still leaves Arm B's jaws in Arm A's elbow envelope.  Reorient
            # the unloaded tool vertically and park its tip near Arm B's own
            # base, outside the declared shared-zone Y bound.
            support_clear_park = np.asarray((0.18, 0.25, 1.24), float)
            move_pose(support, support_clear_park, down_r, 24)

            # Now that every elevated placement is complete, retract the
            # three seed locators before lowering the larger assembly.  In
            # particular, the step-77 tile has only 0.5 mm nominal clearance
            # to WallBack; compliant handback error can otherwise put it onto
            # the wall's top edge and tilt the connector chain.  The measured
            # world lock below becomes the locating datum after retraction.
            retracted_walls = []
            for wall_path in (
                "/World/LegoFixture/WallBack",
                "/World/LegoFixture/WallLeft",
                "/World/LegoFixture/WallRight",
            ):
                wall = stage.GetPrimAtPath(wall_path)
                wall.GetAttribute("physics:collisionEnabled").Set(False)
                UsdGeom.Imageable(wall).CreateVisibilityAttr().Set(
                    "invisible"
                )
                retracted_walls.append(wall_path)
            report["fixture_transitions"].append(
                {
                    "action": "retract_seed_locator_walls",
                    "before_upright_redock": True,
                    "collision_enabled": False,
                    "visibility": "invisible",
                    "retracted_locators": retracted_walls,
                    "mechanical_reason": (
                        "clear_enlarged_assembly_before_upright_redock"
                    ),
                }
            )

            # Compute the current measured Arm-A-to-bracket transform, then
            # lower that real held assembly to the original fixture pose.
            desired_part_p = turnover_state["home_part_position"]
            desired_part_r = turnover_state["home_part_rotation"]
            held_tool_r, held_tool_p = actual_tool_pose(worker)
            held_part_p, held_part_r, _ = pose("p74_bracket")
            part_from_tool_p = held_tool_r.T @ (held_part_p - held_tool_p)
            part_from_tool_r = held_tool_r.T @ held_part_r
            desired_tool_r = desired_part_r @ part_from_tool_r.T
            desired_tool_p = desired_part_p - desired_tool_r @ part_from_tool_p
            move_pose(worker, desired_tool_p, desired_tool_r, 36)
            dock_corrections = []
            for _ in range(40):
                docked_p, docked_r, _ = pose("p74_bracket")
                position_error_vector = desired_part_p - docked_p
                angle_error = _rotation_error_deg(
                    docked_r, desired_part_r
                )
                dock_corrections.append(
                    {
                        "position_error_m": position_error_vector.tolist(),
                        "angle_error_deg": angle_error,
                    }
                )
                if (
                    np.linalg.norm(position_error_vector)
                    <= D.CONNECTOR_POSITION_TOLERANCE_M
                    and angle_error <= D.CONNECTOR_ANGLE_TOLERANCE_DEG
                ):
                    break
                current_tool_r, current_tool_p = actual_tool_pose(worker)
                full_corrected_r = (
                    desired_part_r @ docked_r.T
                ) @ current_tool_r
                rotation_fraction = min(0.5, 5.0 / max(angle_error, 1e-9))
                corrected_tool_r = _interpolate_rotation(
                    current_tool_r, full_corrected_r, rotation_fraction
                )
                correction = 0.5 * position_error_vector
                correction_norm = float(np.linalg.norm(correction))
                if correction_norm > 0.004:
                    correction *= 0.004 / correction_norm
                move_pose(
                    worker,
                    current_tool_p + correction,
                    corrected_tool_r,
                    6,
                    settle=False,
                )
                step(8, capture=not args.physics_only)
            docked_p, docked_r, _ = pose("p74_bracket")
            absolute_dock_position_error = float(
                np.linalg.norm(docked_p - desired_part_p)
            )
            absolute_dock_angle_error = _rotation_error_deg(
                docked_r, desired_part_r
            )
            if (
                absolute_dock_position_error
                > D.CONNECTOR_POSITION_TOLERANCE_M
                or absolute_dock_angle_error
                > D.CONNECTOR_ANGLE_TOLERANCE_DEG
            ):
                raise RuntimeError(
                    "absolute upright docking gate failed: "
                    f"{absolute_dock_position_error*1000:.3f} mm/"
                    f"{absolute_dock_angle_error:.3f} deg"
                )

            seed_lock = UsdPhysics.FixedJoint.Get(
                stage, "/World/LegoConnectorLocks/t00"
            )
            seed_lock.GetJointEnabledAttr().Set(False)
            seed_lock.GetLocalPos0Attr().Set(Gf.Vec3f(*docked_p))
            docked_q = _matrix_quat(docked_r)
            seed_lock.GetLocalRot0Attr().Set(
                Gf.Quatf(float(docked_q[0]), Gf.Vec3f(*docked_q[1:]))
            )
            seed_lock.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            seed_lock.GetLocalRot1Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            step(1, capture=not args.physics_only)
            seed_lock.GetJointEnabledAttr().Set(True)
            step(4, capture=not args.physics_only)
            locked_p, locked_r, _ = pose("p74_bracket")
            position_jump = float(np.linalg.norm(locked_p - docked_p))
            angle_jump = _rotation_error_deg(locked_r, docked_r)
            if position_jump > 0.0002 or angle_jump > 1.5:
                raise RuntimeError(
                    "turnover docking lock jump gate failed: "
                    f"{position_jump*1000:.3f} mm/{angle_jump:.3f} deg"
                )
            before_release_p, before_release_r, _ = pose("p74_bracket")
            lift_joint.GetJointEnabledAttr().Set(False)
            step(4, capture=not args.physics_only)
            after_release_p, after_release_r, _ = pose("p74_bracket")
            release_position_jump = float(
                np.linalg.norm(after_release_p - before_release_p)
            )
            release_angle_jump = _rotation_error_deg(
                after_release_r, before_release_r
            )
            if release_position_jump > 0.0002 or release_angle_jump > 1.5:
                raise RuntimeError(
                    "Arm A-to-world docking handoff jump gate failed: "
                    f"{release_position_jump*1000:.3f} mm/"
                    f"{release_angle_jump:.3f} deg"
                )
            ramp_jaw_only(worker, C.JAW_OPEN, samples=10)
            release_r, release_p = actual_tool_pose(worker)
            move_pose(
                worker,
                release_p + np.asarray((0.0, 0.0, 0.06)),
                release_r,
                10,
            )
            park_arm(worker, home_world["arm_a"])
            step(20, capture=not args.physics_only)
            settled_p, settled_r, _ = pose("p74_bracket")
            settled_position_error = float(
                np.linalg.norm(settled_p - desired_part_p)
            )
            settled_angle_error = _rotation_error_deg(
                settled_r, desired_part_r
            )
            if (
                settled_position_error > D.CONNECTOR_POSITION_TOLERANCE_M
                or settled_angle_error > D.CONNECTOR_ANGLE_TOLERANCE_DEG
            ):
                raise RuntimeError(
                    "post-release upright docking gate failed: "
                    f"{settled_position_error*1000:.3f} mm/"
                    f"{settled_angle_error:.3f} deg"
                )

            # Root registration alone cannot prove that an enlarged rigid
            # assembly survived docking intact: a locator can hold one child
            # against its edge while the robot correctly locates p74.  Check
            # every completed connector again in its mechanical parent frame.
            integrity_rows = []
            for completed_event in report["events"]:
                if not completed_event.get("placement_verified"):
                    continue
                completed_part_id = completed_event["part"]
                completed_task = task_by_part[completed_part_id]
                completed_part = part_by_id[completed_part_id]
                completed_target = completed_task["target"]
                completed_nominal_p = (
                    np.asarray(D.FIXTURE_DATUM)
                    + np.asarray(completed_target["xyz_mm"], float) / 1000
                    + profile_body_target_offset(completed_part)
                )
                completed_nominal_r = _rpy_matrix(
                    completed_target["rpy_deg"]
                )
                completed_parent_id = completed_task.get(
                    "connector_parent"
                )
                if completed_parent_id is None:
                    expected_p = completed_nominal_p
                    expected_r = completed_nominal_r
                    reference_frame = plan["fixture_frame"]
                else:
                    completed_parent_task = task_by_part[
                        completed_parent_id
                    ]
                    completed_parent_part = part_by_id[
                        completed_parent_id
                    ]
                    completed_parent_target = completed_parent_task["target"]
                    parent_nominal_p = (
                        np.asarray(D.FIXTURE_DATUM)
                        + np.asarray(
                            completed_parent_target["xyz_mm"], float
                        )
                        / 1000
                        + profile_body_target_offset(completed_parent_part)
                    )
                    parent_nominal_r = _rpy_matrix(
                        completed_parent_target["rpy_deg"]
                    )
                    child_from_parent_p = parent_nominal_r.T @ (
                        completed_nominal_p - parent_nominal_p
                    )
                    child_from_parent_r = (
                        parent_nominal_r.T @ completed_nominal_r
                    )
                    measured_parent_p, measured_parent_r, _ = pose(
                        completed_parent_id
                    )
                    expected_p = (
                        measured_parent_p
                        + measured_parent_r @ child_from_parent_p
                    )
                    expected_r = measured_parent_r @ child_from_parent_r
                    reference_frame = completed_parent_id
                measured_p, measured_r, _ = pose(completed_part_id)
                row = {
                    "task": completed_task["id"],
                    "part": completed_part_id,
                    "reference_frame": reference_frame,
                    "position_error_m": float(
                        np.linalg.norm(measured_p - expected_p)
                    ),
                    "angle_error_deg": _rotation_error_deg(
                        measured_r, expected_r
                    ),
                }
                row["passed"] = (
                    row["position_error_m"]
                    <= D.FINAL_POSITION_TOLERANCE_M
                    and row["angle_error_deg"]
                    <= D.FINAL_ANGLE_TOLERANCE_DEG
                )
                integrity_rows.append(row)
            integrity_passed = bool(integrity_rows) and all(
                row["passed"] for row in integrity_rows
            )
            report["post_dock_assembly_integrity"] = {
                "rigid_parts_checked": len(integrity_rows),
                "position_tolerance_m": D.FINAL_POSITION_TOLERANCE_M,
                "angle_tolerance_deg": D.FINAL_ANGLE_TOLERANCE_DEG,
                "rows": integrity_rows,
                "passed": integrity_passed,
            }
            if not integrity_passed:
                failed_rows = [
                    row for row in integrity_rows if not row["passed"]
                ]
                raise RuntimeError(
                    "post-dock rigid assembly integrity gate failed: "
                    f"{[(row['part'], round(row['position_error_m']*1000, 3), round(row['angle_error_deg'], 3)) for row in failed_rows]}"
                )
            turnover_state["active"] = False
            turnover_state["inverted"] = False
            turnover_state["worker_lift_joint_active"] = False
            report["fixture_transitions"].append(
                {
                    "action": "arm_b_to_arm_a_upright_handback_and_dock",
                    "arm_a_contacting_jaw_sides": sorted(contact_sides),
                    "arm_b_retreat_m": 0.08,
                    "arm_b_parked_after_release": True,
                    "arm_b_clear_park_tip_m": support_clear_park.tolist(),
                    "arm_b_clear_park_vertical_tool": True,
                    "hidden_part_pose_write": False,
                }
            )
            report["assembly_turnover"].update(
                {
                    "docked_upright": True,
                    "arm_a_dock_contacting_jaw_sides": sorted(contact_sides),
                    "arm_b_to_arm_a_position_jump_m": handback_position_jump,
                    "arm_b_to_arm_a_angle_jump_deg": handback_angle_jump,
                    "absolute_dock_position_error_m": (
                        absolute_dock_position_error
                    ),
                    "absolute_dock_angle_error_deg": absolute_dock_angle_error,
                    "absolute_dock_corrections": dock_corrections,
                    "dock_position_jump_m": position_jump,
                    "dock_angle_jump_deg": angle_jump,
                    "arm_a_to_world_position_jump_m": release_position_jump,
                    "arm_a_to_world_angle_jump_deg": release_angle_jump,
                    "post_release_position_error_m": settled_position_error,
                    "post_release_angle_error_deg": settled_angle_error,
                    "world_lock_reenabled_before_release": True,
                    "docking_arm": "arm_a_vertical",
                    "hidden_part_pose_write": False,
                }
            )

        for task in tasks:
            # Completed-task summaries already contain their needed evidence.
            # Rotate raw buffers so persistent old contacts cannot starve a
            # later connector of telemetry capacity.
            tool_contact_records.clear()
            assembly_contact_records.clear()
            event_contact_start = 0
            assembly_contact_start = 0
            part = part_by_id[task["part"]]
            profile = handling_profile(part, task)
            seating = task.get("seating")
            set_worker_drive_mode("contact")
            for action in task.get("before_actions", []):
                if action == "present_underside":
                    _acquire_and_present_underside()
                elif action == "restore_upright_hold":
                    _rotate_turnover_assembly(False)
                elif action == "restore_upright_and_dock":
                    _restore_upright_and_dock()
                elif action == "retract_fixture_base":
                    support_insert = stage.GetPrimAtPath("/World/LegoFixture/Base")
                    support_insert.GetAttribute("physics:collisionEnabled").Set(False)
                    report["fixture_transitions"].append(
                        {
                            "before_task": task["id"],
                            "action": action,
                            "collision_enabled": False,
                        }
                    )
                    step(2, capture=not args.physics_only)
                else:
                    raise RuntimeError(
                        f"unsupported before_action {action!r} for {task['id']}"
                    )
            event = {
                "task": task["id"], "step": part["step"], "part": part["id"],
                "design_id": part["design_id"], "pickup_verified": False,
                "placement_verified": False, "connector_lock_activated": False,
                "expected_mates": list(task.get("mates", [])),
            }
            report["events"].append(event)
            part_p, part_r, _ = pose(part["id"])
            event["measured_pose_before_pick"] = {
                "position_m": part_p.tolist(),
                "quat_wxyz": _matrix_quat(part_r).tolist(),
            }
            # The grasp direction is part-local, so the same profile works in
            # any declared keyed-feeder yaw. The pick height is measured from
            # the rigid part root instead of the global tray surface.
            grasp_center_world = part_r @ np.asarray(
                profile.get("grasp_center_part_m", (0.0, 0.0, 0.0)), float
            )
            pick_height_m = float(
                part["feeder"].get(
                    "pick_tip_from_part_root_z_mm",
                    profile["pick_tip_from_part_root_z_m"] * 1000,
                )
            ) / 1000
            pick_offset_world = part_r @ np.asarray(
                (0.0, 0.0, pick_height_m), float
            )
            pick_tip = part_p + grasp_center_world + pick_offset_world
            pick_spread = part_r @ np.asarray(profile["spread_part"], float)
            pick_spread[2] = 0.0
            pick_spread /= np.linalg.norm(pick_spread)
            # Parallel-jaw spread is an unoriented line: reversing it does not
            # change either contact face. Choose a deterministic hemisphere so
            # IK does not wind J6 by an unnecessary 180 degrees.
            dominant = int(np.argmax(np.abs(pick_spread)))
            if pick_spread[dominant] < 0:
                pick_spread = -pick_spread
            pick_r = np.column_stack((-np.array((0.0, 0.0, 1.0)), pick_spread, np.cross(-np.array((0.0, 0.0, 1.0)), pick_spread)))
            # column_stack above is [tool X, tool Y, tool Z] because local tool
            # and spread axes are +X/+Y.
            hover = pick_tip + np.array((0, 0, 0.07))
            move_pose(worker, hover, pick_r, 10)
            move_pose(worker, pick_tip, pick_r, 8, settle=False)
            # Rev A's free pad gap is 7 mm plus twice the per-jaw opening.
            # Command the nominal rigid-proxy width. The simulator pad has a
            # high-friction material but no fake geometric compression, so an
            # interference target would inject a continuous separating force.
            nominal_grip_width = profile["nominal_width_m"]
            grip_target = max(
                C.JAW_CLOSED,
                min(
                    C.JAW_OPEN,
                    (
                        nominal_grip_width
                        - D.WORKER_FINGER_CLOSED_PAD_GAP_M
                        - profile["interference_m"]
                    )
                    / 2,
                ),
            )
            event["grasp_profile"] = {
                "tool_asset_id": D.WORKER_FINGER_ASSET_ID,
                "profile": profile["name"],
                "mode": "pad_pinch",
                "nominal_part_width_m": nominal_grip_width,
                "commanded_per_jaw_opening_m": grip_target,
                "spread_part": list(profile["spread_part"]),
                "spread_world_measured": pick_spread.tolist(),
                "grasp_center_part_m": list(
                    profile.get("grasp_center_part_m", (0.0, 0.0, 0.0))
                ),
                "pick_tip_from_part_root_z_m": pick_height_m,
                "retention_mode": "physical_pad_contact_only",
                "keyed_feeder_close_dwell_steps": 12,
            }
            move_pose(
                worker, pick_tip, pick_r, 7, jaw=grip_target, settle=False
            )
            # Let both physical jaw drives reach the commanded pinch while the
            # keyed feeder still supports the part.  Releasing the rigid body
            # on the same step as the close command lets the first contacting
            # pad eject tall/inverted pieces before the opposite pad arrives.
            step(12, capture=not args.physics_only)
            stage.GetPrimAtPath("/World/LegoParts/" + part["id"]).GetAttribute("physics:kinematicEnabled").Set(False)
            step(20, capture=not args.physics_only)
            closed_p, closed_r, _ = pose(part["id"])
            actual_joints = worker["arm"].get_joint_positions()
            actual_left = float(actual_joints[worker["indices"]["jaw_l"]])
            actual_right = float(actual_joints[worker["indices"]["jaw_r"]])
            actual_per_side_opening = C.JAW_OPEN - (
                abs(actual_left) + abs(actual_right)
            ) / 2
            event["measured_pose_after_jaw_close"] = {
                "position_m": closed_p.tolist(),
                "quat_wxyz": _matrix_quat(closed_r).tolist(),
                "tool_tip_target_m": pick_tip.tolist(),
                "part_root_to_tip_m": (closed_p - pick_tip).tolist(),
                "actual_jaw_joint_positions_m": [actual_left, actual_right],
                "estimated_pad_gap_m": (
                    D.WORKER_FINGER_CLOSED_PAD_GAP_M
                    + 2 * actual_per_side_opening
                ),
            }
            pair_summary = {}
            for record in tool_contact_records[event_contact_start:]:
                if not any(
                    path.startswith(D.ARM_SPECS[0]["prim_path"] + "/Jaw")
                    for path in (record["collider0"], record["collider1"])
                ):
                    continue
                key = (record["collider0"], record["collider1"])
                pair_summary[key] = max(
                    pair_summary.get(key, 0.0), record["maximum_impulse_Ns"]
                )
            event["jaw_contact_pairs_through_close"] = [
                {
                    "collider0": pair[0],
                    "collider1": pair[1],
                    "maximum_impulse_Ns": impulse,
                }
                for pair, impulse in sorted(pair_summary.items())
            ]
            contacted_pad_sides = sorted(
                side
                for side in ("JawL", "JawR")
                if any(
                    (
                        f"/{side}/" in record["collider0"]
                        or (
                            side == "JawL"
                            and "/JawLClearance/" in record["collider0"]
                        )
                    )
                    and "finger_pad" in record["collider0"]
                    and record["collider1"].startswith(
                        "/World/LegoParts/" + part["id"]
                    )
                    for record in tool_contact_records[event_contact_start:]
                )
            )
            estimated_gap = event["measured_pose_after_jaw_close"][
                "estimated_pad_gap_m"
            ]
            closure_gap_tolerance = float(
                profile.get("closure_gap_tolerance_m", 0.0003)
            )
            event["physical_grasp_closure_gate"] = {
                "estimated_pad_gap_m": estimated_gap,
                "maximum_gap_m": nominal_grip_width + closure_gap_tolerance,
                "part_contacting_pad_sides": contacted_pad_sides,
                "passed": (
                    estimated_gap <= nominal_grip_width + closure_gap_tolerance
                    and contacted_pad_sides == ["JawL", "JawR"]
                ),
            }
            if not event["physical_grasp_closure_gate"]["passed"]:
                raise RuntimeError(
                    f"physical grasp closure gate failed for {part['id']}: "
                    f"gap={estimated_gap*1000:.2f} mm, "
                    f"pad sides={contacted_pad_sides}"
                )
            jaw_p, jaw_q = worker_grasp_body.get_world_pose()
            jaw_p = np.asarray(jaw_p, float)
            jaw_r = _quat_matrix(jaw_q)
            local_p = jaw_r.T @ (closed_p - jaw_p)
            local_r = jaw_r.T @ closed_r
            local_q = _matrix_quat(local_r)
            grasp_path = "/World/LegoGraspConstraints/" + task["id"]
            grasp_joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(grasp_path))
            grasp_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*local_p))
            grasp_joint.GetLocalRot0Attr().Set(Gf.Quatf(float(local_q[0]), Gf.Vec3f(*local_q[1:])))
            step(1, capture=not args.physics_only)
            if D.CONSTRAINT_ASSISTED_GRASP:
                grasp_joint.GetJointEnabledAttr().Set(True)
            grasp_retention_active = bool(D.CONSTRAINT_ASSISTED_GRASP)
            event["grasp_retention_constraint"] = {
                "path": grasp_path,
                "activated_after_closed_jaw_dwell": D.CONSTRAINT_ASSISTED_GRASP,
                "hidden_pose_write": False,
            }
            event["grasp_profile"]["retention_mode"] = (
                "constraint_assisted_after_closed_jaw_dwell"
                if D.CONSTRAINT_ASSISTED_GRASP
                else "physical_pad_contact_only"
            )
            step(4, capture=not args.physics_only)
            before_lift_z = pose(part["id"])[0][2]
            move_pose(worker, hover, pick_r, 10)
            lifted_p, lifted_r, _ = pose(part["id"])
            lift_m = float(lifted_p[2] - before_lift_z)
            event["lift_m"] = lift_m
            event["pickup_verified"] = lift_m > 0.025
            if not event["pickup_verified"]:
                raise RuntimeError(f"pickup gate failed for {part['id']}: lift {lift_m * 1000:.1f} mm")
            report["verified_pickups"] += 1
            event["grasp_retention_constraint"]["verified_lift_m"] = lift_m

            tool_r, tool_p = actual_tool_pose(worker)
            relative_p = tool_r.T @ (lifted_p - tool_p)
            relative_r = tool_r.T @ lifted_r
            target = task["target"]
            body_target_offset = profile_body_target_offset(part)
            event["connector_to_body_offset_m"] = body_target_offset.tolist()
            nominal_desired_p = (
                np.asarray(D.FIXTURE_DATUM)
                + np.asarray(target["xyz_mm"], float) / 1000
                + body_target_offset
            )
            nominal_desired_r = _rpy_matrix(target["rpy_deg"])
            parent_id = task.get("connector_parent")
            if parent_id is None:
                desired_p = nominal_desired_p
                desired_r = nominal_desired_r
                event["placement_reference"] = {
                    "frame": plan["fixture_frame"],
                    "measured_parent": None,
                }
            else:
                # LEGO placement is fundamentally relative to the piece that
                # is already in the hand-built stack.  Referencing the nominal
                # fixture pose here would turn a harmless seed offset into an
                # artificial axial gap (or interference) at every later mate.
                parent_task = task_by_part[parent_id]
                parent_target = parent_task["target"]
                parent_nominal_p = (
                    np.asarray(D.FIXTURE_DATUM)
                    + np.asarray(parent_target["xyz_mm"], float) / 1000
                    + profile_body_target_offset(part_by_id[parent_id])
                )
                parent_nominal_r = _rpy_matrix(parent_target["rpy_deg"])
                child_from_parent_p = parent_nominal_r.T @ (
                    nominal_desired_p - parent_nominal_p
                )
                child_from_parent_r = parent_nominal_r.T @ nominal_desired_r
                measured_parent_p, measured_parent_r, _ = pose(parent_id)
                desired_p = (
                    measured_parent_p + measured_parent_r @ child_from_parent_p
                )
                desired_r = measured_parent_r @ child_from_parent_r
                event["placement_reference"] = {
                    "frame": parent_id,
                    "measured_parent": True,
                    "nominal_fixture_position_m": nominal_desired_p.tolist(),
                    "parent_position_m": measured_parent_p.tolist(),
                    "child_from_parent_position_m": child_from_parent_p.tolist(),
                }
            gate_p = desired_p + np.asarray(
                profile.get("pose_gate_offset_m", (0.0, 0.0, 0.0)), float
            )
            event["pose_gate_origin_offset_m"] = (gate_p - desired_p).tolist()
            approach_axis_world = desired_r @ np.asarray(
                profile.get("approach_axis_part", (0.0, 0.0, 1.0)), float
            )
            approach_axis_world /= np.linalg.norm(approach_axis_world)
            event["assembly_approach_axis_world"] = approach_axis_world.tolist()
            desired_tool_r = desired_r @ relative_r.T
            desired_tool_p = desired_p - desired_tool_r @ relative_p
            carry_clearance_m = float(profile.get("carry_clearance_m", 0.055))
            approach_tool_p = (
                desired_tool_p + approach_axis_world * carry_clearance_m
            )
            event["carry_clearance_m"] = carry_clearance_m
            carry_drive_mode = profile.get("carry_drive_mode", "contact")
            carry_position_tolerance = float(
                profile.get(
                    "carry_tracking_position_tolerance_m",
                    TRACKING_POSITION_TOLERANCE_M,
                )
            )
            carry_angle_tolerance = float(
                profile.get(
                    "carry_tracking_angle_tolerance_deg",
                    TRACKING_ANGLE_TOLERANCE_DEG,
                )
            )
            event["free_space_carry_drive_mode"] = carry_drive_mode
            set_worker_drive_mode(
                "free_space_carry"
                if carry_drive_mode == "payload_compensated"
                else "contact"
            )
            move_pose(
                worker,
                approach_tool_p,
                desired_tool_r,
                30,
                position_tolerance_m=carry_position_tolerance,
                angle_tolerance_deg=carry_angle_tolerance,
            )
            carry_mode_tool_r, carry_mode_tool_p = actual_tool_pose(worker)
            placement_drive_mode = (
                "seating"
                if seating
                else (
                    "free_space_carry"
                    if carry_drive_mode == "payload_compensated"
                    else "contact"
                )
            )
            set_worker_drive_mode(placement_drive_mode)
            event["placement_drive_mode"] = placement_drive_mode
            # Settle again at the same target after selecting placement gains;
            # only then is the carried-part transform authoritative.
            move_pose(
                worker,
                approach_tool_p,
                desired_tool_r,
                1,
                position_tolerance_m=carry_position_tolerance,
                angle_tolerance_deg=carry_angle_tolerance,
            )
            contact_mode_tool_r, contact_mode_tool_p = actual_tool_pose(worker)
            event["carry_to_contact_drive_transition"] = {
                "position_change_m": float(
                    np.linalg.norm(contact_mode_tool_p - carry_mode_tool_p)
                ),
                "angle_change_deg": _rotation_error_deg(
                    contact_mode_tool_r, carry_mode_tool_r
                ),
                "settled_at_same_target": True,
            }
            # Re-measure after the long carry/rotation.  Small parts can seat
            # differently in compliant jaws during that physical motion; the
            # final descent must use the current grasp transform, not the one
            # observed immediately after pickup.
            carried_p, carried_r, _ = pose(part["id"])
            event["measured_pose_after_carry"] = {
                "position_m": carried_p.tolist(),
                "quat_wxyz": _matrix_quat(carried_r).tolist(),
            }
            descent_samples = int(profile.get("descent_samples", 10))
            approach_stages = tuple(
                float(value)
                for value in profile.get("closed_loop_approach_stages_m", (0.012,))
            )
            approach_records = []
            for stage_clearance in approach_stages:
                # Human-like visual/proprioceptive correction: remeasure the
                # compliant in-jaw transform after every short descent instead
                # of trusting one open-loop 43 mm move.
                carried_p, carried_r, _ = pose(part["id"])
                tool_r, tool_p = actual_tool_pose(worker)
                relative_p = tool_r.T @ (carried_p - tool_p)
                relative_r = tool_r.T @ carried_r
                desired_tool_r = desired_r @ relative_r.T
                stage_part_p = desired_p + approach_axis_world * stage_clearance
                stage_tool_p = stage_part_p - desired_tool_r @ relative_p
                move_pose(
                    worker,
                    stage_tool_p,
                    desired_tool_r,
                    descent_samples,
                    position_tolerance_m=carry_position_tolerance,
                    angle_tolerance_deg=carry_angle_tolerance,
                )
                measured_stage_p, measured_stage_r, _ = pose(part["id"])
                approach_records.append(
                    {
                        "commanded_clearance_m": stage_clearance,
                        "position_error_m": float(
                            np.linalg.norm(measured_stage_p - stage_part_p)
                        ),
                        "angle_error_deg": _rotation_error_deg(
                            measured_stage_r, desired_r
                        ),
                    }
                )
            event["closed_loop_approach_stages"] = approach_records
            carried_p, carried_r, _ = pose(part["id"])
            event["measured_pose_at_precontact"] = {
                "position_m": carried_p.tolist(),
                "quat_wxyz": _matrix_quat(carried_r).tolist(),
            }
            filter_candidates = task.get("collision_filter_parts", [])
            if filter_candidates:
                raise RuntimeError(
                    f"rigid LEGO collisions may not be disabled: {task['id']}"
                )
            preinsert_clearance_m = float(
                profile.get("preinsert_clearance_m", 0.012)
            )
            preinsert_target_p = (
                gate_p + approach_axis_world * preinsert_clearance_m
            )
            preinsert_position_error = float(
                np.linalg.norm(carried_p - preinsert_target_p)
            )
            preinsert_angle_error = _rotation_error_deg(carried_r, desired_r)
            preinsert_position_tolerance = float(
                profile.get(
                    "preinsert_position_tolerance_m",
                    D.PREINSERT_POSITION_TOLERANCE_M,
                )
            )
            preinsert_passed = (
                preinsert_position_error <= preinsert_position_tolerance
                and preinsert_angle_error <= D.PREINSERT_ANGLE_TOLERANCE_DEG
            )
            event["preinsert_gate"] = {
                "position_error_m": preinsert_position_error,
                "angle_error_deg": preinsert_angle_error,
                "position_tolerance_m": preinsert_position_tolerance,
                "angle_tolerance_deg": D.PREINSERT_ANGLE_TOLERANCE_DEG,
                "passed": preinsert_passed,
            }
            if not preinsert_passed:
                raise RuntimeError(
                    f"preinsert gate failed for {part['id']}: "
                    f"{preinsert_position_error * 1000:.2f} mm, "
                    f"{preinsert_angle_error:.2f} deg"
                )
            event["assembly_collision_filters"] = []
            event["mating_proxy_filter_activation"] = {
                "stage": "never",
                "reason": "compound rigid shells model stud/socket clearance",
                "pairs": [],
                "hidden_pose_write": False,
            }
            seat_path = None
            if seating:
                guide_path = f"/World/LegoConnectorGuides/{task['id']}"
                seat_path = guide_path + "/Seat"
                seat_prim = stage.GetPrimAtPath(seat_path)
                if not seat_prim.IsValid():
                    raise RuntimeError(f"missing declared connector seat: {seat_path}")
                event["connector_seat"] = {
                    "path": seat_path,
                    "guide_path": guide_path,
                    "role": "mechanical_stud_clearance_proxy",
                    "armed_after_preinsert_gate": True,
                    "filtered_against": list(event["assembly_collision_filters"]),
                    "hidden_pose_write": False,
                }
            tool_r, tool_p = actual_tool_pose(worker)
            relative_p = tool_r.T @ (carried_p - tool_p)
            relative_r = tool_r.T @ carried_r
            desired_tool_r = desired_r @ relative_r.T
            desired_tool_p = desired_p - desired_tool_r @ relative_p
            placement_bias = np.asarray(profile["placement_bias_m"], float)
            event["precontact_robot_bias_m"] = placement_bias.tolist()
            desired_tool_p = desired_tool_p + placement_bias
            contact_tool_p = desired_tool_p
            if seating:
                press_travel = float(seating["press_travel_mm"]) / 1000
                insertion_axis = desired_tool_r @ np.asarray(C.TOOL_AXIS, float)
                insertion_axis /= np.linalg.norm(insertion_axis)
                desired_press_normal = desired_r @ np.asarray(
                    part["press_normal_part"], float
                )
                alignment_cosine = float(np.dot(insertion_axis, desired_press_normal))
                if alignment_cosine > -math.cos(
                    math.radians(float(seating["alignment_tolerance_deg"]))
                ):
                    raise RuntimeError(
                        f"plunger alignment gate failed for {part['id']}: "
                        f"dot={alignment_cosine:.6f}"
                    )
                # Hold the part one declared press stroke above its target.
                # The wrist may align the part, but only the plunger can drive
                # the remaining axial seating motion onto the physical stop.
                contact_tool_p = desired_tool_p - insertion_axis * press_travel
                event["plunger_standoff"] = {
                    "insertion_axis_world": insertion_axis.tolist(),
                    "desired_press_normal_world": desired_press_normal.tolist(),
                    "alignment_cosine": alignment_cosine,
                    "axial_standoff_m": press_travel,
                }
            move_pose(
                worker,
                contact_tool_p,
                desired_tool_r,
                int(profile.get("contact_samples", 10)),
                position_tolerance_m=float(
                    profile.get(
                        "contact_tracking_position_tolerance_m",
                        TRACKING_POSITION_TOLERANCE_M,
                    )
                ),
            )
            contact_p, contact_r, _ = pose(part["id"])
            event["measured_pose_at_contact"] = {
                "position_m": contact_p.tolist(),
                "quat_wxyz": _matrix_quat(contact_r).tolist(),
            }
            # The tracked contact move has already held its terminal waypoint.
            # Evaluate that measured pose immediately: additional closed-pad
            # dwell can lever a 3.2 mm tile out of a valid capture envelope.
            dwell_p, dwell_r, _ = pose(part["id"])
            event["measured_pose_after_contact_dwell"] = {
                "position_m": dwell_p.tolist(),
                "quat_wxyz": _matrix_quat(dwell_r).tolist(),
            }
            standoff_pairs = {}
            for record in tool_contact_records[event_contact_start:]:
                key = (record["collider0"], record["collider1"])
                standoff_pairs[key] = max(
                    standoff_pairs.get(key, 0.0), record["maximum_impulse_Ns"]
                )
            event["tool_contact_pairs_through_standoff"] = [
                {
                    "collider0": pair[0],
                    "collider1": pair[1],
                    "maximum_impulse_Ns": impulse,
                }
                for pair, impulse in sorted(standoff_pairs.items())
            ]
            neighbor_slide_retracted = False
            allowed_neighbor_prefixes = tuple(
                "/World/LegoParts/" + allowed_id
                for allowed_id in task.get("allowed_contacts", [])
            )
            clearance_prefix = (
                D.ARM_SPECS[0]["prim_path"] + "/JawLClearance/"
            )
            neighbor_clearance_contacts = [
                {
                    "collider0": pair[0],
                    "collider1": pair[1],
                    "maximum_impulse_Ns": impulse,
                }
                for pair, impulse in sorted(standoff_pairs.items())
                if (
                    pair[0].startswith(clearance_prefix)
                    and pair[1].startswith(allowed_neighbor_prefixes)
                    or pair[1].startswith(clearance_prefix)
                    and pair[0].startswith(allowed_neighbor_prefixes)
                )
            ]
            if not seating and neighbor_clearance_contacts:
                # Human fingers yield or lift the obstructed side when working
                # beside an already seated neighbor. Rev C provides the same
                # behavior explicitly: retract only JawL after bilateral
                # pickup, while the measured grasp constraint retains the
                # carried part. This decision is contact-driven and applies to
                # any declared allowed neighbor, not to a task identifier.
                before_neighbor_slide_p, before_neighbor_slide_r, _ = pose(
                    part["id"]
                )
                ramp_clearance_slide(
                    worker, CLEARANCE_SLIDE_STROKE_M, samples=30
                )
                measured_joints = worker["arm"].get_joint_positions()
                measured_neighbor_slide = float(
                    measured_joints[worker["indices"]["JawL_retract"]]
                )
                after_neighbor_slide_p, after_neighbor_slide_r, _ = pose(
                    part["id"]
                )
                neighbor_slide_position_change = float(
                    np.linalg.norm(
                        after_neighbor_slide_p - before_neighbor_slide_p
                    )
                )
                neighbor_slide_angle_change = _rotation_error_deg(
                    after_neighbor_slide_r, before_neighbor_slide_r
                )
                neighbor_slide_passed = (
                    abs(
                        measured_neighbor_slide - CLEARANCE_SLIDE_STROKE_M
                    )
                    <= 0.0002
                    and neighbor_slide_position_change
                    <= D.FINAL_POSITION_TOLERANCE_M
                    and neighbor_slide_angle_change
                    <= D.FINAL_ANGLE_TOLERANCE_DEG
                )
                event["reactive_neighbor_clearance_slide"] = {
                    "trigger": "physical_allowed_neighbor_contact",
                    "contacts": neighbor_clearance_contacts,
                    "commanded_position_m": CLEARANCE_SLIDE_STROKE_M,
                    "measured_position_m": measured_neighbor_slide,
                    "part_position_change_m": (
                        neighbor_slide_position_change
                    ),
                    "part_angle_change_deg": neighbor_slide_angle_change,
                    "grasp_constraint_retained": grasp_retention_active,
                    "passed": neighbor_slide_passed,
                }
                if not neighbor_slide_passed:
                    raise RuntimeError(
                        "reactive neighbor clearance slide gate failed for "
                        f"{part['id']}"
                    )
                neighbor_slide_retracted = True
            if seating:
                # Do not let a broad seat edge perturb the final free-space
                # alignment move. Enable it only once the measured wrist has
                # reached the declared axial standoff.
                enabled_guide_colliders = set_connector_guide(task["id"], True)
                event["connector_seat"]["enabled_at_plunger_standoff"] = True
                event["connector_seat"]["enabled_colliders"] = enabled_guide_colliders
                step(1, capture=not args.physics_only)
                if D.CONSTRAINT_ASSISTED_GRASP:
                    # The constraint stabilizes only the long carry. Transfer
                    # back to the still-closed physical pads at the standoff,
                    # before jaw relaxation or any plunger load is applied.
                    grasp_joint.GetJointEnabledAttr().Set(False)
                    grasp_retention_active = False
                    event["grasp_retention_constraint"][
                        "disabled_at_standoff_with_physical_grip"
                    ] = True
                    step(12, capture=not args.physics_only)

            corrections = []
            connector_position_tolerance = min(
                profile_connector_position_tolerance(),
                DEFAULT_INSERTION_CONFIG.accepted_position_error_m,
            )
            connector_angle_tolerance = min(
                D.CONNECTOR_ANGLE_TOLERANCE_DEG,
                DEFAULT_INSERTION_CONFIG.accepted_angle_error_deg,
            )
            if seating:
                # Latch the measured wrist at the physical support handoff.
                # Without this, jaw/slide-only steps keep commanding the stale
                # ideal IK target and can pull the plunger away from the part
                # while the nest correctly holds the part stationary.
                wrist_before_hold_r, wrist_before_hold_p = actual_tool_pose(worker)
                measured_arm = worker["arm"].get_joint_positions()
                worker["q"] = np.asarray(
                    measured_arm[worker["joint_indices"]], float
                ).copy()
                step(2, capture=not args.physics_only)
                wrist_after_hold_r, wrist_after_hold_p = actual_tool_pose(worker)
                event["seating_measured_wrist_hold"] = {
                    "latched_joint_deg": np.rad2deg(worker["q"]).tolist(),
                    "tool_position_change_m": float(
                        np.linalg.norm(wrist_after_hold_p - wrist_before_hold_p)
                    ),
                    "tool_angle_change_deg": _rotation_error_deg(
                        wrist_after_hold_r, wrist_before_hold_r
                    ),
                    "hidden_pose_write": False,
                }
                # Remove squeeze before pressing: clearance is the total pad
                # gap beyond the nominal rigid width, not a per-jaw increment.
                seating_gap = (
                    nominal_grip_width
                    + float(seating["jaw_clearance_mm"]) / 1000
                )
                seating_jaw = max(
                    C.JAW_CLOSED,
                    min(
                        C.JAW_OPEN,
                        (
                            seating_gap
                            - D.WORKER_FINGER_CLOSED_PAD_GAP_M
                        )
                        / 2,
                    ),
                )
                jaw_start = worker["jaw"]
                for jaw_step in range(10):
                    fraction = _ease((jaw_step + 1) / 10)
                    worker["jaw"] = jaw_start + (seating_jaw - jaw_start) * fraction
                    step(2, capture=not args.physics_only)
                measured_joints = worker["arm"].get_joint_positions()
                measured_left = float(measured_joints[worker["indices"]["jaw_l"]])
                measured_right = float(measured_joints[worker["indices"]["jaw_r"]])
                measured_per_side = C.JAW_OPEN - (
                    abs(measured_left) + abs(measured_right)
                ) / 2
                event["seating_jaw_relaxation"] = {
                    "commanded_per_jaw_opening_m": seating_jaw,
                    "commanded_pad_gap_m": seating_gap,
                    "measured_joint_positions_m": [measured_left, measured_right],
                    "estimated_measured_pad_gap_m": (
                        D.WORKER_FINGER_CLOSED_PAD_GAP_M
                        + 2 * measured_per_side
                    ),
                }

                # Rev C removes the obstructing JawL insert only after the
                # part has reached the physical seat/standoff. This preserves
                # the bilateral pickup while clearing the neighboring plate
                # before the force-limited plunger begins its seating stroke.
                before_prepress_slide_p, before_prepress_slide_r, _ = pose(
                    part["id"]
                )
                ramp_clearance_slide(
                    worker, CLEARANCE_SLIDE_STROKE_M, samples=30
                )
                measured_joints = worker["arm"].get_joint_positions()
                measured_prepress_slide = float(
                    measured_joints[worker["indices"]["JawL_retract"]]
                )
                after_prepress_slide_p, after_prepress_slide_r, _ = pose(
                    part["id"]
                )
                prepress_slide_position_change = float(
                    np.linalg.norm(
                        after_prepress_slide_p - before_prepress_slide_p
                    )
                )
                prepress_slide_angle_change = _rotation_error_deg(
                    after_prepress_slide_r, before_prepress_slide_r
                )
                prepress_slide_passed = (
                    abs(
                        measured_prepress_slide - CLEARANCE_SLIDE_STROKE_M
                    )
                    <= 0.0002
                    and prepress_slide_position_change
                    <= D.PREINSERT_POSITION_TOLERANCE_M
                    and prepress_slide_angle_change
                    <= D.PREINSERT_ANGLE_TOLERANCE_DEG
                )
                event["prepress_clearance_slide"] = {
                    "commanded_position_m": CLEARANCE_SLIDE_STROKE_M,
                    "measured_position_m": measured_prepress_slide,
                    "part_position_change_m": prepress_slide_position_change,
                    "part_angle_change_deg": prepress_slide_angle_change,
                    "position_tolerance_m": D.PREINSERT_POSITION_TOLERANCE_M,
                    "angle_tolerance_deg": D.PREINSERT_ANGLE_TOLERANCE_DEG,
                    "seat_enabled": True,
                    "passed": prepress_slide_passed,
                }
                if not prepress_slide_passed:
                    raise RuntimeError(
                        f"prepress clearance slide gate failed for {part['id']}"
                    )

                actual_p, actual_r, _ = pose(part["id"])
                tool_r, tool_p = actual_tool_pose(worker)
                plunger_axis = tool_r @ np.asarray(C.TOOL_AXIS, float)
                plunger_axis /= np.linalg.norm(plunger_axis)
                face_retracted = (
                    tool_p
                    - PLUNGER_FACE_RETRACTED_FROM_TIP_M * plunger_axis
                )
                press_point = actual_p + actual_r @ (
                    np.asarray(part["press_point_part_mm"], float) / 1000
                )
                press_normal = actual_r @ np.asarray(
                    part["press_normal_part"], float
                )
                denominator = float(np.dot(plunger_axis, press_normal))
                if denominator >= -0.5:
                    raise RuntimeError(
                        f"plunger/press normals oppose insufficiently for {part['id']}"
                    )
                predicted_contact = float(
                    np.dot(press_point - face_retracted, press_normal)
                    / denominator
                )
                face_at_contact = face_retracted + plunger_axis * predicted_contact
                residual = press_point - face_at_contact
                lateral_vector = (
                    residual - press_normal * np.dot(residual, press_normal)
                )
                lateral_error = float(np.linalg.norm(lateral_vector))
                # Supplied flat shoe pad is rectangular: 5 mm across tool Y
                # and 6 mm across tool Z. Test its actual axes instead of an
                # unnecessarily restrictive 2.5 mm inscribed circle.
                lateral_tool_y = abs(float(np.dot(lateral_vector, tool_r[:, 1])))
                lateral_tool_z = abs(float(np.dot(lateral_vector, tool_r[:, 2])))
                target_extension = predicted_contact + press_travel
                if (
                    predicted_contact < 0
                    or target_extension > PLUNGER_STROKE_M
                    or lateral_tool_y > 0.0025
                    or lateral_tool_z > 0.0030
                ):
                    raise RuntimeError(
                        f"plunger geometry gate failed for {part['id']}: "
                        f"contact={predicted_contact*1000:.2f} mm, "
                        f"target={target_extension*1000:.2f} mm, "
                        f"lateral_yz=({lateral_tool_y*1000:.2f}, "
                        f"{lateral_tool_z*1000:.2f}) mm"
                    )

                connector_position_tolerance = (
                    float(seating["position_tolerance_mm"]) / 1000
                )
                connector_angle_tolerance = float(seating["angle_tolerance_deg"])
                press_samples = []
                passed_at_extension = None
                contact_onset_sample = None
                event["plunger_seating"] = {
                    "tool_asset_id": D.WORKER_FINGER_ASSET_ID,
                    "shoe": seating["shoe"],
                    "predicted_contact_extension_m": predicted_contact,
                    "target_extension_m": target_extension,
                    "lateral_error_m": lateral_error,
                    "lateral_tool_y_m": lateral_tool_y,
                    "lateral_tool_z_m": lateral_tool_z,
                    "maximum_force_N": PLUNGER_MAX_FORCE_N,
                    "force_source": "simulated_joint_effort_if_available",
                    "hardware_force_verified": False,
                    "samples": press_samples,
                    "passed": False,
                }
                event["virtual_pressure_sensor"] = {
                    "source": "simulated_plunger_joint_effort_and_contact",
                    "hardware_calibrated": False,
                    "contact_onset_sample": None,
                    "maximum_force_N": PLUNGER_MAX_FORCE_N,
                }
                press_steps = max(
                    1, int(math.ceil(target_extension / 0.0002))
                )
                for press_step in range(press_steps):
                    worker["plunger"] = target_extension * (
                        press_step + 1
                    ) / press_steps
                    step(4, capture=not args.physics_only)
                    measured = worker["arm"].get_joint_positions()
                    measured_extension = float(
                        measured[worker["indices"]["plunger"]]
                    )
                    measured_effort = None
                    try:
                        efforts = worker["arm"].get_measured_joint_efforts()
                        if efforts is not None:
                            measured_effort = abs(
                                float(efforts[worker["indices"]["plunger"]])
                            )
                    except Exception:
                        measured_effort = None
                    pressed_p, pressed_r, _ = pose(part["id"])
                    sample_position_error = float(
                        np.linalg.norm(pressed_p - gate_p)
                    )
                    sample_angle_error = _rotation_error_deg(
                        pressed_r, desired_r
                    )
                    physical_press_contact = any(
                        (
                            record["collider0"].startswith(
                                D.ARM_SPECS[0]["prim_path"] + "/Plunger"
                            )
                            and record["collider1"].startswith(
                                "/World/LegoParts/" + part["id"]
                            )
                            and record["maximum_impulse_Ns"] > 0
                        )
                        or (
                            record["collider1"].startswith(
                                D.ARM_SPECS[0]["prim_path"] + "/Plunger"
                            )
                            and record["collider0"].startswith(
                                "/World/LegoParts/" + part["id"]
                            )
                            and record["maximum_impulse_Ns"] > 0
                        )
                        for record in tool_contact_records[event_contact_start:]
                    )
                    if physical_press_contact and contact_onset_sample is None:
                        contact_onset_sample = press_step
                        event["virtual_pressure_sensor"][
                            "contact_onset_sample"
                        ] = press_step
                        event["virtual_pressure_sensor"][
                            "contact_onset_extension_m"
                        ] = measured_extension
                    press_samples.append(
                        {
                            "commanded_extension_m": worker["plunger"],
                            "measured_extension_m": measured_extension,
                            "measured_effort_N": measured_effort,
                            "position_error_m": sample_position_error,
                            "angle_error_deg": sample_angle_error,
                            "physical_part_contact": physical_press_contact,
                        }
                    )
                    if (
                        measured_effort is not None
                        and measured_effort > PLUNGER_MAX_FORCE_N + 0.1
                    ):
                        raise RuntimeError(
                            f"plunger force cap exceeded for {part['id']}: "
                            f"{measured_effort:.2f} N"
                        )
                    if (
                        physical_press_contact
                        and sample_position_error <= connector_position_tolerance
                        and sample_angle_error <= connector_angle_tolerance
                    ):
                        passed_at_extension = worker["plunger"]
                        break

                valid_efforts = [
                    sample["measured_effort_N"]
                    for sample in press_samples
                    if sample["measured_effort_N"] is not None
                ]
                force_slopes = []
                for previous, current in zip(press_samples, press_samples[1:]):
                    if (
                        previous["measured_effort_N"] is None
                        or current["measured_effort_N"] is None
                    ):
                        continue
                    displacement = (
                        current["measured_extension_m"]
                        - previous["measured_extension_m"]
                    )
                    if displacement > 1.0e-6:
                        force_slopes.append(
                            (
                                current["measured_effort_N"]
                                - previous["measured_effort_N"]
                            )
                            / displacement
                        )
                event["virtual_pressure_sensor"].update(
                    {
                        "sample_count": len(press_samples),
                        "peak_force_N": max(valid_efforts) if valid_efforts else None,
                        "peak_positive_stiffness_N_per_m": (
                            max(force_slopes) if force_slopes else None
                        ),
                        "pose_gate_reached": passed_at_extension is not None,
                    }
                )
                if passed_at_extension is None:
                    event["plunger_seating"]["stop_reason"] = (
                        "strict_pose_gate_not_reached"
                    )
                    raise RuntimeError(
                        f"plunger seating gate failed for {part['id']} at "
                        f"{worker['plunger']*1000:.2f} mm extension"
                    )
                hold_samples = []
                for _ in range(int(seating["hold_steps"])):
                    step(1, capture=not args.physics_only)
                    held_p, held_r, _ = pose(part["id"])
                    held_position_error = float(np.linalg.norm(held_p - gate_p))
                    held_angle_error = _rotation_error_deg(held_r, desired_r)
                    hold_samples.append(
                        {
                            "position_error_m": held_position_error,
                            "angle_error_deg": held_angle_error,
                        }
                    )
                    if (
                        held_position_error > connector_position_tolerance
                        or held_angle_error > connector_angle_tolerance
                    ):
                        raise RuntimeError(
                            f"plunger seating hold gate failed for {part['id']}"
                        )
                event["virtual_pressure_sensor"]["stable_seated_decision"] = True
                event["virtual_pressure_sensor"]["stable_hold_samples"] = len(
                    hold_samples
                )
                if grasp_retention_active:
                    safe_extension = float(worker["plunger"])
                    unload_trials = []
                    while safe_extension > 0.0002:
                        candidate_extension = max(0.0, safe_extension - 0.0002)
                        worker["plunger"] = candidate_extension
                        step(2, capture=not args.physics_only)
                        candidate_p, candidate_r, _ = pose(part["id"])
                        candidate_position_error = float(
                            np.linalg.norm(candidate_p - gate_p)
                        )
                        candidate_angle_error = _rotation_error_deg(
                            candidate_r, desired_r
                        )
                        candidate_passed = (
                            candidate_position_error
                            <= connector_position_tolerance
                            and candidate_angle_error
                            <= connector_angle_tolerance
                        )
                        unload_trials.append(
                            {
                                "commanded_extension_m": candidate_extension,
                                "position_error_m": candidate_position_error,
                                "angle_error_deg": candidate_angle_error,
                                "strict_pose_gate_retained": candidate_passed,
                            }
                        )
                        if not candidate_passed:
                            worker["plunger"] = safe_extension
                            step(2, capture=not args.physics_only)
                            break
                        safe_extension = candidate_extension
                    event["virtual_pressure_sensor"][
                        "minimum_load_prelock_unload"
                    ] = {
                        "retained_extension_m": safe_extension,
                        "trials": unload_trials,
                        "strict_pose_gate_retained": True,
                    }
                plunger_pairs = {}
                for record in tool_contact_records[event_contact_start:]:
                    if not any(
                        path.startswith(D.ARM_SPECS[0]["prim_path"] + "/Plunger")
                        for path in (record["collider0"], record["collider1"])
                    ):
                        continue
                    key = (record["collider0"], record["collider1"])
                    plunger_pairs[key] = max(
                        plunger_pairs.get(key, 0.0),
                        record["maximum_impulse_Ns"],
                    )
                event["plunger_seating"].update(
                    {
                        "passed_at_commanded_extension_m": passed_at_extension,
                        "hold_samples": hold_samples,
                        "contact_pairs": [
                        {
                            "collider0": pair[0],
                            "collider1": pair[1],
                            "maximum_impulse_Ns": impulse,
                        }
                        for pair, impulse in sorted(plunger_pairs.items())
                        ],
                        "stop_reason": "strict_pose_gate_held",
                        "passed": True,
                    }
                )
            else:
                # Contact and jaw compliance can shift a millimetre-scale part
                # in the grasp. Servo only the robot from measured part pose;
                # no object transform is ever written.
                for _ in range(40):
                    actual_p, actual_r, actual_q = pose(part["id"])
                    position_error_vector = gate_p - actual_p
                    angle_error = _rotation_error_deg(actual_r, desired_r)
                    corrections.append(
                        {
                            "position_error_m": position_error_vector.tolist(),
                            "angle_error_deg": angle_error,
                        }
                    )
                    if (
                        np.linalg.norm(position_error_vector)
                        <= connector_position_tolerance
                        and angle_error <= connector_angle_tolerance
                    ):
                        break
                    current_tool_r, current_tool_p_local = fk.tool_pose(worker["q"], tool)
                    current_tool_p = current_tool_p_local + np.asarray(worker["spec"]["base_position"])
                    full_corrected_r = (desired_r @ actual_r.T) @ current_tool_r
                    rotation_fraction = min(0.5, 5.0 / max(angle_error, 1e-9))
                    corrected_tool_r = _interpolate_rotation(current_tool_r, full_corrected_r, rotation_fraction)
                    correction = 0.5 * position_error_vector
                    correction_norm = float(np.linalg.norm(correction))
                    if correction_norm > 0.004:
                        correction *= 0.004 / correction_norm
                    corrected_tool_p = current_tool_p + correction
                    move_pose(
                        worker, corrected_tool_p, corrected_tool_r, 6, settle=False
                    )
                    step(8, capture=not args.physics_only)

                # The connector tolerance is a safety/acceptance envelope, not
                # a signal that the LEGO pieces are physically seated.  Once
                # coarse alignment is inside that envelope, reproduce the
                # human "press until felt" behavior with a bounded axial seek.
                # The robot advances in 50 um increments and stops only after
                # every declared mate has produced sustained positive PhysX
                # impulse.  No part transform is written and collisions stay
                # enabled throughout.
                expected_mates_for_seek = list(task.get("mates", []))
                if expected_mates_for_seek:
                    seek_hold_r, _ = actual_tool_pose(worker)
                    seek_start_step = physics_step
                    seek_increment_m = 0.00005
                    seek_max_travel_m = 0.00050
                    seek_travel_m = 0.0
                    seek_samples = []
                    child_prefix_for_seek = "/World/LegoParts/" + part["id"]

                    def seek_contact_frames(mate_id):
                        mate_prefix = "/World/LegoParts/" + mate_id
                        frames = {
                            record["physics_step"]
                            for record in assembly_contact_records[
                                assembly_contact_start:
                            ]
                            if record["physics_step"] >= seek_start_step
                            and (
                                record["collider0"].startswith(
                                    child_prefix_for_seek
                                )
                                and record["collider1"].startswith(mate_prefix)
                                or record["collider1"].startswith(
                                    child_prefix_for_seek
                                )
                                and record["collider0"].startswith(mate_prefix)
                            )
                        }
                        return sorted(frames)

                    def has_consecutive_contact(frames, count=2):
                        return any(
                            all(frames[index + offset] == frames[index] + offset
                                for offset in range(count))
                            for index in range(len(frames) - count + 1)
                        )

                    while seek_travel_m < seek_max_travel_m - 1.0e-12:
                        contact_frames_by_mate = {
                            mate_id: seek_contact_frames(mate_id)
                            for mate_id in expected_mates_for_seek
                        }
                        if all(
                            has_consecutive_contact(frames)
                            for frames in contact_frames_by_mate.values()
                        ):
                            break
                        current_tool_r, current_tool_p = actual_tool_pose(worker)
                        seek_tool_p = (
                            current_tool_p
                            - approach_axis_world * seek_increment_m
                        )
                        move_pose(
                            worker,
                            seek_tool_p,
                            seek_hold_r,
                            2,
                            settle=False,
                        )
                        step(2, capture=not args.physics_only)
                        seek_travel_m += seek_increment_m
                        seek_p, seek_r, _ = pose(part["id"])
                        seek_position_error = float(
                            np.linalg.norm(seek_p - gate_p)
                        )
                        seek_angle_error = _rotation_error_deg(
                            seek_r, desired_r
                        )
                        seek_samples.append(
                            {
                                "commanded_travel_m": seek_travel_m,
                                "position_error_m": seek_position_error,
                                "angle_error_deg": seek_angle_error,
                                "contact_frames_by_mate": {
                                    mate_id: seek_contact_frames(mate_id)
                                    for mate_id in expected_mates_for_seek
                                },
                            }
                        )
                        # Never use pressure seeking to justify leaving the
                        # strict connector envelope.  The later gate records
                        # the failure, but this bound stops further loading.
                        axial_error = float(
                            np.dot(seek_p - gate_p, approach_axis_world)
                        )
                        if (
                            seek_position_error > connector_position_tolerance
                            and axial_error < 0.0
                        ):
                            break

                    final_seek_frames = {
                        mate_id: seek_contact_frames(mate_id)
                        for mate_id in expected_mates_for_seek
                    }
                    event["digital_pressure_contact_seek"] = {
                        "method": "bounded_positive_impulse_feedback",
                        "orientation_mode": "hold_post_correction_measured_tool",
                        "direction_world": (-approach_axis_world).tolist(),
                        "increment_m": seek_increment_m,
                        "maximum_travel_m": seek_max_travel_m,
                        "commanded_travel_m": seek_travel_m,
                        "minimum_consecutive_contact_frames": 2,
                        "contact_frames_by_mate": final_seek_frames,
                        "samples": seek_samples,
                        "collisions_remained_enabled": True,
                        "hidden_pose_write": False,
                        "passed": all(
                            has_consecutive_contact(frames)
                            for frames in final_seek_frames.values()
                        ),
                    }

            actual_p, actual_r, actual_q = pose(part["id"])
            position_error = float(np.linalg.norm(actual_p - gate_p))
            angle_error = _rotation_error_deg(actual_r, desired_r)
            event["physical_pose_corrections"] = corrections
            event["connector_gate"] = {
                "position_error_m": position_error,
                "angle_error_deg": angle_error,
                "actual_quat_wxyz": _matrix_quat(actual_r).tolist(),
                "desired_quat_wxyz": _matrix_quat(desired_r).tolist(),
                "position_tolerance_m": connector_position_tolerance,
                "angle_tolerance_deg": connector_angle_tolerance,
                "passed": position_error <= connector_position_tolerance and angle_error <= connector_angle_tolerance,
            }
            if not event["connector_gate"]["passed"]:
                raise RuntimeError(
                    f"connector gate failed for {part['id']}: {position_error * 1000:.2f} mm, {angle_error:.2f} deg"
                )
            # A pose gate alone can approve a floating or interpenetrating
            # object.  Hold the robot still and require sustained PhysX
            # contact with every declared mate before the tensile connector
            # joint is allowed to exist.  Collisions remain enabled.
            contact_hold_start = physics_step
            hold_start_p, hold_start_r, _ = pose(part["id"])
            contact_hold_steps = 8
            step(contact_hold_steps, capture=not args.physics_only)
            hold_end_p, hold_end_r, _ = pose(part["id"])
            child_prefix = "/World/LegoParts/" + part["id"]
            expected_mates = list(task.get("mates", []))
            contact_rows = []
            observed_mates = []
            minimum_contact_frames = 2

            def longest_consecutive_run(frames):
                longest = current = 0
                previous = None
                for value in frames:
                    current = current + 1 if previous is not None and value == previous + 1 else 1
                    longest = max(longest, current)
                    previous = value
                return longest

            def is_connector_pair(record):
                left = record["collider0"]
                right = record["collider1"]
                stud_socket = (
                    "/Collision/Stud" in left
                    and "/Collision/Socket" in right
                    or "/Collision/Socket" in left
                    and "/Collision/Stud" in right
                )
                left_role = left.rsplit("/", 1)[-1]
                right_role = right.rsplit("/", 1)[-1]
                skirts = {"Part01", "Part02", "Part03", "Part04"}
                support_surfaces = {"Part00", "Part05"}
                axial_seat = (
                    left_role in skirts
                    and right_role in support_surfaces
                    or right_role in skirts
                    and left_role in support_surfaces
                )
                return stud_socket or axial_seat

            if expected_mates:
                for mate_id in expected_mates:
                    mate_prefix = "/World/LegoParts/" + mate_id
                    rows = [
                        record
                        for record in assembly_contact_records[assembly_contact_start:]
                        if record["physics_step"] >= contact_hold_start
                        and is_connector_pair(record)
                        and (
                            record["collider0"].startswith(child_prefix)
                            and record["collider1"].startswith(mate_prefix)
                            or record["collider1"].startswith(child_prefix)
                            and record["collider0"].startswith(mate_prefix)
                        )
                    ]
                    frames = sorted({row["physics_step"] for row in rows})
                    consecutive_frames = longest_consecutive_run(frames)
                    pressure_seek = event.get(
                        "digital_pressure_contact_seek", {}
                    )
                    pressure_frames = pressure_seek.get(
                        "contact_frames_by_mate", {}
                    ).get(mate_id, [])
                    pressure_consecutive_frames = longest_consecutive_run(
                        pressure_frames
                    )
                    pressure_seek_verified = bool(
                        pressure_seek.get("passed", False)
                        and pressure_consecutive_frames
                        >= minimum_contact_frames
                    )
                    contact_verified = (
                        consecutive_frames >= minimum_contact_frames
                        or pressure_seek_verified
                    )
                    if contact_verified:
                        observed_mates.append(mate_id)
                    contact_rows.append(
                        {
                            "mate": mate_id,
                            "contact_frames": frames,
                            "frame_count": len(frames),
                            "longest_consecutive_frames": consecutive_frames,
                            "maximum_impulse_Ns": max(
                                (row["maximum_impulse_Ns"] for row in rows),
                                default=0.0,
                            ),
                            "pressure_seek_contact_frames": pressure_frames,
                            "pressure_seek_longest_consecutive_frames": (
                                pressure_consecutive_frames
                            ),
                            "pressure_seek_verified": pressure_seek_verified,
                            "connector_pair": "Stud/Socket or skirt/seat",
                            "passed": contact_verified,
                        }
                    )
            else:
                fixture_prefix = "/World/LegoFixture/"
                rows = [
                    record
                    for record in assembly_contact_records[assembly_contact_start:]
                    if record["physics_step"] >= contact_hold_start
                    and (
                        record["collider0"].startswith(child_prefix)
                        and record["collider1"].startswith(fixture_prefix)
                        or record["collider1"].startswith(child_prefix)
                        and record["collider0"].startswith(fixture_prefix)
                    )
                ]
                frames = sorted({row["physics_step"] for row in rows})
                consecutive_frames = longest_consecutive_run(frames)
                contact_rows.append(
                    {
                        "mate": "fixture",
                        "contact_frames": frames,
                        "frame_count": len(frames),
                        "longest_consecutive_frames": consecutive_frames,
                        "maximum_impulse_Ns": max(
                            (row["maximum_impulse_Ns"] for row in rows),
                            default=0.0,
                        ),
                        "connector_pair": "Part/Fixture",
                        "passed": consecutive_frames >= minimum_contact_frames,
                    }
                )
            allowed_part_ids = set(expected_mates) | set(
                task.get("allowed_contacts", [])
            )
            allowed_supports = set(task.get("allowed_supports", []))
            unexpected_contacts = []
            for record in assembly_contact_records[assembly_contact_start:]:
                if record["physics_step"] < contact_hold_start:
                    continue
                if record["collider0"].startswith(child_prefix):
                    other = record["collider1"]
                elif record["collider1"].startswith(child_prefix):
                    other = record["collider0"]
                else:
                    continue
                if other.startswith("/World/LegoParts/"):
                    other_id = other.split("/", 4)[3]
                    if other_id not in allowed_part_ids:
                        unexpected_contacts.append(record)
                elif (
                    other.startswith("/World/LegoFixture/")
                    and "fixture" not in allowed_supports
                ):
                    unexpected_contacts.append(record)
                elif other.startswith("/World/LegoFixture/"):
                    pass
                elif any(
                    other.startswith(path) for path in worker_tool_contact_paths
                ):
                    pass
                elif not other.startswith(child_prefix):
                    # Table, tray, non-tool robot links, and any other scene
                    # collider are unapproved during the connector hold.
                    unexpected_contacts.append(record)
            hold_duration_s = contact_hold_steps / 60.0
            linear_speed = float(
                np.linalg.norm(hold_end_p - hold_start_p) / hold_duration_s
            )
            angular_speed = float(
                _rotation_error_deg(hold_end_r, hold_start_r) / hold_duration_s
            )
            event["connector_contact_gate"] = {
                "expected_mates": expected_mates or ["fixture"],
                "observed_mates": observed_mates or (
                    ["fixture"] if contact_rows[0]["passed"] else []
                ),
                "minimum_contact_frames": minimum_contact_frames,
                "minimum_positive_impulse_Ns": 1.0e-9,
                "contacts": contact_rows,
                "allowed_nonmate_contacts": sorted(task.get("allowed_contacts", [])),
                "allowed_supports": sorted(allowed_supports),
                "unexpected_contacts": unexpected_contacts[:20],
                "linear_speed_m_s": linear_speed,
                "angular_speed_deg_s": angular_speed,
                "maximum_linear_speed_m_s": 0.01,
                "maximum_angular_speed_deg_s": 8.0,
                "collisions_remained_enabled": True,
                "passed": (
                    all(row["passed"] for row in contact_rows)
                    and not unexpected_contacts
                    and linear_speed <= 0.01
                    and angular_speed <= 8.0
                ),
            }
            if not event["connector_contact_gate"]["passed"]:
                raise RuntimeError(
                    f"physical connector contact gate failed for {part['id']}: "
                    f"contacts={[(row['mate'], row['frame_count']) for row in contact_rows]}, "
                    f"speed={linear_speed:.4f} m/s/{angular_speed:.2f} deg/s"
                )
            event["connector_seating_correction"] = {
                "position_error_m": position_error,
                "angle_error_deg": angle_error,
                "method": (
                    "force_limited_plunger_against_temporary_axial_seat"
                    if seating
                    else "measured_robot_servo_before_connector_lock"
                ),
                "hidden_pose_write": False,
            }
            # For a seated operation, first verify the measured-frame connector
            # without changing support. Then unload pressure and release the
            # carry joint while the connector retains the part.
            # Once strict pose and sustained physical-contact gates pass, keep
            # the measured carry constraint active until the measured-frame
            # connector lock owns the part.  Dropping a milligram-scale LEGO
            # piece unsupported for two frames before creating its lock lets
            # contact spring energy undo an otherwise valid placement.
            atomic_connector_handoff = bool(grasp_retention_active)
            if D.CONSTRAINT_ASSISTED_GRASP and not atomic_connector_handoff:
                handoff_contact_start = physics_step
                grasp_joint.GetJointEnabledAttr().Set(False)
                grasp_retention_active = False
                step(2, capture=not args.physics_only)
                handoff_p, handoff_r, _ = pose(part["id"])
                handoff_position_error = float(np.linalg.norm(handoff_p - gate_p))
                handoff_angle_error = _rotation_error_deg(handoff_r, desired_r)
                handoff_contact_rows = []
                for mate_id in expected_mates:
                    mate_prefix = "/World/LegoParts/" + mate_id
                    frames = sorted(
                        {
                            record["physics_step"]
                            for record in assembly_contact_records[assembly_contact_start:]
                            if record["physics_step"] >= handoff_contact_start
                            and is_connector_pair(record)
                            and (
                                record["collider0"].startswith(child_prefix)
                                and record["collider1"].startswith(mate_prefix)
                                or record["collider1"].startswith(child_prefix)
                                and record["collider0"].startswith(mate_prefix)
                            )
                        }
                    )
                    handoff_contact_rows.append(
                        {
                            "mate": mate_id,
                            "contact_frames": frames,
                            "longest_consecutive_frames": longest_consecutive_run(frames),
                            "present_on_final_step": physics_step - 1 in frames,
                            "passed": (
                                longest_consecutive_run(frames) >= 2
                                and physics_step - 1 in frames
                            ),
                        }
                    )
                if not expected_mates:
                    frames = sorted(
                        {
                            record["physics_step"]
                            for record in assembly_contact_records[assembly_contact_start:]
                            if record["physics_step"] >= handoff_contact_start
                            and (
                                record["collider0"].startswith(child_prefix)
                                and record["collider1"].startswith("/World/LegoFixture/")
                                or record["collider1"].startswith(child_prefix)
                                and record["collider0"].startswith("/World/LegoFixture/")
                            )
                        }
                    )
                    handoff_contact_rows.append(
                        {
                            "mate": "fixture",
                            "contact_frames": frames,
                            "longest_consecutive_frames": longest_consecutive_run(frames),
                            "present_on_final_step": physics_step - 1 in frames,
                            "passed": (
                                longest_consecutive_run(frames) >= 2
                                and physics_step - 1 in frames
                            ),
                        }
                    )
                handoff_unexpected = []
                for record in assembly_contact_records[assembly_contact_start:]:
                    if record["physics_step"] < handoff_contact_start:
                        continue
                    if record["collider0"].startswith(child_prefix):
                        other = record["collider1"]
                    elif record["collider1"].startswith(child_prefix):
                        other = record["collider0"]
                    else:
                        continue
                    if other.startswith("/World/LegoParts/"):
                        if other.split("/", 4)[3] not in allowed_part_ids:
                            handoff_unexpected.append(record)
                    elif other.startswith("/World/LegoFixture/"):
                        if "fixture" not in allowed_supports:
                            handoff_unexpected.append(record)
                    elif any(
                        other.startswith(path)
                        for path in worker_tool_contact_paths
                    ):
                        pass
                    elif not other.startswith(child_prefix):
                        handoff_unexpected.append(record)
                event["prelock_grasp_handoff_gate"] = {
                    "position_error_m": handoff_position_error,
                    "angle_error_deg": handoff_angle_error,
                    "contacts": handoff_contact_rows,
                    "unexpected_contacts": handoff_unexpected[:20],
                    "passed": (
                        handoff_position_error <= connector_position_tolerance
                        and handoff_angle_error <= connector_angle_tolerance
                        and all(row["passed"] for row in handoff_contact_rows)
                        and not handoff_unexpected
                    ),
                }
                if not event["prelock_grasp_handoff_gate"]["passed"]:
                    raise RuntimeError(
                        f"pre-lock handoff gate failed for {part['id']}: "
                        f"{handoff_position_error*1000:.2f} mm/"
                        f"{handoff_angle_error:.2f} deg, "
                        f"contacts={[(row['mate'], row['passed']) for row in handoff_contact_rows]}"
                    )
            pre_lock_p, pre_lock_r, _ = pose(part["id"])
            lock_position = pre_lock_p
            lock_rotation = pre_lock_r
            lock_path, parent = connector_lock(task, lock_position, lock_rotation)
            if atomic_connector_handoff:
                event["grasp_retention_constraint"][
                    "staged_handoff_to_connector_lock"
                ] = True
            event.update(connector_lock_activated=True, connector_lock_path=lock_path, connector_parent=parent)
            report["connector_locks"] += 1
            step(
                30 if atomic_connector_handoff else 1,
                capture=not args.physics_only,
            )
            post_lock_p, post_lock_r, _ = pose(part["id"])
            lock_position_jump = float(np.linalg.norm(post_lock_p - pre_lock_p))
            lock_angle_jump = _rotation_error_deg(post_lock_r, pre_lock_r)
            event["connector_lock_jump"] = {
                "position_m": lock_position_jump,
                "angle_deg": lock_angle_jump,
                "position_tolerance_m": 0.0002,
                "angle_tolerance_deg": 1.5,
                "locked_at_measured_frame": True,
                "passed": lock_position_jump <= 0.0002 and lock_angle_jump <= 1.5,
            }
            if not event["connector_lock_jump"]["passed"]:
                raise RuntimeError(
                    f"connector lock jump failed for {part['id']}: "
                    f"{lock_position_jump*1000:.3f} mm, {lock_angle_jump:.3f} deg"
                )
            if atomic_connector_handoff:
                unload_start = worker["plunger"]
                unload_steps = max(1, int(math.ceil(unload_start / 0.0002)))
                for unload_step in range(unload_steps):
                    worker["plunger"] = unload_start * (
                        1 - (unload_step + 1) / unload_steps
                    )
                    step(2, capture=not args.physics_only)
                grasp_joint.GetJointEnabledAttr().Set(False)
                grasp_retention_active = False
                step(12, capture=not args.physics_only)
                handoff_p, handoff_r, _ = pose(part["id"])
                handoff_position_change = float(
                    np.linalg.norm(handoff_p - post_lock_p)
                )
                handoff_angle_change = _rotation_error_deg(
                    handoff_r, post_lock_r
                )
                event["connector_handoff_after_pressure_unload"] = {
                    "position_change_m": handoff_position_change,
                    "angle_change_deg": handoff_angle_change,
                    "position_tolerance_m": D.FINAL_POSITION_TOLERANCE_M,
                    "angle_tolerance_deg": D.FINAL_ANGLE_TOLERANCE_DEG,
                    "passed": (
                        handoff_position_change
                        <= D.FINAL_POSITION_TOLERANCE_M
                        and handoff_angle_change
                        <= D.FINAL_ANGLE_TOLERANCE_DEG
                    ),
                }
                if not event["connector_handoff_after_pressure_unload"][
                    "passed"
                ]:
                    raise RuntimeError(
                        f"connector support handoff gate failed for "
                        f"{part['id']}: {handoff_position_change*1000:.3f} "
                        f"mm/{handoff_angle_change:.3f} deg"
                    )
                handoff_method = (
                    "connector_lock_then_pressure_unload_then_grasp_release"
                    if seating
                    else "connector_lock_then_grasp_release"
                )
                if seating:
                    event["virtual_pressure_sensor"][
                        "support_handoff"
                    ] = handoff_method
                else:
                    event["connector_support_handoff"] = handoff_method
            event["grasp_retention_constraint"]["disabled_before_release"] = True
            jaws_opened_after_lock = False
            if seating:
                event["seating_release_order"] = [
                    "connector_lock",
                    "plunger_retract",
                    "clearance_slide_verify_retracted",
                    "axial_clearance_withdrawal",
                    "jaw_open",
                    "axial_withdrawal",
                    "connector_guide_retained",
                ]
                # Keep the physical seating stop in place while the loaded
                # plunger and jaws unload.  The connector lock has no snap,
                # but cannot by itself prevent a broad shoe from dragging the
                # light part before the tool has cleared it.
                event["connector_seat"][
                    "disabled_before_plunger_retract"
                ] = False
                retract_start = worker["plunger"]
                retract_steps = max(1, int(math.ceil(retract_start / 0.0002)))
                for retract_step in range(retract_steps):
                    worker["plunger"] = retract_start * (
                        1 - (retract_step + 1) / retract_steps
                    )
                    step(2, capture=not args.physics_only)
                measured = worker["arm"].get_joint_positions()
                measured_retracted = float(
                    measured[worker["indices"]["plunger"]]
                )
                step(12, capture=not args.physics_only)
                held_after_retract_p, held_after_retract_r, _ = pose(part["id"])
                retract_position_change = float(
                    np.linalg.norm(held_after_retract_p - post_lock_p)
                )
                retract_angle_change = _rotation_error_deg(
                    held_after_retract_r, post_lock_r
                )
                event["plunger_retraction"] = {
                    "commanded_extension_m": worker["plunger"],
                    "measured_extension_m": measured_retracted,
                    "position_change_from_post_lock_m": retract_position_change,
                    "angle_change_from_post_lock_deg": retract_angle_change,
                    "position_tolerance_m": 0.0002,
                    "angle_tolerance_deg": 0.5,
                    "transient_pose_stayed_within_tolerance": (
                        retract_position_change <= 0.0002
                        and retract_angle_change <= 0.5
                    ),
                    "completed_before_jaw_open": measured_retracted <= 0.0002,
                }
                if not event["plunger_retraction"]["completed_before_jaw_open"]:
                    raise RuntimeError(
                        f"plunger retraction gate failed for {part['id']}: "
                        f"{measured_retracted*1000:.2f} mm"
                    )
                before_slide_p, before_slide_r, _ = pose(part["id"])
                slide_contact_start = len(tool_contact_records)
                ramp_clearance_slide(worker, CLEARANCE_SLIDE_STROKE_M, samples=30)
                measured = worker["arm"].get_joint_positions()
                measured_slide = float(
                    measured[worker["indices"]["JawL_retract"]]
                )
                after_slide_p, after_slide_r, _ = pose(part["id"])
                slide_position_change = float(
                    np.linalg.norm(after_slide_p - before_slide_p)
                )
                slide_angle_change = _rotation_error_deg(
                    after_slide_r, before_slide_r
                )
                slide_pairs = {}
                for record in tool_contact_records[slide_contact_start:]:
                    if "/JawLClearance/" not in record["collider0"]:
                        continue
                    key = (record["collider0"], record["collider1"])
                    slide_pairs[key] = max(
                        slide_pairs.get(key, 0.0), record["maximum_impulse_Ns"]
                    )
                slide_passed = (
                    abs(measured_slide - CLEARANCE_SLIDE_STROKE_M) <= 0.0002
                    and slide_position_change <= 0.0002
                    and slide_angle_change <= 0.5
                )
                event["clearance_slide_retraction"] = {
                    "commanded_position_m": CLEARANCE_SLIDE_STROKE_M,
                    "measured_position_m": measured_slide,
                    "part_position_change_m": slide_position_change,
                    "part_angle_change_deg": slide_angle_change,
                    "position_tolerance_m": 0.0002,
                    "angle_tolerance_deg": 0.5,
                    "contact_pairs": [
                        {
                            "collider0": pair[0],
                            "collider1": pair[1],
                            "maximum_impulse_Ns": impulse,
                        }
                        for pair, impulse in sorted(slide_pairs.items())
                    ],
                    "passed": slide_passed,
                }
                if not slide_passed:
                    raise RuntimeError(
                        f"clearance slide gate failed for {part['id']}: "
                        f"q={measured_slide*1000:.2f} mm, "
                        f"part={slide_position_change*1000:.2f} mm/"
                        f"{slide_angle_change:.2f} deg"
                    )
                # The relaxed jaws already have running clearance around the
                # part. Slide them off axially before opening their envelope;
                # opening beside neighboring bricks can sweep an insert into
                # the assembly even though the grasped part itself is free.
                clearance_tool_r, clearance_tool_p = actual_tool_pose(worker)
                clearance_axis = clearance_tool_r @ np.asarray(C.TOOL_AXIS, float)
                clearance_axis /= np.linalg.norm(clearance_axis)
                clearance_distance = 0.015
                clearance_target = (
                    clearance_tool_p - clearance_axis * clearance_distance
                )
                move_pose(worker, clearance_target, clearance_tool_r, 6)
                after_clearance_p, after_clearance_r, _ = pose(part["id"])
                event["axial_clearance_withdrawal"] = {
                    "distance_m": clearance_distance,
                    "axis_world": (-clearance_axis).tolist(),
                    "part_position_change_from_post_lock_m": float(
                        np.linalg.norm(after_clearance_p - post_lock_p)
                    ),
                    "part_angle_change_from_post_lock_deg": _rotation_error_deg(
                        after_clearance_r, post_lock_r
                    ),
                    "completed_before_jaw_open": True,
                }
            else:
                step(29, capture=not args.physics_only)
            # Open without commanding a stale pre-press wrist target. Then
            # withdraw along the measured post-release tool axis.
            if not jaws_opened_after_lock:
                ramp_jaw_only(worker, C.JAW_OPEN, 10)
            released_tool_r, released_tool_p = actual_tool_pose(worker)
            withdrawal_axis = released_tool_r @ np.asarray(C.TOOL_AXIS, float)
            withdrawal_axis /= np.linalg.norm(withdrawal_axis)
            withdrawal_distance = 0.040 if seating else 0.055
            withdrawal_target = released_tool_p - withdrawal_axis * withdrawal_distance
            event["release_withdrawal"] = {
                "jaw_opened_at_current_pose": True,
                "axis_world": (-withdrawal_axis).tolist(),
                "distance_m": withdrawal_distance,
                "target_m": withdrawal_target.tolist(),
            }
            move_pose(worker, withdrawal_target, released_tool_r, 10)
            if neighbor_slide_retracted:
                before_neighbor_extension_p, before_neighbor_extension_r, _ = (
                    pose(part["id"])
                )
                ramp_clearance_slide(worker, 0.0, samples=30)
                measured_joints = worker["arm"].get_joint_positions()
                measured_neighbor_extension = float(
                    measured_joints[worker["indices"]["JawL_retract"]]
                )
                after_neighbor_extension_p, after_neighbor_extension_r, _ = (
                    pose(part["id"])
                )
                extension_position_change = float(
                    np.linalg.norm(
                        after_neighbor_extension_p - before_neighbor_extension_p
                    )
                )
                extension_angle_change = _rotation_error_deg(
                    after_neighbor_extension_r, before_neighbor_extension_r
                )
                extension_passed = (
                    abs(measured_neighbor_extension) <= 0.0002
                    and extension_position_change
                    <= D.FINAL_POSITION_TOLERANCE_M
                    and extension_angle_change
                    <= D.FINAL_ANGLE_TOLERANCE_DEG
                )
                event["reactive_neighbor_clearance_extension"] = {
                    "completed_after_axial_withdrawal": True,
                    "commanded_position_m": 0.0,
                    "measured_position_m": measured_neighbor_extension,
                    "part_position_change_m": extension_position_change,
                    "part_angle_change_deg": extension_angle_change,
                    "passed": extension_passed,
                }
                if not extension_passed:
                    raise RuntimeError(
                        "reactive neighbor clearance extension gate failed "
                        f"for {part['id']}"
                    )
            if seating:
                ramp_clearance_slide(worker, 0.0, samples=30)
                measured = worker["arm"].get_joint_positions()
                measured_extended = float(
                    measured[worker["indices"]["JawL_retract"]]
                )
                event["clearance_slide_extension"] = {
                    "commanded_position_m": 0.0,
                    "measured_position_m": measured_extended,
                    "position_tolerance_m": 0.0002,
                    "passed": abs(measured_extended) <= 0.0002,
                    "completed_with_tool_clear": True,
                }
                if not event["clearance_slide_extension"]["passed"]:
                    raise RuntimeError("clearance slide failed to re-extend")
                # Retain the collision envelope as the explicit stud/anti-stud
                # clearance model. The measured-frame joint supplies tensile
                # retention; this guide supplies only contact support that the
                # coarse solid LEGO proxies cannot otherwise represent.
                step(12, capture=not args.physics_only)
                event["connector_seat"]["retained_after_tool_clear"] = True
                event["connector_seat"]["retention_function"] = (
                    "mechanical_contact_clearance_only; tensile retention is "
                    "the post-gate connector lock"
                )
            final_p, final_r, _ = pose(part["id"])
            final_gate_p = gate_p
            final_gate_r = desired_r
            if parent_id is not None:
                # During underside operations Arm B is the compliant fixture.
                # Releasing Arm A can move the supported assembly as a whole
                # by a fraction of a millimetre without changing the LEGO
                # connection.  The acceptance frame therefore follows the
                # measured connector parent, just as it did when the target
                # was first constructed.  Absolute assembly registration is
                # checked separately when the root is docked in the fixture.
                final_parent_p, final_parent_r, _ = pose(parent_id)
                final_gate_p = (
                    final_parent_p
                    + final_parent_r @ child_from_parent_p
                    + np.asarray(
                        profile.get(
                            "pose_gate_offset_m", (0.0, 0.0, 0.0)
                        ),
                        float,
                    )
                )
                final_gate_r = final_parent_r @ child_from_parent_r
                event["final_placement_reference"] = {
                    "frame": parent_id,
                    "measured_parent": True,
                    "world_position_change_from_prelock_gate_m": float(
                        np.linalg.norm(final_gate_p - gate_p)
                    ),
                    "world_angle_change_from_prelock_gate_deg": (
                        _rotation_error_deg(final_gate_r, desired_r)
                    ),
                }
            else:
                event["final_placement_reference"] = {
                    "frame": plan["fixture_frame"],
                    "measured_parent": False,
                    "world_position_change_from_prelock_gate_m": 0.0,
                    "world_angle_change_from_prelock_gate_deg": 0.0,
                }
            final_position_error = float(
                np.linalg.norm(final_p - final_gate_p)
            )
            final_angle_error = _rotation_error_deg(final_r, final_gate_r)
            event["final_position_error_m"] = final_position_error
            event["final_angle_error_deg"] = final_angle_error
            event["placement_verified"] = (
                final_position_error <= D.FINAL_POSITION_TOLERANCE_M
                and final_angle_error <= D.FINAL_ANGLE_TOLERANCE_DEG
            )
            if not event["placement_verified"]:
                raise RuntimeError(f"final placement gate failed for {part['id']}")
            report["verified_placements"] += 1
            _atomic_json(run / "execution_report.json", report)
            print(
                f"[lego] {task['id']} {part['id']}: lift={lift_m*1000:.1f} mm "
                f"gate={position_error*1000:.2f} mm/{angle_error:.2f} deg",
                flush=True,
            )

        park_arm(worker, home_world["arm_a"])
        step(30, capture=not args.physics_only)
        report["frames"] = frame
        report["rendered"] = not args.physics_only
        report["exact_connector_graph"] = all(
            event.get("connector_lock_activated")
            and event.get("connector_contact_gate", {}).get("passed")
            for event in report["events"]
        )
        report["physical_contact_graph_verified"] = all(
            set(event.get("connector_contact_gate", {}).get("observed_mates", []))
            == set(event.get("connector_contact_gate", {}).get("expected_mates", []))
            for event in report["events"]
        )
        report["fixture_support_verified"] = bool(report.get("fixture_support"))
        report["task_complete"] = bool(
            len(tasks) == len(plan["tasks"])
            and report["verified_pickups"] == len(plan["tasks"])
            and report["verified_placements"] == len(plan["tasks"])
            and report["connector_locks"] == len(plan["tasks"])
            and report["exact_connector_graph"]
            and report["physical_contact_graph_verified"]
            and report.get("post_dock_assembly_integrity", {}).get(
                "passed", False
            )
            and report["ik_failures"] == 0
            and not report["tracking_gate_misses"]
            and not report.get("contact_telemetry_truncated", False)
            and report["teleports_during_execution"] == 0
        )
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        _atomic_json(run / "execution_report.json", report)
        app.close()


if __name__ == "__main__":
    main()
