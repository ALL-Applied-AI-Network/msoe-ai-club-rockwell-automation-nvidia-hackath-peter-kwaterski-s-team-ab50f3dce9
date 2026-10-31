"""Execute the Cosmos sort plan with physical grasps and pose-based reporting.

Run ./run_sort_pipeline.sh RUN_DIR to capture, perceive, plan, render and encode.
"""

import argparse
import json
import os
from pathlib import Path

import numpy as np
import scene_config as C
import fk
import ik
from cube_pose import square_yaw


def ease(t):
    return t * t * (3 - 2 * t)


def turn(a, b, t):
    a, b = np.array(a, float), np.array(b, float)
    angle = np.arctan2(a[0] * b[1] - a[1] * b[0], np.dot(a[:2], b[:2])) * t
    c, s = np.cos(angle), np.sin(angle)
    return np.array([a[0] * c - a[1] * s, a[0] * s + a[1] * c, 0.0])


def verify_cube_stack(positions, entries):
    """Check site, layer heights and actual adjacent-cube support geometry."""
    centres = []
    for entry in entries:
        name = entry.get("tracked_object")
        if name not in positions:
            return False
        pose = positions[name]
        if "yaw_deg" in pose and (
            abs(square_yaw(pose["yaw_deg"])) >= 8 or pose.get("tilt_deg", 0) >= 5
        ):
            return False
        centre = np.array(pose["centre"])
        if np.linalg.norm(centre[:2] - C.CUBE_STACK_XY) > C.CUBE_STACK_SITE_TOLERANCE:
            return False
        height = (
            C.TABLE_TOP_Z + C.SURFACE_DZ + (entry["stack_level"] + 0.5) * C.CUBE_EDGE
        )
        if abs(centre[2] - height) > 0.0015:
            return False
        if (
            centres
            and np.linalg.norm(centre[:2] - centres[-1][:2])
            > C.CUBE_STACK_LAYER_TOLERANCE
        ):
            return False
        centres.append(centre)
    return True


def segments(pick, home):
    p = np.array([*pick["xy"], pick["pick_z"]])
    d = np.array([*pick["place_xy"], pick["place_z"]])
    up, above = p + [0, 0, 0.13], d + [0, 0, 0.13]
    spread = pick.get("spread_dir")
    dest_spread = pick.get("place_spread_dir", spread)
    # Seat on the expected cube width instead of forcing rigid fingers through it.
    shut = C.CUBE_JAW_HALF_TARGET if pick["category"] == "cube" else C.JAW_CLOSED
    return [
        ("approach", home, up, 1.2, C.JAW_OPEN, C.JAW_OPEN, spread),
        ("descend", up, p, 0.8, C.JAW_OPEN, C.JAW_OPEN, spread),
        ("close", p, p, 0.6, C.JAW_OPEN, shut, spread),
        ("dwell", p, p, 0.5, shut, shut, spread),
        ("lift", p, up, 0.8, shut, shut, spread),
        ("carry", up, above, 1.4, shut, shut, dest_spread),
        ("lower", above, d, 0.8, shut, shut, dest_spread),
        ("release", d, d, 0.6, shut, C.JAW_OPEN, dest_spread),
        ("settle", d, d, 0.6, C.JAW_OPEN, C.JAW_OPEN, dest_spread),
        ("retreat", d, above, 0.8, C.JAW_OPEN, C.JAW_OPEN, dest_spread),
        ("home", above, home, 1.1, C.JAW_OPEN, C.JAW_OPEN, None),
    ]


def validate_plan(plan):
    if plan.get("source") != "Cosmos3-Nano":
        raise ValueError(
            "Expected a Cosmos3-Nano sort plan; run the perception pipeline first"
        )
    if not np.isclose(plan.get("open_gap_m", 0), 2 * C.JAW_OPEN):
        raise ValueError(
            "Plan uses a different jaw opening; regenerate capture and perception"
        )
    if plan.get("task") == "cube_stack" and plan["picks"]:
        if [p.get("colour") for p in plan["picks"]] != list(C.CUBE_STACK_COLOURS):
            raise ValueError(
                "Tower must contain two yellow, two blue, then two red cubes"
            )
        if len({p["detection_id"] for p in plan["picks"]}) != len(C.CUBE_STACK_COLOURS):
            raise ValueError("Tower repeats a cube detection")
        for level, p in enumerate(plan["picks"]):
            yaw = float(p.get("yaw_deg", float("nan")))
            theta = np.deg2rad(yaw)
            expected = [-np.sin(theta), np.cos(theta), 0.0]
            spread = np.asarray(p.get("spread_dir", []), float)
            destination = np.asarray(p.get("place_spread_dir", []), float)
            if (
                not np.isfinite(yaw)
                or not np.isfinite(float(p.get("pose_confidence", 0)))
                or float(p.get("pose_confidence", 0)) < 0.8
                or spread.shape != (3,)
                or not np.allclose(spread, expected)
                or destination.shape != (3,)
                or not np.allclose(destination, [0.0, 1.0, 0.0])
            ):
                raise ValueError(
                    "Cube grasp requires a confident pose and aligned wrist"
                )
            if p["category"] != "cube" or p.get("stack_level") != level:
                raise ValueError("Invalid cube tower level")
            if not np.allclose(p["place_xy"], C.CUBE_STACK_XY) or not np.isclose(
                p["place_z"], C.SURFACE_DZ + level * C.CUBE_EDGE + C.CUBE_TIP_CLEARANCE
            ):
                raise ValueError("Invalid tower destination or placement height")
            if not np.isclose(p.get("pick_z", -1), C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE):
                raise ValueError(
                    "Cube grasp height differs from the current hooked-tip setup"
                )
            if not np.isclose(
                p.get("expected_centre_z", -1),
                C.SURFACE_DZ + (level + 0.5) * C.CUBE_EDGE,
            ):
                raise ValueError("Invalid expected tower height")
    for p in plan["picks"]:
        if p["category"] == "cap":
            if (
                p.get("orientation") != "narrow_end_up"
                or float(p.get("orientation_confidence", 0))
                < C.CAP_ORIENTATION_MIN_CONFIDENCE
            ):
                raise ValueError(
                    "Plan contains a cap without a confident narrow-end-up observation"
                )
            if not np.isclose(p["pick_z"], C.SURFACE_DZ + C.CAP_PICK_HEIGHT):
                raise ValueError("Cap pick height must clear the wide flange")
        if p["category"] == "vial":
            axis = np.array(p.get("axis_world", []), float)
            spread = np.array(p.get("spread_dir", []), float)
            if (
                axis.shape != (2,)
                or spread.shape != (3,)
                or not np.isclose(np.linalg.norm(spread), 1)
                or abs(np.dot(axis, spread[:2])) > 1e-5
            ):
                raise ValueError("Tube must have a perpendicular jaw closing direction")
        for key in ("xy", "place_xy"):
            if len(p[key]) != 2 or not np.all(np.isfinite(p[key])):
                raise ValueError("Invalid plan coordinates")


def capture_table_image(cam, path):
    import cv2

    rgba = cam.get_rgba()
    if rgba is None:
        raise RuntimeError("Camera returned no image for capture")
    cv2.imwrite(
        str(path),
        cv2.cvtColor(rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR),
    )


def capture_table_snapshot(stage, cam, cam_path, resolution, run_dir: Path):
    """Save PNG and matching camera.json for host-side remeasurement."""
    from camera_export import write_isaac_camera_json

    run_dir = Path(run_dir)
    capture_table_image(cam, run_dir / "mid_step_capture.png")
    write_isaac_camera_json(
        stage,
        cam_path,
        resolution,
        run_dir / "mid_step_camera.json",
    )


def execute_one_pick(
    pick,
    *,
    wrappers,
    poses,
    world,
    command,
    fk_mod,
    ik_mod,
    limits,
    tool,
    home,
    down,
    q,
    arm,
    arm_idx,
    idx,
    frame,
    fps,
    physics_only,
    camera_name,
    out,
    side_out,
    cam,
    side,
    events,
    live_frame=None,
):
    """Run one planned grasp; return event, updated q/frame, and optional abort reason."""
    from grasp_fault import INJECT_ACCIDENTAL_DROP

    pick = dict(pick)
    inject_drop = bool(pick.pop(INJECT_ACCIDENTAL_DROP, False))
    if pick.get("spread_dir") is not None:
        r, _ = fk_mod.fk(q)
        current = r @ np.array(C.FINGER_SPREAD_AXIS)
        spread = np.array(pick["spread_dir"], float)
        if np.dot(current, spread) < 0:
            spread = -spread
        pick["spread_dir"] = spread.tolist()
        if pick.get("place_spread_dir") is not None:
            destination = np.array(pick["place_spread_dir"], float)
            if np.dot(spread, destination) < 0:
                destination = -destination
            pick["place_spread_dir"] = destination.tolist()
    start = poses()
    prefix = {"cap": "SortCap", "cube": "SortCube", "vial": "SortVial"}[
        pick["category"]
    ]
    candidates = [name for name in wrappers if name.startswith(prefix)]
    tracked = (
        min(
            candidates,
            key=lambda name: np.linalg.norm(
                np.array(start[name]["centre"][:2]) - pick["xy"]
            ),
        )
        if candidates
        else None
    )
    if (
        tracked
        and np.linalg.norm(np.array(start[tracked]["centre"][:2]) - pick["xy"])
        > 0.04
    ):
        tracked = None
    event = {
        "detection_id": pick["detection_id"],
        "category": pick["category"],
        "tracked_object": tracked,
        "pickup_verified": False,
        "placement_verified": False,
        "trajectory_ik_failures": 0,
    }
    if "stack_level" in pick:
        event.update(
            stack_level=pick["stack_level"],
            colour=pick["colour"],
            perceived_yaw_deg=pick["yaw_deg"],
            grasp_spread_dir=pick["spread_dir"],
            place_spread_dir=pick["place_spread_dir"],
        )
    event["initial_positions"] = start
    for name, p0, p1, duration, j0, j1, spread in segments(pick, home):
        spread_start = None
        if spread is not None and name in ("approach", "carry"):
            r, _ = fk_mod.fk(q)
            s = r @ np.array(C.FINGER_SPREAD_AXIS)
            s[2] = 0
            s /= np.linalg.norm(s)
            spread = np.array(spread, float)
            spread_start = s
        n = round(duration * fps)
        for i in range(n):
            t = ease((i + 1) / n)
            target = p0 + (p1 - p0) * t
            direction = (
                turn(spread_start, spread, t)
                if spread_start is not None
                else spread
            )
            q, ok = ik_mod.solve(
                q, target, down, limits, tool_offset=tool, spread_dir=direction
            )
            if not ok:
                q, ok = ik_mod.solve(
                    q,
                    target,
                    down,
                    limits,
                    tool_offset=tool,
                    spread_dir=direction,
                    iters=500,
                    damping=0.02,
                )
            r, actual = fk_mod.tool_pose(q, tool)
            aligned = (
                direction is None
                or abs(np.dot(r @ np.array(C.FINGER_SPREAD_AXIS), direction))
                > 0.999
            )
            if not ok or np.linalg.norm(actual - target) > 0.001 or not aligned:
                event["trajectory_ik_failures"] += 1
                raise RuntimeError(
                    f'IK failed: detection {pick["detection_id"]}, {name}, frame {i}; refusing off-target descent'
                )
            jaw = j0 + (j1 - j0) * t
            if inject_drop and name == "lift" and i >= max(1, n // 3):
                jaw = C.JAW_OPEN
            elif inject_drop and name == "carry" and i >= max(1, n // 5):
                jaw = C.JAW_OPEN
            command(q, jaw)
            for _ in range(2):
                world.step(render=not physics_only)
            if cam is not None:
                import cv2

                rgba = cam.get_rgba()
                if rgba is None:
                    raise RuntimeError("Camera returned no image")
                bgr = cv2.cvtColor(
                    rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR
                )
                cv2.imwrite(str(out / f"f{frame:05d}.png"), bgr)
                if live_frame is not None:
                    live_frame.maybe_publish(bgr)
            if side is not None:
                import cv2

                rgba_side = side.get_rgba()
                if rgba_side is not None:
                    side_bgr = cv2.cvtColor(
                        rgba_side[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR
                    )
                    cv2.imwrite(str(side_out / f"f{frame:05d}.png"), side_bgr)
                    if live_frame is not None and camera_name != "table":
                        live_frame.maybe_publish(side_bgr)
            frame += 1
        if name in ("descend", "dwell", "lift", "lower", "release"):
            measured = arm.get_joint_positions()
            rot, tip = fk_mod.tool_pose(measured[arm_idx], tool)
            measured_spread = rot @ np.array(C.FINGER_SPREAD_AXIS)
            event.setdefault("phase_measurements", {})[name] = {
                "actual_tip_robot_frame_m": tip.tolist(),
                "actual_spread_dir": measured_spread.tolist(),
                "commanded_spread_dir": (
                    np.asarray(direction).tolist()
                    if direction is not None
                    else None
                ),
                "wrist_joint_deg": float(np.degrees(measured[arm_idx][-1])),
                "commanded_tip_robot_frame_m": target.tolist(),
                "tip_error_mm": float(np.linalg.norm(tip - target) * 1000),
                "object_centre_world_m": (
                    poses()[tracked]["centre"] if tracked else None
                ),
            }
        if name == "dwell":
            jp = arm.get_joint_positions()
            event["jaw_gap_mm"] = float(
                (2 * C.JAW_OPEN - abs(jp[idx["jaw_l"]]) - abs(jp[idx["jaw_r"]]))
                * 1000
            )
        if name == "lift":
            lifted = poses()
            event["objects_lifted"] = [
                key
                for key in wrappers
                if lifted[key]["centre"][2] - start[key]["centre"][2] > 0.025
            ]
            event["pickup_verified"] = tracked in event["objects_lifted"]
            print(
                f'[sort] detection {pick["detection_id"]}: lifted {event["objects_lifted"]}; pickup={event["pickup_verified"]}',
                flush=True,
            )
    finish = poses()
    if tracked:
        position = np.array(finish[tracked]["centre"])
        event["placement_error_mm"] = float(
            np.linalg.norm(position[:2] - pick["place_xy"]) * 1000
        )
        expected_z = (
            C.TABLE_TOP_Z
            + C.SURFACE_DZ
            + {
                "cube": 1.5 * C.CUBE_EDGE,
                "cap": C.CAP_HEIGHT / 2,
                "vial": C.VIAL_RADIUS,
            }[pick["category"]]
        )
        if "stack_level" in pick:
            expected_z = C.TABLE_TOP_Z + pick["expected_centre_z"]
        event["placement_verified"] = bool(
            event["pickup_verified"]
            and event["placement_error_mm"] < 20
            and abs(position[2] - expected_z) < 0.01
        )
        if "stack_level" in pick:
            event["placement_verified"] = bool(
                event["pickup_verified"]
                and event["placement_error_mm"]
                < C.CUBE_STACK_SITE_TOLERANCE * 1000
                and abs(position[2] - expected_z) < 0.0015
            )
            event["placement_yaw_error_deg_mod90"] = abs(
                square_yaw(finish[tracked]["yaw_deg"])
            )
            event["placement_tilt_deg"] = finish[tracked]["tilt_deg"]
            event["placement_pose_verified"] = bool(
                event["placement_yaw_error_deg_mod90"] < 8
                and event["placement_tilt_deg"] < 5
            )
            event["placement_verified"] = (
                event["placement_verified"] and event["placement_pose_verified"]
            )
            event["stack_support_verified"] = verify_cube_stack(
                finish, events + [event]
            )
    abort_reason = None
    return event, q, frame, abort_reason


def _write_events(run, events):
    from run_artifacts import atomic_write_json

    atomic_write_json(run / "execution_events.json", events)


def _invoke_live_recovery(run):
    import subprocess

    host = os.environ.get("HOST_PYTHON")
    script = Path(__file__).resolve().parent / "live_step_recovery.py"
    if not host or not Path(host).is_file() or not script.is_file():
        print("[sort] HOST_PYTHON unset; cannot run live recovery", flush=True)
        return None
    result = subprocess.run(
        [host, str(script), "--run", str(run)],
        cwd=str(script.parent),
        capture_output=True,
        text=True,
        check=False,
    )
    if result.stdout:
        print(result.stdout, flush=True)
    if result.stderr:
        print(result.stderr, flush=True)
    from run_artifacts import read_json

    return read_json(run / "live_recovery_decision.json")


def _run_pick_loop(
    *,
    run,
    plan,
    wrappers,
    poses,
    world,
    command,
    limits,
    tool,
    home,
    down,
    q,
    arm,
    arm_idx,
    idx,
    frame,
    args,
    out,
    side_out,
    cam,
    side,
    before,
    stepwise,
    inject_fault=None,
    stage=None,
    table_cam_path=None,
    table_cam_resolution=None,
):
    from recovery_policy import classify_event
    from run_artifacts import LiveFramePublisher
    from step_controller import (
        advance_after_success,
        apply_recovery_plan,
        begin_recovery,
        cube_stack_failure_reason,
        current_pick,
        is_terminal,
        load_step_state,
        mark_phase,
        mark_stopped,
        save_step_state,
    )

    events = []
    abort_reason = None
    live_frame = LiveFramePublisher(run) if stepwise else None
    step_state = load_step_state(run, plan) if stepwise else None
    if stepwise:
        step_state = mark_phase(step_state, "ready")
        save_step_state(run, step_state)

    index = 0
    while True:
        if stepwise:
            if step_state is None or is_terminal(step_state):
                break
            pick = current_pick(plan, step_state)
        else:
            if index >= len(plan["picks"]):
                break
            pick = plan["picks"][index]
        if pick is None:
            break
        if stepwise:
            step_state = mark_phase(step_state, "executing")
            save_step_state(run, step_state)
            from grasp_fault import inject_fault_label, maybe_biased_pick

            pick_index = int(step_state["current_pick_index"])
            biased = maybe_biased_pick(
                pick,
                pick_index,
                inject_fault,
                retry_count=int(step_state.get("retry_count", 0)),
            )
            if biased is not pick:
                print(
                    f"[sort] inject test fault: {inject_fault_label(inject_fault)} "
                    f"(retry_count={step_state.get('retry_count', 0)})",
                    flush=True,
                )
                pick = biased
        event, q, frame, _ = execute_one_pick(
            pick,
            wrappers=wrappers,
            poses=poses,
            world=world,
            command=command,
            fk_mod=fk,
            ik_mod=ik,
            limits=limits,
            tool=tool,
            home=home,
            down=down,
            q=q,
            arm=arm,
            arm_idx=arm_idx,
            idx=idx,
            frame=frame,
            fps=args.fps,
            physics_only=args.physics_only,
            camera_name=args.camera,
            out=out,
            side_out=side_out,
            cam=cam,
            side=side,
            events=events,
            live_frame=live_frame,
        )
        events.append(event)
        _write_events(run, events)
        if stepwise:
            step_state = mark_phase(step_state, "verifying")
            save_step_state(run, step_state)

        stack_abort = plan.get("task") == "cube_stack" and (
            not event["placement_verified"]
            or not event.get("stack_support_verified")
        )
        if stack_abort and not stepwise:
            abort_reason = f'Cube tower stopped at level {pick["stack_level"]}: pickup, placement or supporting stack failed'
            print(f"[sort] {abort_reason}", flush=True)
            break

        if stepwise:
            failure_type = classify_event(event)
            success = failure_type is None and cube_stack_failure_reason(event) is None
            if success:
                step_state = advance_after_success(step_state, plan)
                save_step_state(run, step_state)
                if is_terminal(step_state):
                    break
                plan = json.loads((run / "sort_plan.json").read_text())
                continue
            if failure_type != "missed_grasp":
                reason = cube_stack_failure_reason(event) or failure_type or "step failed"
                step_state = mark_stopped(
                    step_state,
                    f'Cube tower stopped at level {pick.get("stack_level")}: {reason}',
                )
                save_step_state(run, step_state)
                abort_reason = step_state["terminal_reason"]
                print(f"[sort] {abort_reason}", flush=True)
                break
            if cam is not None and stage is not None and table_cam_path is not None:
                capture_table_snapshot(
                    stage,
                    cam,
                    table_cam_path,
                    table_cam_resolution,
                    run,
                )
            step_state = begin_recovery(step_state)
            save_step_state(run, step_state)
            decision = _invoke_live_recovery(run)
            action = (decision or {}).get("action")
            if action == "stop" or decision is None:
                step_state = mark_stopped(
                    step_state, "Live recovery stopped after missed grasp"
                )
                save_step_state(run, step_state)
                abort_reason = step_state["terminal_reason"]
                break
            events.pop()
            _write_events(run, events)
            plan = json.loads((run / "sort_plan.json").read_text())
            step_state = apply_recovery_plan(step_state, plan)
            save_step_state(run, step_state)
            continue

        index += 1

    if stepwise and step_state and step_state.get("phase") == "ready" and not abort_reason:
        if len(events) == len(plan["picks"]) and all(
            e.get("placement_verified") for e in events
        ):
            step_state = mark_phase(step_state, "completed")
            save_step_state(run, step_state)

    after = poses()
    report = {
        "before": before,
        "after": after,
        "picks": events,
        "frames": frame,
        "rendered": not args.physics_only,
        "task": plan.get("task", "sort"),
        "abort_reason": abort_reason,
        "stepwise": stepwise,
        "hardware": {
            "gripper_model": C.GRIPPER_MODEL,
            "jaw_stroke_m": C.JAW_STROKE,
            "open_gap_m": 2 * C.JAW_OPEN,
            "finger_length_m": C.FINGER_LENGTH,
            "cube_edge_m": C.CUBE_EDGE,
            "cube_tip_clearance_m": C.CUBE_TIP_CLEARANCE,
        },
        "task_complete": bool(plan["picks"])
        and abort_reason is None
        and len(events) == len(plan["picks"])
        and all(e["placement_verified"] for e in events),
        "verified_pickups": sum(e["pickup_verified"] for e in events),
        "verified_placements": sum(e["placement_verified"] for e in events),
    }
    gt = json.loads((run / "ground_truth.json").read_text())
    wide = [
        g["name"] for g in gt["objects"] if g.get("orientation") == "wide_end_up"
    ]
    report["wide_caps"] = [
        {
            "name": name,
            "displacement_mm": float(
                np.linalg.norm(
                    np.array(after[name]["centre"]) - before[name]["centre"]
                )
                * 1000
            ),
            "picked_up": any(name in e.get("objects_lifted", []) for e in events),
        }
        for name in wide
    ]
    from run_artifacts import atomic_write_json

    atomic_write_json(run / "execution_report.json", report)
    print(
        f'[sort] {report["verified_pickups"]}/{len(events)} pickups; {report["verified_placements"]} verified placements; {frame} frames',
        flush=True,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--run",
        required=True,
        help="directory containing sort_plan.json and scene_state.json",
    )
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--camera", choices=["table", "side", "wrist"], default="table")
    ap.add_argument(
        "--physics-only",
        action="store_true",
        help="validate physical grasps without rendering frames",
    )
    ap.add_argument(
        "--stepwise",
        action="store_true",
        help="execute one verified cube at a time with live recovery on missed grasps",
    )
    ap.add_argument(
        "--inject-missed-grasp",
        metavar="PICK:drop|DX:DY:DYAW",
        help="stepwise only: PICK_INDEX:drop opens the gripper mid-lift/carry once, "
        "or PICK_INDEX:DX_MM:DY_MM:DYAW_DEG offsets the grasp (example: 2:drop)",
    )
    a = ap.parse_args()
    if a.inject_missed_grasp and not a.stepwise:
        ap.error("--inject-missed-grasp requires --stepwise")
    inject_fault = None
    if a.inject_missed_grasp:
        from grasp_fault import fault_to_dict, parse_inject_missed_grasp

        inject_fault = parse_inject_missed_grasp(a.inject_missed_grasp)
    a.inject_fault = inject_fault
    if a.fps != 30:
        ap.error("Use 30 fps: two 1/60 s physics steps per video frame")
    run = Path(a.run).resolve()
    out = Path(a.out).resolve() if a.out else run / "frames"
    if out.exists() and any(out.glob("*.png")):
        raise SystemExit(
            f"Output already contains PNGs: {out}. Choose a fresh run directory."
        )
    plan = json.loads((run / "sort_plan.json").read_text())
    validate_plan(plan)
    if inject_fault is not None:
        from run_artifacts import atomic_write_json

        fault_index = inject_fault[0]
        if fault_index >= len(plan["picks"]):
            raise SystemExit(
                f"inject pick index {fault_index} out of range "
                f"(plan has {len(plan['picks'])} picks)"
            )
        atomic_write_json(run / "inject_missed_grasp.json", fault_to_dict(inject_fault))
    if not plan["picks"]:
        raise SystemExit(
            "No eligible picks; inspect detections.json and sort_plan.json before rendering"
        )
    state = json.loads((run / "scene_state.json").read_text())
    # Resolve paths and validate before Kit changes the working directory.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import cv2
        import omni.usd
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        import cell

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        stage = omni.usd.get_context().get_stage()
        cell.build(stage)
        wrappers = {}
        for name, saved in state["objects"].items():
            prim = stage.GetPrimAtPath("/World/" + name)
            if not prim.IsValid():
                raise ValueError(f"Captured object missing: {name}")
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
            wrapper = SingleXFormPrim("/World/" + name)
            wrapper.set_world_pose(
                np.array(saved["position"]), np.array(saved["orientation"])
            )
            wrappers[name] = wrapper
        cam_paths = {
            "table": cell.TABLE_CAM_PATH,
            "side": cell.SIDE_CAM_PATH,
            "wrist": cell.WRIST_CAM_PATH,
        }
        configs = {"table": C.TABLE_CAM, "side": C.SIDE_CAM, "wrist": C.WRIST_CAM}
        from camera_export import table_camera_resolution

        table_cam_resolution = table_camera_resolution(plan.get("task", "sort"))
        cam = (
            None
            if a.physics_only
            else Camera(
                prim_path=cam_paths[a.camera],
                resolution=(
                    table_cam_resolution
                    if a.camera == "table"
                    else configs[a.camera]["resolution"]
                ),
            )
        )
        cell.enable_ccd(world)
        world.reset()
        if cam is not None:
            cam.initialize()
        arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="er4ia")
        arm.initialize()
        names = list(arm.dof_names)
        arm_idx = [names.index(f"J{i}") for i in range(1, 7)]
        idx = {name: i for i, name in enumerate(names)}
        limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
        ik.set_tool_axis(C.TOOL_AXIS)
        ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
        tool = tuple(np.array(C.TOOL_AXIS) * cell.tip_offset())
        home = np.array([0.30, 0.0, 0.24])
        down = [0.0, 0.0, -1.0]
        q = np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)])
        q, ok = ik.solve(q, home, down, limits, tool_offset=tool)
        if not ok:
            raise RuntimeError("Home IK failed")

        def command(q6, jaw):
            full = arm.get_joint_positions().copy()
            full[arm_idx] = q6
            for joint, value in cell.jaw_targets(jaw).items():
                full[idx[joint]] = value
            arm.apply_action(ArticulationAction(joint_positions=full))

        full = arm.get_joint_positions().copy()
        full[arm_idx] = q
        for joint, value in cell.jaw_targets(C.JAW_OPEN).items():
            full[idx[joint]] = value
        arm.set_joint_positions(full)
        for _ in range(120):
            command(q, C.JAW_OPEN)
            world.step(render=not a.physics_only)
        for name in wrappers:
            UsdPhysics.RigidBodyAPI(
                stage.GetPrimAtPath("/World/" + name)
            ).CreateKinematicEnabledAttr().Set(False)
        for _ in range(30):
            command(q, C.JAW_OPEN)
            world.step(render=not a.physics_only)

        def poses():
            values = {}
            for name, wrapper in wrappers.items():
                pos, quat = wrapper.get_world_pose()
                w, x, y, z = quat
                rot = np.array(
                    [
                        [
                            1 - 2 * (y * y + z * z),
                            2 * (x * y - z * w),
                            2 * (x * z + y * w),
                        ],
                        [
                            2 * (x * y + z * w),
                            1 - 2 * (x * x + z * z),
                            2 * (y * z - x * w),
                        ],
                        [
                            2 * (x * z - y * w),
                            2 * (y * z + x * w),
                            1 - 2 * (x * x + y * y),
                        ],
                    ]
                )
                centre = np.array(pos)
                if name.startswith("SortVial"):
                    centre += rot @ np.array([0, 0, C.VIAL_LENGTH / 2])
                elif name.startswith("SortCap"):
                    centre += rot @ np.array([0, 0, C.CAP_HEIGHT / 2])
                elif name.startswith("SortCube"):
                    centre += rot @ np.array([0, 0, C.CUBE_EDGE / 2])
                values[name] = {
                    "position": pos.tolist(),
                    "orientation": quat.tolist(),
                    "centre": centre.tolist(),
                    "yaw_deg": float(np.degrees(np.arctan2(rot[1, 0], rot[0, 0]))),
                    "tilt_deg": float(np.degrees(np.arccos(np.clip(rot[2, 2], -1, 1)))),
                }
            return values

        before = poses()
        frame = 0
        out.mkdir(parents=True, exist_ok=True)
        # Additional fixed side camera provides a clear lift/contact review.
        side = None
        if not a.physics_only and a.camera != "side":
            side = Camera(
                prim_path=cell.SIDE_CAM_PATH, resolution=C.SIDE_CAM["resolution"]
            )
            side.initialize()
        side_out = run / "side_frames"
        side_out.mkdir(exist_ok=True)
        _run_pick_loop(
            run=run,
            plan=plan,
            wrappers=wrappers,
            poses=poses,
            world=world,
            command=command,
            limits=limits,
            tool=tool,
            home=home,
            down=down,
            q=q,
            arm=arm,
            arm_idx=arm_idx,
            idx=idx,
            frame=frame,
            args=a,
            out=out,
            side_out=side_out,
            cam=cam,
            side=side,
            before=before,
            stepwise=a.stepwise,
            inject_fault=a.inject_fault,
            stage=stage,
            table_cam_path=cam_paths["table"],
            table_cam_resolution=table_cam_resolution,
        )
    except Exception:
        # Kit shutdown may terminate Python before the exception is displayed.
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
