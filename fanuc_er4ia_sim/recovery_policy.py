"""Classify failed runs and propose bounded cube-pick replay plans.

The pure helpers do not access files or run a simulator. The command-line
wrapper can replay a corrected plan in a NEW Isaac Sim run directory. This is
offline replay from the captured initial scene, not live in-session recovery.
"""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys


MAX_PICKUP_RETRIES = 2
RETRY_LADDER = ("remeasure", "flip_spread", "nudge")
NUDGE_M = 0.002
MIN_POSE_FIT = 0.75
COPY_TO_RETRY = (
    "camera.json",
    "world_camera.png",
    "scene_state.json",
    "ground_truth.json",
    "detections.json",
)
EXIT_RETRY = 0
EXIT_COMPLETE = 10
EXIT_STOP = 20

STAGE_BY_FAILURE = {
    "untracked_target": "perception",
    "ik_failure": "reachability",
    "missed_grasp": "pickup",
    "wrong_object": "pickup",
    "placement_failed": "placement",
    "support_failed": "support",
    "collision_detected": "execution",
    "dropped_object": "execution",
    "state_uncertain": "execution",
    "execution_crash": "execution",
    "event_plan_mismatch": "execution",
    "incomplete": "execution",
}
RETRYABLE_FAILURES = {"missed_grasp"}


def _failure_for_event(event):
    if event.get("collision_detected") is True:
        return "collision_detected"
    if event.get("drop_detected") is True:
        return "dropped_object"
    if event.get("state_uncertain") is True:
        return "state_uncertain"
    if event.get("tracked_object") is None and "tracked_object" in event:
        return "untracked_target"
    ik_failures = event.get("trajectory_ik_failures", 0)
    if (
        isinstance(ik_failures, bool)
        or not isinstance(ik_failures, (int, float))
        or not math.isfinite(ik_failures)
        or ik_failures < 0
    ):
        raise ValueError("trajectory_ik_failures must be a finite nonnegative number")
    if ik_failures > 0:
        return "ik_failure"

    lifted = event.get("objects_lifted", [])
    if not isinstance(lifted, list):
        raise ValueError("objects_lifted must be a list when present")
    target = event.get("tracked_object")
    if len(lifted) > 1:
        return "wrong_object"
    if event.get("pickup_verified") is False:
        if not lifted:
            return "missed_grasp"
        if target is None or target not in lifted:
            return "wrong_object"
        return "missed_grasp"
    if event.get("pickup_verified") is not True:
        return "state_uncertain"
    if target is not None and target not in lifted:
        return "state_uncertain"
    if event.get("placement_verified") is False:
        return "placement_failed"
    if event.get("placement_verified") is not True:
        return "state_uncertain"
    if event.get("stack_support_verified") is False:
        return "support_failed"
    return None


def classify_event(event):
    """Classify a single execution event, returning ``None`` on success."""
    if not isinstance(event, dict):
        raise TypeError("event must be a dictionary")
    return _failure_for_event(event)


def classify_failure(events, plan=None, report=None):
    """Return the first failure, or ``None`` for successful supplied events.

    When ``plan`` and ``report`` are supplied, also detect early termination,
    a missing execution report, and incomplete runs with no failed event.
    """
    if not isinstance(events, list):
        raise TypeError("events must be a list")
    if plan is not None and not isinstance(plan, dict):
        raise TypeError("plan must be a dictionary")
    if report is not None and not isinstance(report, dict):
        raise TypeError("report must be a dictionary or None")
    picks = plan.get("picks", []) if plan is not None else []
    if not isinstance(picks, list) or any(not isinstance(pick, dict) for pick in picks):
        raise ValueError("plan.picks must be a list of dictionaries")

    if plan is not None and report is None:
        return {
            "stage": "execution",
            "event_index": len(events),
            "failure_type": "execution_crash",
        }

    for event_index, event in enumerate(events):
        if not isinstance(event, dict):
            raise TypeError(f"events[{event_index}] must be a dictionary")
        if plan is not None and event_index < len(picks):
            planned_id = picks[event_index].get("detection_id")
            event_id = event.get("detection_id")
            if planned_id is not None and event_id is not None and planned_id != event_id:
                return {
                    "stage": "execution",
                    "event_index": event_index,
                    "failure_type": "event_plan_mismatch",
                    "detection_id": event_id,
                }
        failure_type = _failure_for_event(event)
        if (
            failure_type is None
            and plan is not None
            and plan.get("task") == "cube_stack"
            and event_index < len(picks)
            and "stack_support_verified" not in event
        ):
            failure_type = "state_uncertain"
        if failure_type is not None:
            result = {
                "stage": STAGE_BY_FAILURE[failure_type],
                "event_index": event_index,
                "failure_type": failure_type,
                "detection_id": event.get("detection_id"),
                "stack_level": event.get("stack_level"),
            }
            if result["detection_id"] is None:
                result.pop("detection_id")
            if result["stack_level"] is None:
                result.pop("stack_level")
            return result

    if plan is None and report is None:
        return None
    if len(events) < len(picks):
        result = {
            "stage": "execution",
            "event_index": len(events),
            "failure_type": "execution_crash",
        }
        if len(events) < len(picks):
            result["detection_id"] = picks[len(events)].get("detection_id")
        return result
    if report.get("task_complete") is True and len(events) == len(picks):
        return {
            "stage": "complete",
            "event_index": None,
            "failure_type": "none",
        }
    return {
        "stage": "execution",
        "event_index": None,
        "failure_type": "incomplete",
    }


def classify(events, plan=None, report=None):
    """Compatibility-friendly alias for the full execution classifier."""
    return classify_failure(events, plan, report)


def square_yaw(degrees):
    """Fold a square's orientation into its 90-degree symmetry interval."""
    return (float(degrees) + 45.0) % 90.0 - 45.0


def _with_grasp(pick, xy, yaw_deg):
    angle = math.radians(yaw_deg)
    updated = dict(pick)
    updated["xy"] = [float(xy[0]), float(xy[1])]
    updated["yaw_deg"] = float(yaw_deg)
    updated["spread_dir"] = [-math.sin(angle), math.cos(angle), 0.0]
    return updated


def with_grasp(pick, xy, yaw_deg):
    """Public pure helper to update a pick pose and its aligned wrist direction."""
    return _with_grasp(pick, xy, yaw_deg)


def next_action(failure, retries_done, max_retries=MAX_PICKUP_RETRIES):
    """Choose one bounded action rung; unsafe failure types always stop."""
    if not isinstance(failure, dict):
        raise TypeError("failure must be a dictionary")
    if isinstance(max_retries, bool) or not isinstance(max_retries, int):
        raise TypeError("max_retries must be an integer")
    if not 0 <= max_retries <= MAX_PICKUP_RETRIES:
        raise ValueError(f"max_retries must be between 0 and {MAX_PICKUP_RETRIES}")
    if isinstance(retries_done, bool) or not isinstance(retries_done, int):
        raise TypeError("retries_done must be an integer")
    if retries_done < 0:
        raise ValueError("retries_done cannot be negative")

    failure_type = failure.get("failure_type")
    if failure_type == "none":
        return "done"
    if failure_type not in RETRYABLE_FAILURES or retries_done >= max_retries:
        return "stop"
    rung = failure.get("recovery_rungs_used", 0)
    if isinstance(rung, bool) or not isinstance(rung, int) or rung < 0:
        raise ValueError("recovery_rungs_used must be a nonnegative integer")
    if rung >= len(RETRY_LADDER):
        return "stop"
    return RETRY_LADDER[rung]


def apply_action(plan, failure, action, image=None, cam=None):
    """Return a modified plan copy or ``None`` when an action cannot be used.

    ``remeasure`` fits the cube to the saved table image. It is an image-only
    re-analysis, not a fresh observation of the state after the failed run.
    """
    if not isinstance(plan, dict) or not isinstance(failure, dict):
        raise TypeError("plan and failure must be dictionaries")
    picks = plan.get("picks")
    if not isinstance(picks, list):
        raise ValueError("plan.picks must be a list")
    index = failure.get("event_index")
    if isinstance(index, bool) or not isinstance(index, int):
        return None
    if not 0 <= index < len(picks):
        return None
    if failure.get("failure_type") not in RETRYABLE_FAILURES:
        return None

    old_pick = picks[index]
    if not isinstance(old_pick, dict) or old_pick.get("category") != "cube":
        return None
    if (
        failure.get("detection_id") is not None
        and old_pick.get("detection_id") != failure["detection_id"]
    ):
        raise ValueError("Event and plan disagree about which cube failed")
    xy = old_pick.get("xy")
    if (
        not isinstance(xy, (list, tuple))
        or len(xy) != 2
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in xy)
        or not all(math.isfinite(v) for v in xy)
    ):
        raise ValueError("planned cube pick must have two finite XY coordinates")
    yaw = old_pick.get("yaw_deg")
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        raise ValueError("planned cube pick must have a finite yaw_deg")

    if action == "remeasure":
        if image is None or cam is None:
            return None
        from cube_pose import refine_pose

        measured_xy, measured_yaw, fit = refine_pose(
            image,
            cam,
            {"colour": old_pick["colour"]},
            xy,
            yaw,
            search_half_width=45,
        )
        if not math.isfinite(float(fit)) or fit < MIN_POSE_FIT:
            return None
        new_pick = _with_grasp(old_pick, measured_xy, square_yaw(measured_yaw))
        note = {"action": action, "pose_fit": round(float(fit), 3)}
    elif action == "flip_spread":
        new_pick = _with_grasp(old_pick, xy, float(yaw) + 90.0)
        note = {"action": action}
    elif action == "nudge":
        spread = old_pick.get("spread_dir")
        if (
            not isinstance(spread, (list, tuple))
            or len(spread) < 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in spread[:2])
            or not all(math.isfinite(v) for v in spread[:2])
        ):
            raise ValueError("planned cube pick must have a finite spread_dir")
        magnitude = math.hypot(spread[0], spread[1])
        if magnitude <= 1e-9:
            raise ValueError("planned spread_dir must be nonzero")
        offset = [NUDGE_M * spread[0] / magnitude, NUDGE_M * spread[1] / magnitude]
        new_pick = _with_grasp(old_pick, [xy[0] + offset[0], xy[1] + offset[1]], yaw)
        note = {"action": action, "nudge_m": offset}
    else:
        raise ValueError(f"Unknown recovery action {action!r}")

    new_plan = deepcopy(plan)
    new_plan["picks"][index] = new_pick
    history = plan.get("recovery_history", [])
    if not isinstance(history, list):
        raise ValueError("plan.recovery_history must be a list")
    note.update(
        pick_index=index,
        detection_id=old_pick.get("detection_id"),
        new_xy=new_pick["xy"],
        new_yaw_deg=new_pick["yaw_deg"],
    )
    new_plan["recovery_history"] = [*history, note]
    return new_plan


def decide(
    events,
    plan,
    report,
    image=None,
    cam=None,
    max_retries=MAX_PICKUP_RETRIES,
):
    """Classify a completed attempt and propose the next bounded action."""
    if not isinstance(plan, dict):
        raise TypeError("plan must be a dictionary")
    failure = classify_failure(events, plan, report)
    if failure["failure_type"] == "none":
        return {"failure": failure, "action": "done", "plan": None, "skipped": []}

    failure["recovery_rungs_used"] = plan.get("recovery_rungs_used", 0)
    retries_done = plan.get("recovery_retries", 0)
    if isinstance(retries_done, bool) or not isinstance(retries_done, int):
        raise ValueError("plan.recovery_retries must be a nonnegative integer")
    if retries_done < 0:
        raise ValueError("plan.recovery_retries must be a nonnegative integer")
    skipped = []
    while True:
        action = next_action(failure, retries_done, max_retries)
        if action in ("done", "stop"):
            return {"failure": failure, "action": action, "plan": None, "skipped": skipped}
        candidate = apply_action(plan, failure, action, image, cam)
        rung = plan.get("recovery_rungs_used", 0) + len(skipped) + 1
        if candidate is not None:
            candidate["recovery_rungs_used"] = rung
            candidate["recovery_retries"] = retries_done + 1
            return {
                "failure": failure,
                "action": action,
                "plan": candidate,
                "skipped": skipped,
            }
        skipped.append(action)
        failure = {**failure, "recovery_rungs_used": rung}
        if rung >= len(RETRY_LADDER):
            return {
                "failure": failure,
                "action": "stop",
                "plan": None,
                "skipped": skipped,
            }


def propose_recovery_action(
    plan, failure, retry_number=1, max_retries=MAX_PICKUP_RETRIES
):
    """Backward-compatible proposal for the original bounded world-X nudge."""
    if not isinstance(plan, dict) or not isinstance(failure, dict):
        raise TypeError("plan and failure must be dictionaries")
    if not isinstance(max_retries, int) or isinstance(max_retries, bool):
        raise TypeError("max_retries must be an integer")
    if not 0 <= max_retries <= MAX_PICKUP_RETRIES:
        raise ValueError(f"max_retries must be between 0 and {MAX_PICKUP_RETRIES}")
    if not isinstance(retry_number, int) or isinstance(retry_number, bool):
        raise TypeError("retry_number must be an integer")
    if retry_number < 1:
        raise ValueError("retry_number must be one-based")
    if retry_number > max_retries:
        return {
            "action": "stop_and_review",
            "reason": "pickup_retry_limit_reached",
            "requires_operator_review": True,
            "retry_number": retry_number,
            "max_retries": max_retries,
        }
    if failure.get("failure_type") != "pickup_failed":
        return {
            "action": "stop_and_review",
            "reason": failure.get("failure_type", "unclassified_failure"),
            "requires_operator_review": True,
        }
    index = failure.get("event_index")
    if isinstance(index, bool) or not isinstance(index, int):
        raise ValueError("pickup failure must contain an integer event_index")
    picks = plan.get("picks")
    if not isinstance(picks, list) or not 0 <= index < len(picks):
        raise ValueError("failure event_index does not identify a planned pick")
    pick = picks[index]
    if not isinstance(pick, dict) or pick.get("category") != "cube":
        return {
            "action": "stop_and_review",
            "reason": "automatic_retry_prototype_supports_cubes_only",
            "requires_operator_review": True,
        }
    if (
        failure.get("detection_id") is not None
        and pick.get("detection_id") != failure["detection_id"]
    ):
        raise ValueError("failure detection_id does not match the indexed planned pick")
    xy = pick.get("xy")
    if (
        not isinstance(xy, (list, tuple))
        or len(xy) != 2
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in xy)
        or not all(math.isfinite(v) for v in xy)
    ):
        raise ValueError("planned cube pick must have two finite XY coordinates")
    offset = (NUDGE_M, 0.0) if retry_number == 1 else (-NUDGE_M, 0.0)
    candidate = deepcopy(plan)
    candidate["picks"][index]["xy"] = [xy[0] + offset[0], xy[1] + offset[1]]
    return {
        "action": "propose_pickup_retry",
        "retry_number": retry_number,
        "max_retries": max_retries,
        "reason": "pickup_failed",
        "nudge_xy_m": list(offset),
        "candidate_plan": candidate,
        "requires_fresh_observation": True,
        "requires_motion_validation": True,
        "executed": False,
    }


def recovery_rate(logs):
    """Compute full-task recovery rate for runs with a first missed-grasp failure."""
    failures = [
        item
        for item in logs
        if item.get("first_failure", {}).get("failure_type") in RETRYABLE_FAILURES
    ]
    recovered = sum(item.get("final_complete") is True for item in failures)
    return {
        "first_attempt_failures": len(failures),
        "recovered": recovered,
        "rate": recovered / len(failures) if failures else None,
    }


def _load_json(path, required=False):
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"Required run artifact missing: {path}")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f"Invalid JSON artifact {path}: {exc}") from exc


def _read_observation(run):
    image_path, camera_path = run / "world_camera.png", run / "camera.json"
    if not image_path.is_file() or not camera_path.is_file():
        return None, None
    import cv2
    from backproject import CameraModel

    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"Could not decode captured image: {image_path}")
    return image, CameraModel(str(camera_path))


def _execute_simulation(run, isaac_python, command_runner=subprocess.run):
    sim_script = Path(__file__).resolve().with_name("sort_scene.py")
    if isaac_python is None:
        isaac = os.environ.get("ISAAC")
        if not isaac:
            raise ValueError("Set ISAAC or pass --isaac-python to run recovery")
        isaac_python = str(Path(isaac) / "python.sh")
    if not Path(isaac_python).is_file():
        raise FileNotFoundError(f"Isaac Python launcher not found: {isaac_python}")
    isaac_python = str(Path(isaac_python).expanduser().resolve())

    command = [
        str(isaac_python),
        str(sim_script),
        "--run",
        str(run),
        "--physics-only",
    ]
    result = command_runner(
        command,
        cwd=str(sim_script.parent),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    (run / "sort.log").write_text(result.stdout or "", encoding="utf-8")
    return result.returncode


def run_recovery(
    run_dir,
    isaac_python=None,
    max_retries=MAX_PICKUP_RETRIES,
    simulator=None,
):
    """Run bounded retry attempts in new run folders; never mutates prior reports."""
    if isinstance(max_retries, bool) or not isinstance(max_retries, int):
        raise TypeError("max_retries must be an integer")
    if not 0 <= max_retries <= MAX_PICKUP_RETRIES:
        raise ValueError(f"max_retries must be between 0 and {MAX_PICKUP_RETRIES}")
    base = Path(run_dir).expanduser().resolve()
    if not base.is_dir():
        raise FileNotFoundError(f"Run directory does not exist: {base}")
    if not (base / "sort_plan.json").is_file():
        raise FileNotFoundError(f"sort_plan.json missing from {base}")
    if simulator is None:
        def simulator(path):
            return _execute_simulation(path, isaac_python)

    current = base
    steps = []
    for attempt in range(max_retries + 1):
        report_path = current / "execution_report.json"
        simulator_return_code = None
        if not report_path.is_file():
            simulator_return_code = simulator(current)

        plan = _load_json(current / "sort_plan.json", required=True)
        events = _load_json(current / "execution_events.json") or []
        report = _load_json(report_path)
        if simulator_return_code not in (None, 0):
            report = None
        image, cam = _read_observation(current)
        result = decide(events, plan, report, image, cam, max_retries)
        decision = {key: value for key, value in result.items() if key != "plan"}
        (current / "recovery_decision.json").write_text(
            json.dumps(decision, indent=2) + "\n", encoding="utf-8"
        )
        steps.append(
            {
                "run": current.name,
                "failure": result["failure"],
                "action": result["action"],
                "skipped": result["skipped"],
                "simulator_return_code": simulator_return_code,
            }
        )

        if result["action"] == "done":
            outcome = "complete"
            break
        if result["action"] == "stop" or attempt >= max_retries:
            outcome = "stopped"
            break

        retry_dir = Path(f"{base}_retry{attempt + 1}")
        if retry_dir.exists():
            raise FileExistsError(
                f"Recovery output already exists; inspect it or choose a fresh base run: {retry_dir}"
            )
        from sort_scene import validate_plan

        validate_plan(result["plan"])
        retry_dir.mkdir(parents=True)
        for name in COPY_TO_RETRY:
            source = current / name
            if source.is_file():
                shutil.copy2(source, retry_dir / name)
        (retry_dir / "sort_plan.json").write_text(
            json.dumps(result["plan"], indent=2) + "\n", encoding="utf-8"
        )
        current = retry_dir
    else:
        outcome = "stopped"

    first_failure = steps[0]["failure"] if steps else None
    log = {
        "base_run": base.name,
        "attempts": len(steps),
        "retries": max(0, len(steps) - 1),
        "steps": steps,
        "first_failure": first_failure,
        "final_complete": outcome == "complete",
        "outcome": outcome,
    }
    (base / "recovery_log.json").write_text(
        json.dumps(log, indent=2) + "\n", encoding="utf-8"
    )
    return log


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, help="existing run directory with sort_plan.json")
    parser.add_argument("--isaac-python", help="Isaac Sim python.sh launcher")
    parser.add_argument(
        "--max-retries",
        type=int,
        default=MAX_PICKUP_RETRIES,
        help=f"maximum pickup retries (0-{MAX_PICKUP_RETRIES})",
    )
    parser.add_argument("--summarize", type=Path, help="print an existing recovery_log.json")
    parser.add_argument("--rate", nargs="+", type=Path, help="compute recovery rate from run dirs")
    args = parser.parse_args(argv)

    if args.rate:
        logs = [
            _load_json(run / "recovery_log.json")
            for run in args.rate
        ]
        print(json.dumps(recovery_rate([log for log in logs if log]), indent=2))
        return 0
    if args.summarize:
        log = _load_json(args.summarize / "recovery_log.json", required=True)
        print(json.dumps(log, indent=2))
        return 0
    if args.run is None:
        parser.error("--run is required")
    result = run_recovery(
        args.run,
        isaac_python=args.isaac_python,
        max_retries=args.max_retries,
    )
    print(json.dumps(result, indent=2))
    return 0 if result["final_complete"] else EXIT_STOP


if __name__ == "__main__":
    raise SystemExit(main())
