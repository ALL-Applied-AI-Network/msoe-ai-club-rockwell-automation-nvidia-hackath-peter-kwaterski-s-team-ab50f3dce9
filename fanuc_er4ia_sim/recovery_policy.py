"""Recovery policy for the FANUC cube tower (team guide, Problem D).

Retry happens ABOVE Isaac: a run fails -> we read its JSON -> write a corrected
sort_plan.json into a NEW run folder -> sort_scene.py --physics-only runs it.
run_with_recovery.sh drives that loop.

Part 1  classify(events, plan, report)        -> {stage, event_index, failure_type, ...}
Part 2  next_action(failure, retries_done)    -> "remeasure" | "flip_spread" | "nudge" | "stop" | "done"
        apply_action(plan, failure, action)   -> corrected copy of the plan (or None)

Every function above is PURE: no files are read or written and inputs are never
modified. Only main() at the bottom touches files.

Simulator truth note: pickup_verified / objects_lifted come from the simulator's
object tracking. Here they are used to DETECT a failure after a run, which is
fine in simulation. Retry grasp poses are computed only from the plan and the
camera image, never from simulator object poses (AGENTS.md rule).
"""

import copy
import json

import numpy as np

# --------------------------------------------------------------------------
# Part 1: detect and classify
# --------------------------------------------------------------------------
# failure_type      stage         meaning
# none              complete      every pick verified
# untracked_target  perception    no cube within 4 cm of the planned grasp
# ik_failure        reachability  arm could not follow the planned path
# missed_grasp      pickup        jaws closed and nothing came up
# wrong_object      pickup        something else came up (or more than the target)
# placement_failed  placement     cube picked but not set down at the right spot/pose
# support_failed    support       the tower underneath is not valid
# execution_crash   execution     sort_scene stopped without writing its report
# incomplete        execution     report says incomplete but no event explains why

RETRYABLE = {"missed_grasp"}


def classify_event(event):
    """Failure type of ONE pick event, or None if it succeeded."""
    if event.get("tracked_object") is None:
        return "untracked_target"
    if event.get("trajectory_ik_failures", 0) > 0:
        return "ik_failure"
    if not event.get("pickup_verified"):
        lifted = event.get("objects_lifted", [])
        return "missed_grasp" if not lifted else "wrong_object"
    if len(event.get("objects_lifted", [])) > 1:
        return "wrong_object"
    if not event.get("placement_verified"):
        return "placement_failed"
    if event.get("stack_support_verified") is False:  # absent in the plain sort task
        return "support_failed"
    return None


STAGE = {
    "untracked_target": "perception",
    "ik_failure": "reachability",
    "missed_grasp": "pickup",
    "wrong_object": "pickup",
    "placement_failed": "placement",
    "support_failed": "support",
}


def classify(events, plan=None, report=None):
    """Find the FIRST failure in a run (later failures are usually consequences).

    events: list from execution_events.json
    plan:   dict from sort_plan.json (optional; detects a run that stopped early)
    report: dict from execution_report.json, or None if the file is missing
    """
    for i, event in enumerate(events):
        failure = classify_event(event)
        if failure:
            return {
                "stage": STAGE[failure],
                "event_index": i,
                "failure_type": failure,
                "detection_id": event.get("detection_id"),
                "stack_level": event.get("stack_level"),
            }
    planned = len(plan["picks"]) if plan else len(events)
    if report is None or len(events) < planned:
        # Isaac can exit 0 after an exception; a missing report is a failure.
        return {"stage": "execution", "event_index": len(events),
                "failure_type": "execution_crash", "detection_id": None, "stack_level": None}
    if not report.get("task_complete", False):
        return {"stage": "execution", "event_index": None,
                "failure_type": "incomplete", "detection_id": None, "stack_level": None}
    return {"stage": "complete", "event_index": None, "failure_type": "none",
            "detection_id": None, "stack_level": None}


# --------------------------------------------------------------------------
# Part 2: choose and apply the next action
# --------------------------------------------------------------------------
# Order of retries for a missed grasp. Each retry builds on the previous plan.
LADDER = ("remeasure", "flip_spread", "nudge")
MAX_RETRIES = len(LADDER)
NUDGE_M = 0.002
MIN_POSE_FIT = 0.75  # same gate plan_sort.py uses for an image pose fit


def next_action(failure, retries_done):
    """What to do next. Bounded: never more than MAX_RETRIES, never retry unsafe failures."""
    if failure["failure_type"] == "none":
        return "done"
    if failure["failure_type"] not in RETRYABLE:
        return "stop"  # notebook 08: placement/support/perception problems need a human
    if retries_done >= MAX_RETRIES:
        return "stop"
    return LADDER[retries_done]


def square_yaw(degrees):
    """Same as cube_pose.square_yaw: a square looks identical every 90 degrees."""
    return (float(degrees) + 45) % 90 - 45


def with_grasp(pick, xy, yaw_deg):
    """Copy of a pick with a new grasp pose. spread_dir follows the rule
    sort_scene.validate_plan enforces: [-sin(yaw), cos(yaw), 0]."""
    theta = np.deg2rad(yaw_deg)
    new = dict(pick)
    new["xy"] = [float(xy[0]), float(xy[1])]
    new["yaw_deg"] = float(yaw_deg)
    new["spread_dir"] = [-float(np.sin(theta)), float(np.cos(theta)), 0.0]
    return new


def apply_action(plan, failure, action, image=None, cam=None):
    """Return a corrected COPY of the plan, or None if the action cannot be applied.

    remeasure    fit the cube's pose again in the table image, full +/-45 deg yaw search
                 (needs image + cam; uses only the picture, like plan_sort.py)
    flip_spread  grasp the other two faces: jaws turned 90 degrees
    nudge        move the grasp 2 mm along the jaw closing direction (blind, last resort)
    """
    i = failure["event_index"]
    if i is None or i >= len(plan["picks"]):
        return None
    new_plan = copy.deepcopy(plan)
    pick = new_plan["picks"][i]
    if failure.get("detection_id") is not None and pick["detection_id"] != failure["detection_id"]:
        raise ValueError("Event and plan disagree about which cube this is")
    xy = np.array(pick["xy"], float)
    yaw = float(pick["yaw_deg"])

    if action == "remeasure":
        if image is None or cam is None:
            return None
        from cube_pose import refine_pose  # pure: deterministic fit, no file access
        fit_xy, fit_yaw, fit = refine_pose(
            image, cam, {"colour": pick["colour"]}, xy, yaw, search_half_width=45)
        if fit < MIN_POSE_FIT:
            return None
        pick = with_grasp(pick, fit_xy, square_yaw(fit_yaw))
        note = {"action": action, "pose_fit": round(float(fit), 3)}
    elif action == "flip_spread":
        # Add 90 AFTER folding: square_yaw(yaw + 90) == square_yaw(yaw) would undo the flip.
        pick = with_grasp(pick, xy, square_yaw(yaw) + 90.0)
        note = {"action": action}
    elif action == "nudge":
        spread = np.array(pick["spread_dir"][:2], float)
        pick = with_grasp(pick, xy + NUDGE_M * spread / np.linalg.norm(spread), yaw)
        note = {"action": action, "nudge_mm": NUDGE_M * 1000}
    else:
        raise ValueError(f"Unknown action {action!r}")

    note.update(pick_index=i, detection_id=pick["detection_id"],
                new_xy=pick["xy"], new_yaw_deg=pick["yaw_deg"])
    new_plan["picks"][i] = pick
    new_plan["recovery_history"] = list(plan.get("recovery_history", [])) + [note]
    return new_plan


def decide(events, plan, report, image=None, cam=None):
    """Part 1 + Part 2 together. The plan remembers how far down the ladder we are
    (recovery_rungs_used), so a skipped rung (e.g. the re-measured fit is too poor)
    is never tried twice. Returns {"failure", "action", "plan" (new plan or None), "skipped"}."""
    failure = classify(events, plan, report)
    skipped = []
    step = plan.get("recovery_rungs_used", 0)
    while True:
        action = next_action(failure, step)
        if action in ("done", "stop"):
            return {"failure": failure, "action": action, "plan": None, "skipped": skipped}
        new_plan = apply_action(plan, failure, action, image, cam)
        if new_plan is not None:
            new_plan["recovery_rungs_used"] = step + 1
            return {"failure": failure, "action": action, "plan": new_plan, "skipped": skipped}
        skipped.append(action)
        step += 1


def recovery_rate(logs):
    """logs: list of recovery_log.json dicts. Recovered / first-attempt pickup failures."""
    failed = [g for g in logs if g["first_failure"]["failure_type"] in RETRYABLE]
    recovered = [g for g in failed if g["final_complete"]]
    return {"first_attempt_failures": len(failed), "recovered": len(recovered),
            "rate": len(recovered) / len(failed) if failed else None}


# --------------------------------------------------------------------------
# File wrapper (the only part with side effects). Used by run_with_recovery.sh.
# --------------------------------------------------------------------------
EXIT_NEW_PLAN, EXIT_COMPLETE, EXIT_STOP = 0, 10, 20
COPY_FILES = ("camera.json", "world_camera.png", "scene_state.json",
              "ground_truth.json", "detections.json")


def _read(path):
    return json.loads(path.read_text()) if path.is_file() else None


def main():
    import argparse
    import shutil
    import sys
    from pathlib import Path

    ap = argparse.ArgumentParser(description="Decide the next recovery step for a finished run.")
    ap.add_argument("--run", help="run folder that sort_scene.py already executed")
    ap.add_argument("--out", help="new run folder for the corrected plan")
    ap.add_argument("--summarize", metavar="BASE_RUN",
                    help="write BASE_RUN/recovery_log.json from BASE_RUN and BASE_RUN_retry*")
    ap.add_argument("--rate", nargs="+", metavar="BASE_RUN",
                    help="recovery rate over several base runs (after --summarize)")
    a = ap.parse_args()

    if a.rate:
        logs = [_read(Path(r) / "recovery_log.json") for r in a.rate]
        print(json.dumps(recovery_rate([g for g in logs if g]), indent=2))
        return 0

    if a.summarize:
        base = Path(a.summarize)
        chain = [base] + sorted(base.parent.glob(base.name + "_retry*"),
                                key=lambda p: int(p.name.rsplit("_retry", 1)[1]))
        steps = []
        for r in chain:
            plan = _read(r / "sort_plan.json")
            events = _read(r / "execution_events.json") or []
            report = _read(r / "execution_report.json")
            steps.append({"run": r.name, "failure": classify(events, plan, report),
                          "action_taken": (plan.get("recovery_history") or [None])[-1] if plan else None})
        log = {"base_run": base.name, "attempts": len(steps), "steps": steps,
               "first_failure": steps[0]["failure"],
               "final_complete": steps[-1]["failure"]["failure_type"] == "none"}
        (base / "recovery_log.json").write_text(json.dumps(log, indent=2) + "\n")
        print(json.dumps({k: log[k] for k in ("base_run", "attempts", "final_complete")}))
        return 0

    if not a.run:
        ap.error("--run is required")
    run = Path(a.run)
    plan = _read(run / "sort_plan.json")
    if plan is None:
        sys.exit(f"{run}/sort_plan.json missing")
    events = _read(run / "execution_events.json") or []
    report = _read(run / "execution_report.json")

    image = cam = None
    if (run / "world_camera.png").is_file() and (run / "camera.json").is_file():
        import cv2
        from backproject import CameraModel
        image = cv2.imread(str(run / "world_camera.png"))
        cam = CameraModel(str(run / "camera.json"))

    result = decide(events, plan, report, image, cam)
    decision = {k: v for k, v in result.items() if k != "plan"}
    (run / "recovery_decision.json").write_text(json.dumps(decision, indent=2) + "\n")
    f = result["failure"]
    print(f"[recovery] {run.name}: {f['failure_type']} at pick {f['event_index']} "
          f"(stage {f['stage']}) -> {result['action']}"
          + (f" (skipped {result['skipped']})" if result["skipped"] else ""))

    if result["action"] == "done":
        return EXIT_COMPLETE
    if result["action"] == "stop":
        return EXIT_STOP
    if not a.out:
        ap.error("--out is required when a retry is possible")
    out = Path(a.out)
    if out.exists():
        sys.exit(f"{out} already exists; use a fresh folder")
    out.mkdir(parents=True)
    for name in COPY_FILES:
        if (run / name).is_file():
            shutil.copy(run / name, out / name)
    (out / "sort_plan.json").write_text(json.dumps(result["plan"], indent=2) + "\n")
    return EXIT_NEW_PLAN


if __name__ == "__main__":
    raise SystemExit(main())
