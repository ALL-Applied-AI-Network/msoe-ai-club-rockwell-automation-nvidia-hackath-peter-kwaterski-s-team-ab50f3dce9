"""Host-side bounded recovery for one step inside a live Isaac session."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from recovery_policy import decide
from run_artifacts import atomic_write_json, read_json


def _observation(run_dir: Path):
    image_path = run_dir / "mid_step_capture.png"
    if not image_path.is_file():
        image_path = run_dir / "world_camera.png"
    camera_path = run_dir / "camera.json"
    if not image_path.is_file() or not camera_path.is_file():
        return None, None
    import cv2
    from backproject import CameraModel

    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"Could not decode image: {image_path}")
    return image, CameraModel(str(camera_path))


def run_live_recovery(run_dir: Path) -> dict:
    run_dir = Path(run_dir).expanduser().resolve()
    plan = read_json(run_dir / "sort_plan.json")
    if not isinstance(plan, dict):
        raise FileNotFoundError("sort_plan.json missing or invalid")
    events = read_json(run_dir / "execution_events.json") or []
    if not isinstance(events, list):
        raise ValueError("execution_events.json must be a list")
    partial_report = {
        "task_complete": False,
        "abort_reason": None,
        "verified_pickups": sum(1 for e in events if e.get("pickup_verified")),
        "verified_placements": sum(1 for e in events if e.get("placement_verified")),
    }
    image, cam = _observation(run_dir)
    result = decide(events, plan, partial_report, image, cam)
    decision = {key: value for key, value in result.items() if key != "plan"}
    atomic_write_json(run_dir / "live_recovery_decision.json", decision)
    if result.get("plan") is not None:
        from sort_scene import validate_plan

        validate_plan(result["plan"])
        atomic_write_json(run_dir / "sort_plan.json", result["plan"])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run_live_recovery(args.run)
    print(json.dumps({k: v for k, v in result.items() if k != "plan"}, indent=2))
    action = result.get("action")
    if action == "done":
        return 0
    if action == "stop":
        return 1
    if result.get("plan") is not None:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
