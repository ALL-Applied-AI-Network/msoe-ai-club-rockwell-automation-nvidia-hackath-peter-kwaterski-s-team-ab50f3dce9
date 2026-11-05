#!/usr/bin/env python3
"""Preflight checks before a physical-robot slot."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _ok(msg: str) -> None:
    print(f"  OK  {msg}")


def _fail(msg: str) -> None:
    print(f"  FAIL  {msg}", file=sys.stderr)


def main() -> int:
    print("Real-robot preflight")
    errors = 0

    calib = HERE / "calibration" / "cameras.json"
    if calib.is_file():
        _ok(f"calibration present: {calib}")
    else:
        _fail(f"missing {calib}")
        errors += 1

    profile = HERE / "config" / "screw_task.yaml"
    if profile.is_file():
        _ok(f"task profile: {profile}")
    else:
        _fail(f"missing {profile}")
        errors += 1

    cosmos_py = os.environ.get("REAL_ROBOT_COSMOS_PYTHON", "")
    if cosmos_py and Path(cosmos_py).is_file():
        _ok(f"Cosmos python: {cosmos_py}")
    else:
        print("  WARN  REAL_ROBOT_COSMOS_PYTHON not set; policy uses fallback screw grid")

    if shutil.which("uv"):
        _ok("uv found")
    else:
        _fail("uv not found (install for client package)")
        errors += 1

    try:
        from policy import Policy  # noqa: WPS433

        p = Policy()
        _ok(f"Policy loads (cosmos={'on' if p._use_cosmos else 'off'})")
    except Exception as e:  # noqa: BLE001
        _fail(f"Policy import/init: {e}")
        errors += 1

    if errors == 0 and shutil.which("uv"):
        print("Running organizer mock (1 trial, 45s) ...")
        env = os.environ.copy()
        env["REAL_ROBOT_ENABLE_COSMOS"] = "0"
        r = subprocess.run(
            ["uv", "run", "run.py", "--mock", "--trials", "1", "--trial-s", "45",
             "--instruction", "Pick all eight screws and place them in the yellow bin."],
            cwd=str(HERE),
            env=env,
            capture_output=True,
            text=True,
            timeout=180,
        )
        if r.returncode == 0:
            _ok("mock cell check passed")
        else:
            _fail("mock cell check failed")
            print(r.stdout[-2000:] if r.stdout else "")
            print(r.stderr[-2000:] if r.stderr else "")
            errors += 1

    print()
    if errors:
        print(f"Preflight failed ({errors} issue(s)).")
        return 1
    print("Preflight passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
