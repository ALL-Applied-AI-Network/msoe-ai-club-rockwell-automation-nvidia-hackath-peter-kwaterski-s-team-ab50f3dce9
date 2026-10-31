"""Subprocess client for the persistent Cosmos worker."""

from __future__ import annotations

import base64
import json
import os
import subprocess
import threading
from pathlib import Path

import cv2
import numpy as np


class CosmosClient:
    def __init__(self, python: str | None = None, timeout_s: float = 120.0):
        self.timeout_s = timeout_s
        self.python = python or os.environ.get("REAL_ROBOT_COSMOS_PYTHON", "")
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()
        self._next_id = 0

    def available(self) -> bool:
        return bool(self.python) and Path(self.python).is_file()

    def start(self) -> None:
        if not self.available():
            raise RuntimeError("REAL_ROBOT_COSMOS_PYTHON not set or missing")
        if self._proc is not None and self._proc.poll() is None:
            return
        env = os.environ.copy()
        here = Path(__file__).resolve().parent.parent
        env["PYTHONPATH"] = str(here) + (
            os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
        )
        self._proc = subprocess.Popen(
            [self.python, "-m", "screw_policy.cosmos_worker"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            cwd=str(here),
            env=env,
        )

    def close(self) -> None:
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
        self._proc = None

    def infer_rgb(self, rgb: np.ndarray) -> dict:
        self.start()
        assert self._proc and self._proc.stdin and self._proc.stdout
        ok, buf = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if not ok:
            raise RuntimeError("failed to encode RGB for Cosmos")
        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        with self._lock:
            self._next_id += 1
            rid = self._next_id
            req = json.dumps({"id": rid, "rgb_b64": b64}) + "\n"
            self._proc.stdin.write(req)
            self._proc.stdin.flush()
            line = self._proc.stdout.readline()
        if not line:
            raise RuntimeError("cosmos worker closed stdout")
        resp = json.loads(line)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error", "cosmos inference failed"))
        return resp.get("parsed") or {"screws": [], "bin": None}
