"""Atomic run-directory writes shared by Isaac execution and host services."""

from __future__ import annotations

import json
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_text(path: Path, text: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(
        dir=path.parent, prefix=f"{path.name}.", suffix=".tmp"
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def atomic_write_json(path: Path, payload: Any) -> None:
    atomic_write_text(path, json.dumps(payload, indent=2) + "\n")


class LiveFramePublisher:
    """Write a throttled PNG snapshot supported by Tkinter PhotoImage."""

    def __init__(self, run_dir: Path, *, min_interval_s: float = 0.5):
        self.path = Path(run_dir) / "live_frame.png"
        self.min_interval_s = min_interval_s
        self._last_publish = 0.0

    def maybe_publish(self, image_bgr) -> None:
        now = time.monotonic()
        if now - self._last_publish < self.min_interval_s:
            return
        import cv2

        fd, tmp_path = tempfile.mkstemp(
            dir=self.path.parent, prefix=".live_frame.", suffix=".png"
        )
        try:
            os.close(fd)
            if not cv2.imwrite(tmp_path, image_bgr):
                raise RuntimeError("cv2.imwrite failed for live_frame.png")
            os.replace(tmp_path, self.path)
            self._last_publish = now
        except Exception:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise


def read_json(path: Path, default: Optional[Any] = None) -> Any:
    path = Path(path)
    if not path.is_file():
        return default
    return json.loads(path.read_text(encoding="utf-8"))
