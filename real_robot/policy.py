"""Team policy: pick all eight default-scene screws into the yellow bin."""

from __future__ import annotations

import os
import time

import numpy as np

from screw_policy.artifacts import RolloutLogger
from screw_policy.config import load_config
from screw_policy.cosmos_client import CosmosClient
from screw_policy.state_machine import ScrewController


class Policy:
    CAMERAS = ["table"]
    IMAGE_SIZE = (640, 360)
    IMAGE_FORMAT = "png"
    ACTION = "tcp"
    PREFETCH = False
    DEPTH = ["table"]
    DT = 0.15

    def __init__(self, checkpoint: str | None = None):
        del checkpoint  # Cosmos weights come from COSMOS3_NANO_PATH / HF cache
        self.cfg = load_config()
        self.DT = self.cfg.action_dt
        self.controller = ScrewController(self.cfg)
        self.logger = RolloutLogger()
        self._cosmos = CosmosClient()
        self._use_cosmos = (
            os.environ.get("REAL_ROBOT_ENABLE_COSMOS", "1").strip().lower()
            not in ("0", "false", "no")
            and self._cosmos.available()
        )
        self._trial_started = False
        self._last_trial_running = False
        if self._use_cosmos:
            try:
                self._cosmos.start()
            except Exception:
                self._use_cosmos = False

    def reset(self) -> None:
        self._finalize_if_needed("trial_reset")
        self.controller.reset()
        self._trial_started = False
        self._last_trial_running = False

    def _finalize_if_needed(self, reason: str) -> None:
        if not self.logger.active:
            return
        st = self.controller.state
        complete = st.drops >= self.cfg.expected_screws
        if reason == "trial_reset" and not complete:
            reason = st.last_event if st.last_event not in ("reset",) else "trial_ended"
        self.logger.finalize(
            end_reason=reason,
            task_complete=complete,
            screws_dropped=st.drops,
        )

    def _maybe_start_rollout(self, obs: dict) -> None:
        trial = obs.get("trial")
        running = bool(trial and trial.get("status") == "running")
        if self._last_trial_running and not running:
            self._finalize_if_needed("trial_ended")
            self._trial_started = False
        self._last_trial_running = running
        if self._trial_started or not running:
            return
        self.logger.start_rollout(
            trial,
            expected_screws=self.cfg.expected_screws,
            cosmos_enabled=self._use_cosmos,
        )
        self._trial_started = True

    def _cosmos_parse(self, obs: dict) -> tuple[dict | None, float | None]:
        if not self._use_cosmos:
            return None, None
        table = (obs.get("images") or {}).get("table")
        if table is None:
            return None, None
        t0 = time.perf_counter()
        try:
            parsed = self._cosmos.infer_rgb(np.asarray(table, dtype=np.uint8))
            return parsed, (time.perf_counter() - t0) * 1000.0
        except Exception:
            return None, (time.perf_counter() - t0) * 1000.0

    def act(self, obs: dict):
        self._maybe_start_rollout(obs)
        t0 = time.perf_counter()
        cosmos_parsed, cosmos_ms = self._cosmos_parse(obs)
        chunk, info = self.controller.step(
            obs,
            cosmos_parsed,
            image_wh=self.IMAGE_SIZE,
        )
        act_ms = (time.perf_counter() - t0) * 1000.0
        row = chunk[0].tolist() if len(chunk) == 1 else None
        self.logger.log_step(
            info,
            chunk.shape,
            act_ms=act_ms,
            cosmos_ms=cosmos_ms,
            tcp_row=row,
        )
        if (
            self.logger.active
            and self.controller.state.drops >= self.cfg.expected_screws
        ):
            self._finalize_if_needed("task_complete")
        if len(chunk) == 1:
            return chunk[0]
        return chunk
