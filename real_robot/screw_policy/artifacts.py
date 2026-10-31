"""Per-rollout logging and presentation metrics (no secrets)."""

from __future__ import annotations

import json
import os
from pathlib import Path

from screw_policy.metrics import (
    PickRecord,
    RolloutMetrics,
    finalize_act_latencies,
    presentation_markdown,
    utc_now,
)


class RolloutLogger:
    def __init__(self, root: Path | None = None):
        base = root or Path(
            os.environ.get(
                "REAL_ROBOT_ARTIFACT_DIR",
                Path(__file__).resolve().parent.parent / "artifacts",
            )
        )
        self.root = base
        self.run_dir: Path | None = None
        self._step = 0
        self._metrics: RolloutMetrics | None = None
        self._act_ms: list[float] = []
        self._cosmos_ms: list[float] = []
        self._pick_t0: float | None = None
        self._pending_pick: dict | None = None

    @property
    def active(self) -> bool:
        return self._metrics is not None

    def start_rollout(self, trial: dict | None, *, expected_screws: int, cosmos_enabled: bool) -> None:
        from datetime import datetime, timezone

        tid = (trial or {}).get("trial_id", "unknown")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.run_dir = self.root / f"{tid}_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._step = 0
        self._act_ms = []
        self._cosmos_ms = []
        self._pick_t0 = None
        self._pending_pick = None
        started = utc_now()
        self._metrics = RolloutMetrics(
            trial_id=tid,
            started_utc=started,
            expected_screws=expected_screws,
            cosmos_enabled=cosmos_enabled,
        )
        self._write(
            "rollout.json",
            {"trial": trial, "started": started, "expected_screws": expected_screws},
        )

    def log_step(
        self,
        info: dict,
        chunk_shape: tuple,
        *,
        act_ms: float,
        cosmos_ms: float | None,
        tcp_row: list[float] | None = None,
    ) -> None:
        if not self.run_dir or not self._metrics:
            return
        m = self._metrics
        m.total_act_calls += 1
        rows = chunk_shape[0] if chunk_shape else 1
        m.total_actions_queued += int(rows)
        self._act_ms.append(act_ms)
        if cosmos_ms is not None:
            self._cosmos_ms.append(cosmos_ms)

        phase = info.get("phase", "")
        event = info.get("event", "")
        m.phase_counts[phase] = m.phase_counts.get(phase, 0) + 1
        m.event_counts[event] = m.event_counts.get(event, 0) + 1

        if info.get("localized"):
            m.localization_events += 1
            if info.get("perception"):
                m.perception = dict(info["perception"])
            tgt = info.get("target") or {}
            self._pending_pick = tgt
            import time

            self._pick_t0 = time.perf_counter()

        if event == "dropped":
            tgt = info.get("target") or self._pending_pick
            if tgt is None:
                tgt = {}
            import time

            cycle = (time.perf_counter() - self._pick_t0) if self._pick_t0 else None
            m.picks.append(
                PickRecord(
                    pick_index=len(m.picks) + 1,
                    x_mm=float(tgt.get("x", 0)),
                    y_mm=float(tgt.get("y", 0)),
                    yaw_deg=float(tgt.get("yaw", 0)),
                    confidence=float(tgt.get("confidence", 0)),
                    source=str(tgt.get("source", "unknown")),
                    cycle_s=cycle,
                    retries=int(info.get("retries", 0)),
                )
            )
            self._pending_pick = None
            self._pick_t0 = None

        m.screws_dropped = int(info.get("drops", m.screws_dropped))

        self._step += 1
        row = {
            "step": self._step,
            "act_ms": round(act_ms, 2),
            "cosmos_ms": round(cosmos_ms, 2) if cosmos_ms is not None else None,
            "info": info,
            "chunk_shape": list(chunk_shape),
        }
        if tcp_row is not None:
            row["tcp"] = [round(v, 3) for v in tcp_row]
        self._append("steps.jsonl", row)

    def finalize(self, *, end_reason: str, task_complete: bool, screws_dropped: int) -> Path | None:
        if not self.run_dir or not self._metrics:
            return None
        m = self._metrics
        m.ended_utc = utc_now()
        m.end_reason = end_reason
        m.task_complete = task_complete
        m.screws_dropped = screws_dropped
        m.act_latency_ms = finalize_act_latencies(self._act_ms)
        m.cosmos_latency_ms = finalize_act_latencies(self._cosmos_ms)
        payload = m.to_dict()
        self._write("metrics.json", payload)
        self._write("PRESENTATION.md", presentation_markdown(payload))
        self._append_session(payload)
        out = self.run_dir / "metrics.json"
        self._metrics = None
        self.run_dir = None
        return out

    def _append_session(self, rollout_metrics: dict) -> None:
        session_path = self.root / "session_rollouts.jsonl"
        self.root.mkdir(parents=True, exist_ok=True)
        with session_path.open("a") as f:
            f.write(json.dumps(rollout_metrics) + "\n")

    def _write(self, name: str, obj: dict | str) -> None:
        assert self.run_dir
        path = self.run_dir / name
        if isinstance(obj, str):
            path.write_text(obj)
        else:
            path.write_text(json.dumps(obj, indent=2))

    def _append(self, name: str, obj: dict) -> None:
        assert self.run_dir
        with (self.run_dir / name).open("a") as f:
            f.write(json.dumps(obj) + "\n")
