"""Rollout metrics for hackathon reporting and presentations."""

from __future__ import annotations

import json
import statistics
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass
class PickRecord:
    pick_index: int
    x_mm: float
    y_mm: float
    yaw_deg: float
    confidence: float
    source: str
    cycle_s: float | None = None
    retries: int = 0


@dataclass
class RolloutMetrics:
    trial_id: str
    started_utc: str
    ended_utc: str | None = None
    expected_screws: int = 8
    screws_dropped: int = 0
    task_complete: bool = False
    end_reason: str = "unknown"
    cosmos_enabled: bool = False
    total_act_calls: int = 0
    total_actions_queued: int = 0
    act_latency_ms: dict[str, float] = field(default_factory=dict)
    cosmos_latency_ms: dict[str, float] = field(default_factory=dict)
    localization_events: int = 0
    perception: dict[str, Any] = field(default_factory=dict)
    picks: list[PickRecord] = field(default_factory=list)
    phase_counts: dict[str, int] = field(default_factory=dict)
    event_counts: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["success_rate"] = (
            self.screws_dropped / self.expected_screws if self.expected_screws else 0.0
        )
        cycles = [p.cycle_s for p in self.picks if p.cycle_s is not None]
        d["pick_cycle_s"] = {
            "mean": statistics.mean(cycles) if cycles else None,
            "median": statistics.median(cycles) if cycles else None,
            "min": min(cycles) if cycles else None,
            "max": max(cycles) if cycles else None,
            "count": len(cycles),
        }
        return d


def _percentiles(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p50": None, "p95": None, "max": None}
    s = sorted(values)
    n = len(s)

    def pct(p: float) -> float:
        i = min(n - 1, max(0, int(p * (n - 1))))
        return s[i]

    return {"p50": pct(0.5), "p95": pct(0.95), "max": s[-1]}


def finalize_act_latencies(samples_ms: list[float]) -> dict[str, float | None]:
    if not samples_ms:
        return {"mean": None, "p50": None, "p95": None, "max": None}
    pct = _percentiles(samples_ms)
    return {
        "mean": statistics.mean(samples_ms),
        "p50": pct["p50"],
        "p95": pct["p95"],
        "max": pct["max"],
    }


def aggregate_rollouts(paths: list[Path]) -> dict:
    """Combine per-rollout metrics.json files into session-level KPIs."""
    rollouts = []
    for p in paths:
        if p.is_file():
            rollouts.append(json.loads(p.read_text()))
    if not rollouts:
        return {"rollout_count": 0}

    complete = sum(1 for r in rollouts if r.get("task_complete"))
    dropped = [r.get("screws_dropped", 0) for r in rollouts]
    success_rates = [r.get("success_rate", 0) for r in rollouts]
    act_p50 = [
        (r.get("act_latency_ms") or {}).get("p50")
        for r in rollouts
        if (r.get("act_latency_ms") or {}).get("p50") is not None
    ]
    cosmos_p50 = [
        (r.get("cosmos_latency_ms") or {}).get("p50")
        for r in rollouts
        if (r.get("cosmos_latency_ms") or {}).get("p50") is not None
    ]
    cycle_means = [
        (r.get("pick_cycle_s") or {}).get("mean")
        for r in rollouts
        if (r.get("pick_cycle_s") or {}).get("mean") is not None
    ]

    return {
        "rollout_count": len(rollouts),
        "task_complete_count": complete,
        "task_complete_rate": complete / len(rollouts),
        "screws_dropped": {
            "mean": statistics.mean(dropped),
            "median": statistics.median(dropped),
            "max": max(dropped),
        },
        "success_rate": {
            "mean": statistics.mean(success_rates),
            "median": statistics.median(success_rates),
        },
        "act_latency_ms_p50": statistics.mean(act_p50) if act_p50 else None,
        "cosmos_latency_ms_p50": statistics.mean(cosmos_p50) if cosmos_p50 else None,
        "pick_cycle_s_mean": statistics.mean(cycle_means) if cycle_means else None,
        "trial_ids": [r.get("trial_id") for r in rollouts],
    }


def presentation_markdown(metrics: dict, title: str = "Physical robot rollout") -> str:
    """One-rollout summary suitable for slides."""
    lines = [
        f"# {title}",
        "",
        f"- **Trial:** {metrics.get('trial_id', 'n/a')}",
        f"- **Screws placed:** {metrics.get('screws_dropped', 0)} / {metrics.get('expected_screws', 8)} "
        f"({100 * metrics.get('success_rate', 0):.0f}% of task)",
        f"- **Task complete:** {'yes' if metrics.get('task_complete') else 'no'} ({metrics.get('end_reason', '')})",
        "",
        "## Perception",
    ]
    perc = metrics.get("perception") or {}
    lines.append(
        f"- Localization passes: {metrics.get('localization_events', 0)}; "
        f"targets seen (last pass): {perc.get('merged_target_count', 'n/a')}"
    )
    lines.append(f"- Source mix (last pass): {perc.get('source_counts', {})}")
    if perc.get("mean_confidence") is not None:
        lines.append(f"- Mean target confidence: {perc['mean_confidence']:.2f}")
    lines.append(f"- Cosmos enabled: {metrics.get('cosmos_enabled')}")
    cosmos = metrics.get("cosmos_latency_ms") or {}
    if cosmos.get("p50") is not None:
        lines.append(f"- Cosmos inference p50: {cosmos['p50']:.0f} ms")

    picks = metrics.get("picks") or []
    if picks:
        lines.extend(["", "## Pick log"])
        for p in picks[:8]:
            cyc = p.get("cycle_s")
            cyc_s = f"{cyc:.1f}s" if cyc is not None else "n/a"
            lines.append(
                f"- #{p.get('pick_index')}: ({p.get('x_mm', 0):.0f}, {p.get('y_mm', 0):.0f}) mm, "
                f"{p.get('source')}, conf {p.get('confidence', 0):.2f}, cycle {cyc_s}"
            )

    lines.extend(["", "## Control"])
    act = metrics.get("act_latency_ms") or {}
    if act.get("p50") is not None:
        lines.append(f"- Policy `act()` latency p50: {act['p50']:.0f} ms (mean {act.get('mean', 0):.0f} ms)")
    lines.append(f"- TCP commands sent: {metrics.get('total_actions_queued', 0)} "
                 f"({metrics.get('total_act_calls', 0)} act calls)")
    pc = metrics.get("pick_cycle_s") or {}
    if pc.get("mean") is not None:
        lines.append(
            f"- Pick–place cycle: mean {pc['mean']:.1f} s, median {pc.get('median', 0):.1f} s "
            f"({pc.get('count', 0)} screws)"
        )
    lines.append("")
    return "\n".join(lines)


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
