#!/usr/bin/env python3
"""Aggregate rollout metrics for hackathon presentations.

Usage:
  uv run python report_metrics.py
  uv run python report_metrics.py --artifacts ./artifacts --out presentation_kpis.md
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from screw_policy.metrics import aggregate_rollouts, presentation_markdown


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--artifacts",
        default=os.environ.get("REAL_ROBOT_ARTIFACT_DIR", "artifacts"),
        help="Rollout artifact root",
    )
    p.add_argument(
        "--out",
        default=None,
        help="Write combined markdown report to this path",
    )
    args = p.parse_args()
    root = Path(args.artifacts)
    metric_files = sorted(root.glob("*/metrics.json"))
    session = aggregate_rollouts(metric_files)

    print("# Physical robot — session KPIs\n")
    if session.get("rollout_count", 0) == 0:
        print("No rollout metrics found. Run the policy on mock or the real cell first.")
        return

    print(f"- Rollouts recorded: **{session['rollout_count']}**")
    print(
        f"- Full task success: **{session['task_complete_count']}** "
        f"({100 * session['task_complete_rate']:.0f}%)"
    )
    sd = session["screws_dropped"]
    print(
        f"- Screws placed per rollout: mean **{sd['mean']:.1f}**, "
        f"median **{sd['median']:.1f}**, best **{sd['max']}** / 8"
    )
    sr = session["success_rate"]
    print(f"- Task progress (drops/8): mean **{100 * sr['mean']:.0f}%**")
    if session.get("pick_cycle_s_mean") is not None:
        print(f"- Mean pick–place cycle: **{session['pick_cycle_s_mean']:.1f} s**")
    if session.get("act_latency_ms_p50") is not None:
        print(f"- Policy act() latency (avg p50): **{session['act_latency_ms_p50']:.0f} ms**")
    if session.get("cosmos_latency_ms_p50") is not None:
        print(f"- Cosmos latency (avg p50): **{session['cosmos_latency_ms_p50']:.0f} ms**")
    print()

    lines = ["# Physical robot — session KPIs\n"]
    lines.append(json.dumps(session, indent=2))
    lines.append("\n\n## Per-rollout summaries\n")
    for mf in metric_files:
        m = json.loads(mf.read_text())
        lines.append(presentation_markdown(m, title=f"Rollout {m.get('trial_id')}"))
        lines.append("\n---\n")

    if args.out:
        Path(args.out).write_text("\n".join(lines))
        print(f"Wrote {args.out}")
    else:
        for mf in metric_files[-3:]:
            m = json.loads(mf.read_text())
            print(presentation_markdown(m))
            print()

    session_path = root / "session_summary.json"
    session_path.write_text(json.dumps(session, indent=2))
    print(f"Session aggregate: {session_path}")


if __name__ == "__main__":
    main()
