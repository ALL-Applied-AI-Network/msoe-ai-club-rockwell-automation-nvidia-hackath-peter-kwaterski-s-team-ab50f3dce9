# Metrics for presentations

Every physical-robot rollout (mock or real cell) writes metrics under `artifacts/<trial>_<timestamp>/`.

## Headline KPIs (for slides)

| Metric | Field | Talking point |
|--------|--------|----------------|
| Task success | `task_complete`, `success_rate` | All 8 screws in the bin |
| Throughput | `screws_dropped`, `pick_cycle_s.mean` | Placements per rollout and seconds per pick |
| Vision stack | `perception.source_counts`, `localization_events` | Cosmos vs depth vs fallback mix |
| Latency | `act_latency_ms`, `cosmos_latency_ms` | Closed-loop responsiveness on your GPU |
| Reliability | `event_counts`, session `task_complete_rate` | Consistency across 10 hackathon rollouts |

## Files

- **`metrics.json`** — machine-readable; import into Excel or a chart tool.
- **`PRESENTATION.md`** — copy-paste bullets for one rollout.
- **`steps.jsonl`** — optional deep dive (phase timeline, TCP waypoints).
- **`artifacts/session_summary.json`** — aggregate after multiple rollouts.

## Commands

```bash
# After your slot or mock rehearsal
uv run python report_metrics.py
uv run python report_metrics.py --out docs/presentation_kpis.md
```

Compare sim vs real: cite Isaac cube-stack videos separately; use these metrics for **Track 3 physical** evidence.

## Notes

- Organizer **My results** scores are authoritative for the leaderboard; these metrics explain *how* your stack performed (perception, cycles, latency).
- Do not commit `.request.json` or artifact folders with private keys.
