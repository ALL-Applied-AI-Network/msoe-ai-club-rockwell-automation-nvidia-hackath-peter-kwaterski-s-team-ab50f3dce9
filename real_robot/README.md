# Real robot client (Track 3)

Self-contained package for the [Physical AI Hackathon 2026](https://github.com/SIDCLab/PhysicalAI-Hackathon-2026) cell. The six-cube Isaac Sim demo in `../fanuc_er4ia_sim/` is unchanged.

## Setup

```bash
cd real_robot
uv sync
```

Optional Cosmos (same GPU machine as simulation):

```bash
export REAL_ROBOT_COSMOS_PYTHON=/path/to/cosmos-venv/bin/python
export COSMOS3_NANO_PATH=/path/to/Cosmos3-Nano/snapshot
export REAL_ROBOT_ENABLE_COSMOS=1
```

Without Cosmos, the policy uses calibrated depth (when present) plus a configured fallback screw grid for mock/API checks.

## Commands

```bash
uv run python preflight.py
uv run run.py --mock --trials 3 --trial-s 60 --instruction "Pick all eight screws into the yellow bin."
uv run run.py --submit
uv run run.py --status
uv run run.py   # during approved slot
```

Tests (from `real_robot/`):

```bash
uv run python -m unittest discover -s tests -p 'test_*.py'
```

## Layout

| Path | Role |
|------|------|
| `policy.py` | Organizer entry: `Policy.act(obs)` |
| `screw_policy/` | Perception, localization, TCP state machine |
| `config/screw_task.yaml` | Physical task profile (UF0 mm) |
| `submission/` | Hackathon submission YAML |
| `UPSTREAM.md` | Pinned vendor client revision |

Artifacts: `artifacts/` (override with `REAL_ROBOT_ARTIFACT_DIR`).

Each rollout writes:

| File | Content |
|------|---------|
| `metrics.json` | KPIs: screws placed, success rate, pick cycle times, perception mix, act/Cosmos latency |
| `PRESENTATION.md` | Slide-ready bullet summary for that rollout |
| `steps.jsonl` | Step-by-step trace (phase, TCP, timings) |
| `session_rollouts.jsonl` | Append-only log of all rollouts (under artifact root) |

After mock or real runs, aggregate for your deck:

```bash
uv run python report_metrics.py
uv run python report_metrics.py --out docs/presentation_kpis.md
```

Also generates `artifacts/session_summary.json` with mean/median screws placed, task completion rate, and latency averages.

## Cell readiness

See [docs/CELL_READINESS.md](docs/CELL_READINESS.md) for organizer questions and supervised tuning order before real motion.
