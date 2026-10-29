# Team mentee guide — Grasping & Sorting (FANUC digital twin)

This document  complements the official starter docs and maps your hackathon metrics and problem ideas to concrete files and workflows.

**Official references**

| Resource | Link |
|----------|------|
| Event hub | [Rockwell-NVIDIA Fall 2026](https://msoe-maic.com/library/?article=Rockwell-NVIDIA-Fall-2026) |
| Grasping & Sorting brief | [dataset article](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting) |
| Compute (Brev, DGX Spark) | [compute guide](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute) |
| Starter setup | [`fanuc_er4ia_sim/SETUP.md`](../fanuc_er4ia_sim/SETUP.md) |
| Notebook curriculum | [`tutorials/notebooks/README.md`](../tutorials/notebooks/README.md) |

---

## What this repository contains

| Area | Path | Purpose |
|------|------|---------|
| Simulation + pipeline | [`fanuc_er4ia_sim/`](../fanuc_er4ia_sim/) | Isaac scene, Cosmos perception, planning, execution |
| Repo scripts | [`scripts/`](../scripts/) | `setup-dev.sh`, `preflight.sh` |
| Tutorials | [`tutorials/notebooks/`](../tutorials/notebooks/) | Git, Python, uv, perception eval, recovery, CAD objects |

The **default capstone task** is a **six-cube stack** (yellow×2, blue×2, red×2) with **two caps and two vials as distractors**. The codebase also supports a **three-class sort** (cubes, caps, vials/tubes) into labelled bins—your **Mixed Sort** track builds on that.

---

## Environment checklist (run on every new machine)

Use this table after clone. On **NVIDIA Brev**, follow the [compute guide](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute) for Isaac Sim and Cosmos install paths.

### 1. One-time repo setup (Brev)

If **Isaac Sim is already on the instance** (normal Brev template):

```bash
cd <repo-root>
git pull
bash scripts/setup-brev.sh --download-model --verify-model   # first time on empty disk
# or, if /workspace/models/Cosmos3-Nano already exists from a teammate:
bash scripts/setup-brev.sh --verify-model
```

That script: detects `ISAAC`, bootstraps the host venv, installs **Cosmos** (`transformers==4.57.6` + `transformers-cosmos3`), syncs tutorial notebooks, writes **`scripts/env.local.sh`**, and runs **`preflight.sh`**.

Manual alternative (same result in pieces): `setup-dev.sh` + `setup-cosmos-venv.sh` + edit `env.example.sh` → `env.local.sh`.

**If `python3` is missing on PATH** (some GPU images only ship Isaac’s interpreter):

```bash
export ISAAC=/path/to/isaacsim   # see step 2
bash fanuc_er4ia_sim/bootstrap.sh
# or: export HOST_PYTHON_BOOTSTRAP="$ISAAC/kit/python/bin/python3" then bootstrap again
```

### 2. Export paths (every new shell)

After `setup-brev.sh`:

```bash
source scripts/env.local.sh
```

Or copy [`scripts/env.example.sh`](../scripts/env.example.sh) to `env.local.sh` and edit manually.

Required:

| Variable | Purpose |
|----------|---------|
| `ISAAC` | Isaac Sim install root (must contain `python.sh`) |
| `COSMOS` | Python executable in a **separate** CUDA venv for Cosmos3-Nano |
| `COSMOS3_NANO_PATH` | Directory with model `config.json` and weights (recommended) |

Optional: `COSMOS3_ROOT`, `FANUC_ROBOT_USD`, `HF_HOME`, `HOST_PYTHON`.

Install Cosmos deps (once per Cosmos venv):

```bash
"$COSMOS" -m pip install -r fanuc_er4ia_sim/requirements-cosmos.txt
```

### 3. Preflight

```bash
bash scripts/preflight.sh
bash scripts/preflight.sh --load-model   # slow; confirms weights load
```

### 4. Isaac smoke test (no Cosmos)

```bash
mkdir -p /tmp/scene_test
"$ISAAC/python.sh" fanuc_er4ia_sim/build_scene.py --headless --save /tmp/scene_test/scene.usd
```

Success: log line `[scene] stage exported to ...` and ER-4iA DOFs listed. First run may download the robot USD from NVIDIA’s asset CDN.

### 5. Full pipeline (needs Cosmos)

```bash
cd fanuc_er4ia_sim
./run_sort_pipeline.sh runs/my_run_01   # directory must be empty
```

Inspect `runs/my_run_01/execution_report.json`:

- `task_complete: true`
- `verified_pickups: 6`
- `verified_placements: 6`

**Important:** Isaac can exit 0 even when the stack failed—always read the report (see notebook **08**).

---

## Three Python interpreters (never mix)

| Role | Typical command | Used for |
|------|-----------------|----------|
| **Host** | `fanuc_er4ia_sim/.venv/bin/python` | `plan_sort.py`, `encode_video.py`, unit tests |
| **Isaac** | `"$ISAAC/python.sh" script.py` | `build_scene.py`, `capture_scene.py`, `sort_scene.py` |
| **Cosmos** | `"$COSMOS" perceive.py ...` | GPU VLM inference |

Jupyter uses the **Rockwell tutorials** kernel only for notebooks 00–02 and analysis cells—not for Isaac Kit or Cosmos.

---

## Pipeline stages (what each artifact means)

```text
capture_scene.py  →  table/wrist images, ground_truth.json (eval only)
perceive.py       →  detections.json, cube_pose_crops/
plan_sort.py      →  sort_plan.json, perception_evaluation.json
sort_scene.py     →  execution_report.json, execution_events.json, frames/
encode_video.py   →  sort_table.mp4, sort_side.mp4
```

| Metric (team) | Where to read it |
|---------------|------------------|
| **Task success rate** | `execution_report.json` → `task_complete`, counts of verified pickup/placement |
| **Perception robustness** | `perception_evaluation.json`; notebook **04**; recall lines in planning logs |
| **Placement quality** | `execution_events.json` → `placement_error_mm`, `placement_yaw_error_deg_mod90`, stack fields |
| **Recovery rate** | Not automated yet—see **Problem D** below and notebook **08** |
| **Cycle time** | Frame count × sim timestep, or wall clock on Brev; compare runs via notebook **09** |
| **Novel object** | New USD in `assets/`, `scene_config.py`, notebook **11** |

---

## Team roadmap

Suggested order: **Mixed Sort + counting** → **organize / harder scenes** → **recovery + failure alerts**.

### B — Mixed Sort + counting (primary)

**Goal:** Sort **cubes, caps, vials (tubes)** into bins with category rules; count objects per class.

**Code hooks already present:**

- Three bin definitions: `SORT_BINS` in [`scene_config.py`](../fanuc_er4ia_sim/scene_config.py) (`cube`, `cap`, `vial`).
- Sort routing: [`plan_grasps.py`](../fanuc_er4ia_sim/plan_grasps.py) (category → bin).
- Unified planner: [`plan_sort.py`](../fanuc_er4ia_sim/plan_sort.py) with `--task sort`.
- Cosmos prompt already asks for vials, caps, and cubes in [`perceive.py`](../fanuc_er4ia_sim/perceive.py).

**Steps to switch from stack to sort:**

1. In `scene_config.py`: set `SORT_TASK = "sort"` (default is `"cube_stack"`).
2. Set `SHOW_BINS = True` when you want bin geometry visible in sim (optional for planning; bins are defined either way).
3. Run pipeline **without** `--stack-cubes` on perceive (use default detection mode), e.g. adjust `run_sort_pipeline.sh` or run stages manually:
   - `"$ISAAC/python.sh" capture_scene.py --out runs/sort_01`
   - `"$COSMOS" perceive.py --run runs/sort_01`
   - `.venv/bin/python plan_sort.py --run runs/sort_01 --task sort`
   - `"$ISAAC/python.sh" sort_scene.py --run runs/sort_01`
4. Add **counting** in a small script or notebook: compare `detections.json` class counts to `ground_truth.json` (eval only) for development; for submission, report counts from vision-only `detections.json`.

**Generalizability angle:** Category-level rules (colour + shape family), not instance memorization—document prompts and ICL examples in notebook **06**.

### A — Harder scene (clutter / pose)

**Goal:** Same stack or sort, but harder layout and distractors.

- Reseed layout: `CUBE_LAYOUT_SEED`, `hackathon_cubes()` in `scene_config.py`.
- Add clutter: more caps/vials, `SHOW_FIXTURE`, notebook **07** (domain randomization).
- Do **not** weaken perception gates to force passes—diagnose skips in `sort_plan.json` (`skip_reason`).

### C — Pick novel rigid objects

**Goal:** New instances of known geometric families (e.g. new cube size, new cap mesh).

- Follow notebook **11** (CAD → USD → colliders).
- Update masses, grasp heights, and jaw clearance in `scene_config.py` / `gripper_geometry.py`.
- Re-run calibration checklist in [`docs/CALIBRATION_V05.md`](../fanuc_er4ia_sim/docs/CALIBRATION_V05.md) if camera or table changes.

### D — Recovery and retry policy

**Goal:** When `pickup_verified: false`, retry with offset or spread flip without human reset.

**Current behavior:** [`sort_scene.py`](../fanuc_er4ia_sim/sort_scene.py) records `pickup_verified` per pick in `execution_events.json` and may **abort** the cube stack on failed placement. There is **no automatic retry loop** in the starter—this is intentional hackathon extension space.

**Safe extension pattern:**

1. Read notebook **08** and `execution_events.json` fields (`objects_lifted`, `grasp_spread_dir`, phase measurements).
2. Implement retry **above** Isaac execution: re-plan from fresh perception or nudge grasp XY/yaw in `plan_sort.py` / a new `recovery_policy.py`.
3. Re-run `sort_scene.py --physics-only` only when reusing a valid plan (see `SETUP.md`).

**Metric:** recovery rate = successful retries / failed first attempts (define clearly in your write-up).

### E — Finger redesign

**Goal:** Hardware-accurate gripper geometry for real cell transfer.

- [`gripper_geometry.py`](../fanuc_er4ia_sim/gripper_geometry.py), `scene_config.py` jaw limits.
- After geometry changes: host tests + fresh capture + full pipeline run.

### Failure alerts

**Goal:** Notify when a run fails thresholds.

- Parse `execution_report.json` and `perception_evaluation.json` in CI or a small watcher script.
- Fail conditions examples: `task_complete == false`, mean position error above X mm, any `pickup_verified == false` with no retry success.

---

## One shared VM or container (we are doing this currently btw)

### What you must change (team rules)

| Topic | Per-person Brev | Shared VM |
|--------|-----------------|-----------|
| **Git** | Pull before work | **Required:** branch per task, pull before push, no editing `main` directly on the box without syncing |
| **Pipeline runs** | Any time | **One GPU job at a time** — coordinate in Slack (“running full pipeline ~20 min”) |
| **Run directories** | `runs/stack_01` | **Prefix by person or experiment:** `runs/alice_sort_03`, `runs/bob_recovery_01` — pipeline requires **empty** dirs |
| **Dependency changes** | Local `.venv` | Changing `requirements.txt` / `uv sync` affects **everyone** — announce and re-run bootstrap once |
| **Jupyter** | Private kernel | Shared machine: avoid long-running notebooks while someone else needs GPU |
| **Env file** | Optional per user | Usually **one** `scripts/env.local.sh` on the box; don’t commit it |

### Suggested shared-VM ritual

1. One “platform lead” owns first-time install + `preflight.sh --load-model` + documents paths in team Slack (not in git).
2. Before a full pipeline: `git pull`, confirm no one else is using GPU (`nvidia-smi` or a simple `#gpu-busy` Slack emoji).
3. Mentees do **host-only** work (planning tests, notebooks 04–05 on saved runs) in parallel; **Isaac + Cosmos** runs are scheduled.
4. Persist what matters: clone lives on disk; **commit and push** code often so a rebuilt container is not catastrophic. Large `runs/` and weights stay out of git (note locations in README).

---

## Development workflow (daily)

1. `git pull` on the team repo.
2. `source scripts/env.example.sh` (or your private env file—**never commit secrets or weights**).
3. `bash scripts/preflight.sh` (quick health check).
4. Work on a **feature branch**; run host tests after Python changes:

   ```bash
   cd fanuc_er4ia_sim
   .venv/bin/python -m unittest test_sort_planning test_cube_perception test_cube_pose test_runtime_config
   ```

5. For sim/perception changes, create a **new empty** `runs/<id>/` and run the full or partial pipeline.
6. **Stop Brev** when idle—credits accrue while the instance runs.

---

## Learning path for new mentees

| Week / phase | Notebooks | Focus |
|--------------|-----------|--------|
| Onboarding | 00, 01, 02 | Git, Python, uv, three interpreters |
| First end-to-end | 03, 04, 05 | Pipeline, metrics, frames |
| Team track | 06, 07, 08 | Prompts, robustness, recovery |
| Experiments | 09, 10 | Reproducibility, sim-to-real |
| New objects | 11 | CAD → sim |

If time is short after 03: do **04, 05, 08** next.

---

## Workspace verification log (2026-10-27)

This section records what was checked on a **Cursor cloud GPU workspace** with Isaac Lab’s bundled Isaac Sim. Your Brev instance may differ—always run preflight locally.

| Check | Result |
|-------|--------|
| NVIDIA GPU | OK — RTX PRO 6000 Blackwell |
| Repo + `fanuc_er4ia_sim/` | OK |
| Isaac Sim at `/workspace/isaaclab/_isaac_sim` | OK — headless `build_scene.py` exported scene; ER-4iA loaded from Isaac 6.0 CDN |
| Host `.venv` + 33 unit tests | OK (after bootstrap with Isaac kit Python) |
| Tutorial `uv sync` | OK |
| `uv` CLI | OK (user-local install) |
| System `python3` on PATH | **Missing** — use Brev’s Python or `HOST_PYTHON_BOOTSTRAP` / `ISAAC/kit/python/bin/python3` |
| `COSMOS` / `COSMOS3_NANO_PATH` | **Not configured** — required for full pipeline |
| `preflight.sh` | Passes only after Cosmos exports are set |

**Action for the team:** On Brev, complete Cosmos3-Nano download and venv per [`SETUP.md`](../fanuc_er4ia_sim/SETUP.md) and event compute docs, then run `preflight.sh --load-model` once before demo day.

---

## Getting help

- **Starter / gates:** [`fanuc_er4ia_sim/docs/MODEL_NOTES.md`](../fanuc_er4ia_sim/docs/MODEL_NOTES.md)
- **Slack:** `#hackathon-announcements` and the Grasping & Sorting channel (see root [`README.md`](../README.md))
- **Agents / coding assistants:** [`fanuc_er4ia_sim/AGENTS.md`](../fanuc_er4ia_sim/AGENTS.md)
