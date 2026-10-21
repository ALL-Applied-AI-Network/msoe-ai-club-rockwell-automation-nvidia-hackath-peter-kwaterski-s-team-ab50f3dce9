# Mentee notebook curriculum

Hands-on Jupyter notebooks for the Rockwell + NVIDIA hackathon team. Work through them in order on your **NVIDIA Brev** GPU instance (or any Linux machine with Isaac Sim 6.0 and Cosmos3-Nano configured).

## Learning order

| Notebook | Time (approx.) | What you will do |
|----------|----------------|------------------|
| [00_git_and_github.ipynb](00_git_and_github.ipynb) | 45–60 min | Clone, branch, commit, push, PRs; keep secrets and weights out of git |
| [01_python_fundamentals.ipynb](01_python_fundamentals.ipynb) | 60–90 min | Python basics using data from the FANUC starter |
| [02_uv_and_environments.ipynb](02_uv_and_environments.ipynb) | 45–60 min | `uv` for the tutorial kernel; map host vs Isaac vs Cosmos interpreters |
| [03_extend_the_simulation.ipynb](03_extend_the_simulation.ipynb) | 3–5 hours | Live pipeline: scene objects, planning, Cosmos, in-context learning, full run |
| [04_evaluate_perception.ipynb](04_evaluate_perception.ipynb) | 90–120 min | Score perception and execution, visualize errors, compare runs |
| [05_debug_coordinate_frames.ipynb](05_debug_coordinate_frames.ipynb) | 90–120 min | Pixel, camera, world and robot frames; projection and debugging |
| [06_prompt_and_icl_experiments.ipynb](06_prompt_and_icl_experiments.ipynb) | 2–3 hours | Controlled Cosmos prompt and image-example experiments |
| [07_synthetic_data_and_domain_randomization.ipynb](07_synthetic_data_and_domain_randomization.ipynb) | 2–4 hours | Reproducible Isaac scene variants and held-out robustness tests |
| [08_failure_analysis_and_recovery.ipynb](08_failure_analysis_and_recovery.ipynb) | 90–150 min | Diagnose the first failed stage and choose safe recovery behavior |
| [09_reproducible_experiments.ipynb](09_reproducible_experiments.ipynb) | 90–120 min + run | Hypotheses, run manifests, hashes and controlled comparisons |
| [10_sim_to_real_readiness.ipynb](10_sim_to_real_readiness.ipynb) | 2 hours + cell session | Evidence review and staged, supervised physical validation |

Notebooks **00–02** run entirely in this folder’s Python environment. **03–10** use saved run artifacts and progressively add the live Isaac/Cosmos stack documented in [`fanuc_er4ia_sim/SETUP.md`](../../fanuc_er4ia_sim/SETUP.md). Live cells are opt-in when they are expensive or create new runs.

For beginners, work in order and pair with a teammate for notebooks 06–10. If time is limited, complete **04, 05 and 08** after the original sequence; they teach the core loop of measuring, diagnosing and recovering.

## Setup (once per machine)

From the **repository root** (recommended on Brev):

```bash
bash scripts/setup-dev.sh
```

That runs `fanuc_er4ia_sim/bootstrap.sh` (host venv, textures, 33 tests) and `uv sync` for this notebook folder, and registers the **Rockwell tutorials** Jupyter kernel. Use `bash scripts/setup-dev.sh --no-kernel` if you only want the virtual environments.

Manual equivalent: install [uv](https://docs.astral.sh/uv/getting-started/installation/), then from this directory run `uv sync` and `uv run python -m ipykernel install --user --name rockwell-tutorials --display-name "Rockwell tutorials"`.

Open Jupyter Lab or VS Code and select the **Rockwell tutorials** kernel.

Host simulation code (planning, tests) still uses `fanuc_er4ia_sim/.venv`. Do not replace Isaac or Cosmos with this kernel.

## NVIDIA Brev workflow

1. **Start** your Brev instance when you begin work. Clone or pull this repository.
2. **Configure paths** on the instance (adjust to your layout):

   ```bash
   export ISAAC=/path/to/isaacsim
   export COSMOS=/path/to/cosmos-env/bin/python
   export COSMOS3_NANO_PATH=/path/to/Cosmos3-Nano/snapshot
   # Optional: export COSMOS3_ROOT=... if transformers-cosmos3 is not pip-installed
   # Optional: export FANUC_ROBOT_USD=... if the ER-4iA asset is not in Isaac’s cache
   ```

3. **Set up repo-owned environments:** `bash scripts/setup-dev.sh`

4. **Preflight** (after exports above):

   ```bash
   bash scripts/preflight.sh
   bash scripts/preflight.sh --load-model   # optional: verify Cosmos weights load
   ```

5. **Stop or pause** the Brev instance when you are done. Brev credits accrue while the instance runs.

Event compute guide: [Rockwell-NVIDIA Fall 2026 Compute](https://msoe-maic.com/library/?article=Rockwell-NVIDIA-Fall-2026-Compute).

## Three Python interpreters (do not mix)

| Role | Typical command | Used for |
|------|-----------------|----------|
| **Host** | `fanuc_er4ia_sim/.venv/bin/python` | `plan_sort.py`, `encode_video.py`, unit tests |
| **Isaac Sim** | `"$ISAAC/python.sh" script.py` | `build_scene.py`, `capture_scene.py`, `sort_scene.py` |
| **Cosmos** | `"$COSMOS" perceive.py ...` | GPU inference in `perceive.py` |

Notebook 03 calls these via shell cells; it does not import Isaac Kit inside Jupyter.

## Repository paths

| Path | Purpose |
|------|---------|
| `fanuc_er4ia_sim/scene_config.py` | Cube layout, stack order, cameras |
| `fanuc_er4ia_sim/cell.py` | USD scene build, `add_sort_objects()` |
| `fanuc_er4ia_sim/perceive.py` | Cosmos prompts, hybrid pose, ICL via `ask(..., examples=...)` |
| `fanuc_er4ia_sim/plan_sort.py` | Vision-only planning, `build_plan()` |
| `fanuc_er4ia_sim/run_sort_pipeline.sh` | End-to-end capture → perceive → plan → execute |

## Success criteria (capstone)

After a full pipeline run, `runs/<id>/execution_report.json` must show:

- `task_complete: true`
- `verified_pickups: 6`
- `verified_placements: 6`

Isaac can exit with code 0 even when the stack failed; always read the report.

## Getting help

- Starter setup: [`fanuc_er4ia_sim/SETUP.md`](../../fanuc_er4ia_sim/SETUP.md)
- Perception gates: [`fanuc_er4ia_sim/docs/MODEL_NOTES.md`](../../fanuc_er4ia_sim/docs/MODEL_NOTES.md)
- Hackathon Slack and dataset channels (see root [`README.md`](../../README.md))
