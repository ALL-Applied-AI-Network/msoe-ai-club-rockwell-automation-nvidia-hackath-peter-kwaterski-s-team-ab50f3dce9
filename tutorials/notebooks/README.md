# Mentee notebook curriculum

Hands-on Jupyter notebooks for the Rockwell + NVIDIA hackathon team. Work through them in order on your **NVIDIA Brev** GPU instance (or any Linux machine with Isaac Sim 6.0 and Cosmos3-Nano configured).

## Learning order

| Notebook | Time (approx.) | What you will do |
|----------|----------------|------------------|
| [00_git_and_github.ipynb](00_git_and_github.ipynb) | 45–60 min | Clone, branch, commit, push, PRs; keep secrets and weights out of git |
| [01_python_fundamentals.ipynb](01_python_fundamentals.ipynb) | 60–90 min | Python basics using data from the FANUC starter |
| [02_uv_and_environments.ipynb](02_uv_and_environments.ipynb) | 45–60 min | `uv` for the tutorial kernel; map host vs Isaac vs Cosmos interpreters |
| [03_extend_the_simulation.ipynb](03_extend_the_simulation.ipynb) | 3–5 hours | Live pipeline: scene objects, planning, Cosmos, in-context learning, full run |

Notebooks **00–02** run entirely in this folder’s Python environment. **03** requires the live stack documented in [`fanuc_er4ia_sim/SETUP.md`](../../fanuc_er4ia_sim/SETUP.md).

## Setup (once per machine)

From this directory:

```bash
# Install uv if needed: https://docs.astral.sh/uv/getting-started/installation/
uv sync
uv run python -m ipykernel install --user --name rockwell-tutorials --display-name "Rockwell tutorials"
```

Open Jupyter Lab or VS Code and select the **Rockwell tutorials** kernel.

Host simulation code (planning, tests) still uses `fanuc_er4ia_sim/.venv` from `./bootstrap.sh`. Do not replace Isaac or Cosmos with this kernel.

## NVIDIA Brev workflow

1. **Start** your Brev instance when you begin work. Open a terminal and clone or pull this repository.
2. **Configure paths** on the instance (adjust to your layout):

   ```bash
   export ISAAC=/path/to/isaacsim
   export COSMOS=/path/to/cosmos-env/bin/python
   export COSMOS3_NANO_PATH=/path/to/Cosmos3-Nano/snapshot
   # Optional: export COSMOS3_ROOT=... if transformers-cosmos3 is not pip-installed
   # Optional: export FANUC_ROBOT_USD=... if the ER-4iA asset is not in Isaac’s cache
   ```

3. **Bootstrap the host venv** (from `fanuc_er4ia_sim/`):

   ```bash
   cd fanuc_er4ia_sim && ./bootstrap.sh
   ```

4. **Run Cosmos smoke checks** (from `SETUP.md`):

   ```bash
   "$COSMOS" -c 'import torch; print("CUDA:", torch.cuda.is_available())'
   cd fanuc_er4ia_sim
   "$COSMOS" -c 'from perceive import load_model; load_model(); print("Model ready")'
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
