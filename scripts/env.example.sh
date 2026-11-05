#!/usr/bin/env bash
# Copy to env.local.sh, fill in paths, then: source scripts/env.local.sh
# Do not commit env.local.sh or model weights.

# --- Isaac Sim (required for capture + execution)
# Brev: path from the compute guide. Isaac Lab bundle example:
export ISAAC="${ISAAC:-/workspace/isaaclab/_isaac_sim}"

# --- Cosmos3-Nano (required for perceive.py)
# Separate venv with CUDA PyTorch + requirements-cosmos.txt
export COSMOS="${COSMOS:-/workspace/cosmos-venv/bin/python}"
export COSMOS3_NANO_PATH="${COSMOS3_NANO_PATH:-/workspace/models/Cosmos3-Nano}"

# --- Optional
export COSMOS3_ROOT="${COSMOS3_ROOT:-/workspace/cosmos-framework}"
# One-time Cosmos stack: bash scripts/setup-cosmos-venv.sh (pins transformers==4.57.6)
# export FANUC_ROBOT_USD=/path/to/er4ia.usd
# export HF_HOME=$HOME/.cache/huggingface
export HOST_PYTHON="${HOST_PYTHON:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/fanuc_er4ia_sim/.venv/bin/python}"

# --- Git on Brev (optional): plain `git push` uses a broken VS Code credential socket in SSH.
# source scripts/git-env.sh   # or: bash scripts/git-push.sh
