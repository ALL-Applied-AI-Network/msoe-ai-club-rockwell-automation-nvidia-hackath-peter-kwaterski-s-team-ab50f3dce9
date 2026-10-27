#!/usr/bin/env bash
# One-time Cosmos venv for fanuc_er4ia_sim/perceive.py (shared VM friendly).
set -euo pipefail

VENV="${COSMOS_VENV:-/workspace/cosmos-venv}"
FRAMEWORK="${COSMOS3_FRAMEWORK:-/workspace/cosmos-framework}"
MODEL="${COSMOS3_NANO_PATH:-/workspace/models/Cosmos3-Nano}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d "$VENV" ]; then
    python3 -m venv "$VENV"
fi

"$VENV/bin/pip" install -U pip
"$VENV/bin/pip" install torch torchvision --index-url https://download.pytorch.org/whl/cu128
# transformers 5.x breaks Cosmos3 weight mapping; starter validated on 4.57.x
"$VENV/bin/pip" install 'transformers==4.57.6' accelerate safetensors
"$VENV/bin/pip" install -r "$REPO/fanuc_er4ia_sim/requirements-cosmos.txt"

if [ ! -d "$FRAMEWORK/packages/transformers-cosmos3" ]; then
    git clone --depth 1 https://github.com/NVIDIA/cosmos-framework.git "$FRAMEWORK"
fi
"$VENV/bin/pip" install -e "$FRAMEWORK/packages/transformers-cosmos3"

cat <<EOF

Cosmos venv ready: $VENV

Add to scripts/env.local.sh (or export manually):
  export COSMOS=$VENV/bin/python
  export COSMOS3_NANO_PATH=$MODEL
  export COSMOS3_ROOT=$FRAMEWORK

Verify:
  source $REPO/scripts/env.local.sh
  cd $REPO/fanuc_er4ia_sim
  "\$COSMOS" -c 'from perceive import load_model; load_model(); print("ok")'

EOF
