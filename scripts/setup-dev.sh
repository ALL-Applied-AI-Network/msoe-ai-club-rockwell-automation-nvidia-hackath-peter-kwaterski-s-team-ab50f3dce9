#!/usr/bin/env bash
# Host + tutorial notebook environment (repo-owned deps only).
# Does not install Isaac Sim, Cosmos weights, or GPU stacks — configure those separately.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$ROOT/fanuc_er4ia_sim"
TUTORIALS="$ROOT/tutorials/notebooks"
INSTALL_KERNEL=1

usage() {
    cat <<'EOF'
Usage: scripts/setup-dev.sh [options]

Prepare development environments after cloning on a GPU instance (e.g. NVIDIA Brev).

  - Host: fanuc_er4ia_sim/.venv via bootstrap.sh (textures + 33 unit tests)
  - Tutorials: tutorials/notebooks/.venv via uv sync

Options:
  --no-kernel   Skip registering the Jupyter "Rockwell tutorials" kernel
  -h, --help    Show this help

Requires: python3 with venv/pip, and uv on PATH for the tutorial project.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-kernel) INSTALL_KERNEL=0 ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
    shift
done

if [ ! -f "$SIM/bootstrap.sh" ]; then
    echo "Missing $SIM/bootstrap.sh — run from the hackathon repository root." >&2
    exit 1
fi

if [ "${SETUP_DEV_SKIP_HOST:-0}" != 1 ]; then
    echo "==> Host environment (fanuc_er4ia_sim)"
    bash "$SIM/bootstrap.sh"
fi

if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found on PATH. Install from https://docs.astral.sh/uv/getting-started/installation/" >&2
    echo "Host setup finished; tutorial notebooks still need: cd tutorials/notebooks && uv sync" >&2
    exit 1
fi

echo "==> Tutorial notebooks (uv)"
(
    cd "$TUTORIALS"
    uv sync
)

if [ "$INSTALL_KERNEL" -eq 1 ]; then
    echo "==> Jupyter kernel (Rockwell tutorials)"
    (
        cd "$TUTORIALS"
        uv run python -m ipykernel install --user --name rockwell-tutorials --display-name "Rockwell tutorials"
    )
fi

cat <<EOF

Development setup complete.

Next:
  1. Export ISAAC, COSMOS, and COSMOS3_NANO_PATH (see fanuc_er4ia_sim/SETUP.md).
  2. Run: bash scripts/preflight.sh
  3. Optional full Cosmos check: bash scripts/preflight.sh --load-model

Stop your Brev instance when you are done to save credits.
EOF
