#!/usr/bin/env bash
# Verify GPU instance readiness for the FANUC Isaac/Cosmos pipeline.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$ROOT/fanuc_er4ia_sim"
LOAD_MODEL=0
FAIL=0

usage() {
    cat <<'EOF'
Usage: scripts/preflight.sh [options]

Checks environment variables, interpreter paths, GPU visibility, and optional Cosmos load.
Export ISAAC, COSMOS, and (recommended) COSMOS3_NANO_PATH before running.

Options:
  --load-model   Also run perceive.load_model() (slow; uses GPU memory)
  -h, --help     Show this help

Exit code 0 if all required checks pass; 1 otherwise.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --load-model) LOAD_MODEL=1 ;;
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

pass() { echo "[ok] $*"; }
warn() { echo "[warn] $*"; }
fail() { echo "[fail] $*"; FAIL=1; }

check_executable() {
    local label="$1" path="$2"
    if [ -z "$path" ]; then
        fail "$label: path is empty"
        return
    fi
    if [ ! -e "$path" ]; then
        fail "$label: not found: $path"
        return
    fi
    if [ ! -x "$path" ]; then
        fail "$label: not executable: $path"
        return
    fi
    pass "$label: $path"
}

echo "==> Repository"
pass "root: $ROOT"
if [ ! -d "$SIM" ]; then
    fail "missing fanuc_er4ia_sim/"
fi

echo "==> GPU"
if command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi -L >/dev/null 2>&1; then
        nvidia-smi -L | head -n 3
        pass "nvidia-smi"
    else
        fail "nvidia-smi present but returned an error"
    fi
else
    fail "nvidia-smi not found (no NVIDIA driver or wrong machine type)"
fi

echo "==> Disk"
if command -v df >/dev/null 2>&1; then
    df -h "$ROOT" | tail -n 1
else
    warn "df not available; skipping disk summary"
fi

echo "==> Environment variables"
for var in ISAAC COSMOS; do
    if [ -z "${!var:-}" ]; then
        fail "unset required variable: $var"
    else
        pass "$var is set"
    fi
done
for var in COSMOS3_NANO_PATH COSMOS3_ROOT FANUC_ROBOT_USD HOST_PYTHON HF_HOME HF_HUB_CACHE; do
    if [ -n "${!var:-}" ]; then
        pass "$var is set"
    else
        warn "optional unset: $var"
    fi
done

HOST_PYTHON="${HOST_PYTHON:-$SIM/.venv/bin/python}"
ISAAC_PY="${ISAAC:-}/python.sh"
COSMOS="${COSMOS:-}"

echo "==> Interpreters"
check_executable "Isaac launcher" "$ISAAC_PY"
check_executable "Cosmos python" "$COSMOS"
check_executable "Host python" "$HOST_PYTHON"

if [ -x "$HOST_PYTHON" ]; then
    echo "==> Host smoke"
    if "$HOST_PYTHON" -c "import numpy, scipy, cv2; print('host imports ok')"; then
        pass "host imports (numpy, scipy, cv2)"
    else
        fail "host import check failed — run scripts/setup-dev.sh"
    fi
fi

if [ -x "$COSMOS" ]; then
    echo "==> Cosmos smoke"
    if "$COSMOS" -c 'import torch, transformers, scipy, cv2; print("CUDA:", torch.cuda.is_available())'; then
        pass "Cosmos deps import"
    else
        fail "Cosmos dependency import failed"
    fi
    if [ "$LOAD_MODEL" -eq 1 ]; then
        echo "==> Cosmos model load (this may take several minutes)"
        if (cd "$SIM" && "$COSMOS" -c 'from perceive import load_model; load_model(); print("Model ready")'); then
            pass "Cosmos load_model()"
        else
            fail "Cosmos load_model() — check COSMOS3_NANO_PATH / HF cache / COSMOS3_ROOT"
        fi
    else
        warn "skipped model load (pass --load-model to verify weights)"
    fi
fi

if [ -x "$ISAAC_PY" ]; then
    echo "==> Isaac launcher"
    if "$ISAAC_PY" -c "import sys; print('Isaac python', sys.version.split()[0])" >/dev/null 2>&1; then
        pass "Isaac python.sh responds"
    else
        warn "Isaac python.sh did not run a simple -c check (Kit may require full scripts)"
    fi
fi

echo "==> Tutorial uv project"
if command -v uv >/dev/null 2>&1 && [ -f "$ROOT/tutorials/notebooks/pyproject.toml" ]; then
    if [ -x "$ROOT/tutorials/notebooks/.venv/bin/python" ] || [ -x "$ROOT/tutorials/notebooks/.venv/Scripts/python.exe" ]; then
        pass "tutorials/notebooks/.venv exists"
    else
        warn "tutorials venv missing — run scripts/setup-dev.sh"
    fi
else
    warn "uv or tutorials/notebooks/pyproject.toml not found"
fi

if [ "$FAIL" -ne 0 ]; then
    echo >&2
    echo "Preflight failed. See fanuc_er4ia_sim/SETUP.md and tutorials/notebooks/README.md." >&2
    exit 1
fi

echo
echo "Preflight passed."
