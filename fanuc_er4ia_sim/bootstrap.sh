#!/usr/bin/env bash
# Prepare only the host environment and locally generated materials.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
bootstrap_python() {
    if [ -n "${HOST_PYTHON_BOOTSTRAP:-}" ]; then
        echo "$HOST_PYTHON_BOOTSTRAP"
        return
    fi
    if command -v python3 >/dev/null 2>&1; then
        command -v python3
        return
    fi
    if [ -n "${ISAAC:-}" ] && [ -x "${ISAAC}/kit/python/bin/python3" ]; then
        echo "${ISAAC}/kit/python/bin/python3"
        return
    fi
    echo "python3 not found. Install Python 3.12+, or export ISAAC, or set HOST_PYTHON_BOOTSTRAP." >&2
    exit 127
}
if [ ! -x .venv/bin/python ]; then
    "$(bootstrap_python)" -m venv .venv
fi
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python make_textures.py
.venv/bin/python -m unittest test_sort_planning test_cube_perception test_cube_pose test_runtime_config test_recovery_policy test_recovery_runner test_fake_isaac
echo "Host setup complete. Configure ISAAC and COSMOS next; see SETUP.md."
