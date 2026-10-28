#!/usr/bin/env bash
# Stand-in for "$ISAAC/python.sh" so the recovery loop runs on a laptop (no GPU).
# Usage: ISAAC="$PWD/fake_isaac" ./run_with_recovery.sh runs/fake_01
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "${HOST_PYTHON:-$HERE/../.venv/bin/python}" "$HERE/launch.py" "$@"
