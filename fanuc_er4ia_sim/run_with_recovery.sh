#!/usr/bin/env bash
# Replay an existing planned run with bounded pickup-recovery attempts.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
if [ -f "$REPO/scripts/env.local.sh" ]; then
    # shellcheck source=/dev/null
    source "$REPO/scripts/env.local.sh"
fi

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 RUN_DIR" >&2
    exit 2
fi

RUN="$(cd "$1" && pwd)"
ISAAC="${ISAAC:-}"
if [ -z "$ISAAC" ]; then
    for candidate in /workspace/isaaclab/_isaac_sim /isaacsim; do
        if [ -x "$candidate/python.sh" ]; then
            ISAAC="$candidate"
            break
        fi
    done
fi
HOST_PYTHON="${HOST_PYTHON:-$HERE/.venv/bin/python}"
MAX_RECOVERY_RETRIES="${MAX_RECOVERY_RETRIES:-2}"

if [ ! -f "$RUN/sort_plan.json" ]; then
    echo "Missing $RUN/sort_plan.json; prepare a run with capture, perception and planning first." >&2
    exit 1
fi
if [ -z "$ISAAC" ] || [ ! -x "$ISAAC/python.sh" ]; then
    echo "Isaac launcher unavailable; set ISAAC to the Isaac Sim installation directory." >&2
    exit 1
fi
if [ ! -x "$HOST_PYTHON" ]; then
    echo "Host Python unavailable or not executable: $HOST_PYTHON" >&2
    exit 1
fi

exec "$HOST_PYTHON" "$HERE/recovery_policy.py" \
    --run "$RUN" \
    --isaac-python "$ISAAC/python.sh" \
    --max-retries "$MAX_RECOVERY_RETRIES"
