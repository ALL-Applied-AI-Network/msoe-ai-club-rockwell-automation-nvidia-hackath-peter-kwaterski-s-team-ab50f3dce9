#!/usr/bin/env bash
# Pickup recovery ABOVE Isaac (team guide, Problem D).
#
#   ./run_with_recovery.sh runs/stack_01
#
# RUN must already contain sort_plan.json (i.e. capture/perceive/plan are done).
# If RUN has no execution_report.json yet, it is executed first (physics-only).
# Then: classify the failure -> write a corrected plan into RUN_retry1 -> execute
# it -> repeat, at most 3 retries. recovery_policy.py decides; this script only
# moves files and starts the simulator. Summary: RUN/recovery_log.json
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
if [ -f "$REPO/scripts/env.local.sh" ]; then
    # shellcheck source=/dev/null
    source "$REPO/scripts/env.local.sh"
fi
ISAAC="${ISAAC:-/isaacsim}"
HOST_PYTHON="${HOST_PYTHON:-$HERE/.venv/bin/python}"
RUN="$(cd "${1:?usage: $0 RUN_DIR}" && pwd)"
[ -f "$RUN/sort_plan.json" ] || { echo "$RUN/sort_plan.json missing: run the pipeline first" >&2; exit 1; }
[ -x "$ISAAC/python.sh" ] || { echo "Not executable: $ISAAC/python.sh (set ISAAC, or chmod +x it)" >&2; exit 1; }

run_sim() {
    # Isaac can exit 0 after an exception; recovery_policy.py treats a missing report as a crash.
    "$ISAAC/python.sh" "$HERE/sort_scene.py" --run "$1" --physics-only 2>&1 \
        | tee "$1/sort.log" | grep -E '^\[sort\]' || true
}

if [ ! -f "$RUN/execution_report.json" ]; then
    echo "=== executing $RUN"
    run_sim "$RUN"
fi

current="$RUN"
n=0
while true; do
    next="${RUN}_retry$((n + 1))"
    set +e
    "$HOST_PYTHON" "$HERE/recovery_policy.py" --run "$current" --out "$next"
    code=$?
    set -e
    case "$code" in
        0)  echo "=== retry $((n + 1)): $next"
            run_sim "$next"
            current="$next"
            n=$((n + 1)) ;;
        10) echo "=== tower complete"; break ;;
        20) echo "=== stopped: needs a person, see $current/recovery_decision.json"; break ;;
        *)  echo "recovery_policy.py failed (exit $code)" >&2; exit "$code" ;;
    esac
done
"$HOST_PYTHON" "$HERE/recovery_policy.py" --summarize "$RUN"
