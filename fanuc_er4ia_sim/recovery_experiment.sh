#!/usr/bin/env bash
# Batch recovery experiment on ONE finished run, then pack the results to share.
#
#   ./recovery_experiment.sh runs/stack_01 matthew_exp1
#
# For each fault below: copy the run, corrupt one grasp, run ./run_with_recovery.sh.
# The FIRST attempt of each variant is the no-recovery baseline; the last attempt
# is the result with recovery. Output: runs/<prefix>_results.tgz (JSON + logs only,
# small enough to upload). Override faults: FAULTS="2:0:0:25 4:3:0:0" ./recovery_experiment.sh ...
# Fault format PICK:DX_MM:DY_MM:DYAW_DEG, or "none" for the unchanged plan.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
if [ -f "$REPO/scripts/env.local.sh" ]; then
    # shellcheck source=/dev/null
    source "$REPO/scripts/env.local.sh"
fi
HOST_PYTHON="${HOST_PYTHON:-$HERE/.venv/bin/python}"
SRC="${1:?usage: $0 FINISHED_RUN PREFIX}"
PREFIX="${2:?usage: $0 FINISHED_RUN PREFIX (e.g. matthew_exp1)}"
FAULTS="${FAULTS:-none 1:0:0:25 3:0:0:25 5:0:0:25 2:3:0:0 4:0:3:0 0:-3:0:0 2:0:0:15}"
cd "$HERE"

bases=()
i=0
for fault in $FAULTS; do
    out="runs/${PREFIX}_$i"
    if [ "$fault" = none ]; then
        "$HOST_PYTHON" make_fault_run.py "$SRC" "$out"
    else
        "$HOST_PYTHON" make_fault_run.py "$SRC" "$out" --fault "$fault"
    fi
    echo "=== [$((i + 1))] $out  fault=$fault  $(date +%H:%M:%S)"
    ./run_with_recovery.sh "$out" | grep -E '^\[recovery\]|^===' || true
    bases+=("$out")
    i=$((i + 1))
done

echo "=== recovery rate"
"$HOST_PYTHON" recovery_policy.py --rate "${bases[@]}" | tee "runs/${PREFIX}_rate.json"
# Pack only small text files: no images or frames.
tar czf "runs/${PREFIX}_results.tgz" "runs/${PREFIX}_rate.json" \
    $(for b in "${bases[@]}"; do ls -d "$b" "$b"_retry* 2>/dev/null; done | while read -r d; do
        for f in sort_plan.json execution_events.json execution_report.json recovery_decision.json \
                 recovery_log.json sort.log; do [ -f "$d/$f" ] && echo "$d/$f"; done; done)
echo "=== packed runs/${PREFIX}_results.tgz ($(du -h "runs/${PREFIX}_results.tgz" | cut -f1)), upload it to Claude"
