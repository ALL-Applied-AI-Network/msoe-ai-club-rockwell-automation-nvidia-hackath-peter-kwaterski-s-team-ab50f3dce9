#!/usr/bin/env bash
# Capture -> Cosmos -> plan -> stepwise physical execution -> review videos.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
if [ -f "$REPO/scripts/env.local.sh" ]; then
    # shellcheck source=/dev/null
    source "$REPO/scripts/env.local.sh"
fi
ISAAC="${ISAAC:-/isaacsim}"
: "${COSMOS:?Set COSMOS (e.g. source $REPO/scripts/env.local.sh)}"
HOST_PYTHON="${HOST_PYTHON:-$HERE/.venv/bin/python}"
export HOST_PYTHON
STREAM_PORT="${TWIN_STREAM_PORT:-8765}"
RUN="${1:-$HERE/runs/cube_stack_step_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN"
RUN="$(cd "$RUN" && pwd)"
if [ -n "$(ls -A "$RUN")" ]; then
    echo "Use an empty run directory: $RUN" >&2
    exit 1
fi
"$HOST_PYTHON" "$HERE/twin_stream_server.py" --run "$RUN" --port "$STREAM_PORT" &
STREAM_PID=$!
cleanup() {
    kill "$STREAM_PID" 2>/dev/null || true
}
trap cleanup EXIT
"$ISAAC/python.sh" "$HERE/capture_scene.py" --out "$RUN"
"$COSMOS" "$HERE/perceive.py" --run "$RUN" --stack-cubes
"$HOST_PYTHON" "$HERE/plan_sort.py" --run "$RUN"
"$ISAAC/python.sh" "$HERE/sort_scene.py" --run "$RUN" --stepwise
"$HOST_PYTHON" - "$RUN" <<'PY'
import json, sys
from pathlib import Path
report = Path(sys.argv[1]) / 'execution_report.json'
if not report.is_file():
    raise SystemExit('Execution report missing; inspect the simulator output.')
result = json.loads(report.read_text())
if not result.get('task_complete'):
    raise SystemExit(f"Stack incomplete: {result.get('abort_reason')}; inspect {report}")
PY
"$HOST_PYTHON" "$HERE/encode_video.py" --frames "$RUN/frames" --out "$RUN/sort_table.mp4" --fps 30
"$HOST_PYTHON" "$HERE/encode_video.py" --frames "$RUN/side_frames" --out "$RUN/sort_side.mp4" --fps 30
echo "Review $RUN — stream dashboard: python3 TkinterView.py --url http://127.0.0.1:$STREAM_PORT (via SSH tunnel from laptop)"
