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
STREAM_HOST="${TWIN_STREAM_HOST:-127.0.0.1}"
STREAM_TLS_ARGS=()
STREAM_SCHEME=http
if [ -n "${TWIN_TLS_CA:-}${TWIN_TLS_CERT:-}${TWIN_TLS_KEY:-}" ]; then
    : "${TWIN_TLS_CA:?Set all three TWIN_TLS_CA, TWIN_TLS_CERT and TWIN_TLS_KEY for mutual TLS}"
    : "${TWIN_TLS_CERT:?Set all three TWIN_TLS_CA, TWIN_TLS_CERT and TWIN_TLS_KEY for mutual TLS}"
    : "${TWIN_TLS_KEY:?Set all three TWIN_TLS_CA, TWIN_TLS_CERT and TWIN_TLS_KEY for mutual TLS}"
    STREAM_TLS_ARGS=(--tls-ca "$TWIN_TLS_CA" --tls-cert "$TWIN_TLS_CERT" --tls-key "$TWIN_TLS_KEY")
    STREAM_SCHEME=https
fi
RUN="${1:-$HERE/runs/cube_stack_step_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN"
RUN="$(cd "$RUN" && pwd)"
if [ -n "$(ls -A "$RUN")" ]; then
    echo "Use an empty run directory: $RUN" >&2
    exit 1
fi
# Validate keys, trust configuration and binding before starting expensive stages.
"$HOST_PYTHON" "$HERE/twin_stream_server.py" --run "$RUN" --host "$STREAM_HOST" --port "$STREAM_PORT" "${STREAM_TLS_ARGS[@]}" --check-config
"$HOST_PYTHON" "$HERE/twin_stream_server.py" --run "$RUN" --host "$STREAM_HOST" --port "$STREAM_PORT" "${STREAM_TLS_ARGS[@]}" &
STREAM_PID=$!
cleanup() {
    kill "$STREAM_PID" 2>/dev/null || true
}
trap cleanup EXIT
"$ISAAC/python.sh" "$HERE/capture_scene.py" --out "$RUN"
"$COSMOS" "$HERE/perceive.py" --run "$RUN" --stack-cubes
"$HOST_PYTHON" "$HERE/plan_sort.py" --run "$RUN"
SORT_ARGS=(--run "$RUN" --stepwise)
if [ -n "${INJECT_MISSED_GRASP:-}" ]; then
    SORT_ARGS+=(--inject-missed-grasp "$INJECT_MISSED_GRASP")
fi
"$ISAAC/python.sh" "$HERE/sort_scene.py" "${SORT_ARGS[@]}"
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
"$HOST_PYTHON" "$HERE/run_bundle.py" --prepare-download "$RUN"
if [ "$STREAM_SCHEME" = https ]; then
    echo "Review $RUN — connect TkinterView.py to https://<certificate-host>:$STREAM_PORT with the viewer's --tls-ca, --tls-cert and --tls-key (see docs/SECURE_STREAM.md)."
else
    echo "Review $RUN — stream dashboard: python3 TkinterView.py --url http://127.0.0.1:$STREAM_PORT (via SSH tunnel from laptop)"
fi
POST_RUN="${TWIN_STREAM_POST_RUN_SEC:-900}"
if [ "$POST_RUN" != 0 ]; then
    echo "[twin-stream] Stream stays up ${POST_RUN}s on ${STREAM_HOST}:${STREAM_PORT} for dashboard + bundle download (set TWIN_STREAM_POST_RUN_SEC=0 to stop immediately)." >&2
    sleep "$POST_RUN"
fi
