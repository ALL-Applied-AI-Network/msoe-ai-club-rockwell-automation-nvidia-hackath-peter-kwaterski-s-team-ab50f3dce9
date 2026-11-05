#!/usr/bin/env bash
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
RUN_DIR=${1:-"$HERE/runs/lego_$(date +%Y%m%d_%H%M%S)"}
ISAAC=${ISAAC:-/isaac-sim}
HOST_PYTHON=${HOST_PYTHON:-"$HERE/.venv/bin/python"}

if [[ -e "$RUN_DIR" ]] && [[ -n "$(find "$RUN_DIR" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    echo "Run directory must be empty: $RUN_DIR" >&2
    exit 2
fi

"$ISAAC/python.sh" "$HERE/lego_scene.py" --run "$RUN_DIR"
"$HOST_PYTHON" "$HERE/encode_video.py" --frames "$RUN_DIR/frames" --out "$RUN_DIR/lego_overview.mp4"
"$HOST_PYTHON" "$HERE/encode_video.py" --frames "$RUN_DIR/side_frames" --out "$RUN_DIR/lego_side.mp4"
"$HOST_PYTHON" - "$RUN_DIR/execution_report.json" <<'PY'
import json, pathlib, sys
report = json.loads(pathlib.Path(sys.argv[1]).read_text())
if not report.get("task_complete"):
    raise SystemExit("LEGO run failed; inspect execution_report.json")
print(f"LEGO complete: {report['verified_pickups']} pickups, {report['verified_placements']} placements")
PY
