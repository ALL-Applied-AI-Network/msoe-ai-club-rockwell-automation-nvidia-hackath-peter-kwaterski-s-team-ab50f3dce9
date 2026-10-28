#!/usr/bin/env bash
# LOCAL TESTS ONLY. This is not Isaac Sim and does not model PhysX.
HERE="$(cd "$(dirname "$0")" && pwd)"
HOST_PYTHON="${HOST_PYTHON:-$HERE/../.venv/bin/python}"
exec "$HOST_PYTHON" "$HERE/launch.py" "$@"
