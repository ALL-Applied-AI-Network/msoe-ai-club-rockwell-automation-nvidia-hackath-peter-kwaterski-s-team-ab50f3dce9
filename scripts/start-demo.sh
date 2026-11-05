#!/usr/bin/env bash
# Start the Brev demo API, local tunnel, and React development server from WSL.
#
# The Brev instance and its Isaac container must already be running. Override
# BREV_INSTANCE, DEMO_REMOTE_PROJECT, DEMO_CONTAINER, DEMO_API_PORT, or
# DEMO_WEB_PORT when needed.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
FRONTEND_DIR="$REPO_ROOT/demo/frontend"

BREV_INSTANCE=${BREV_INSTANCE:-isaac-launchable-innovation}
DEMO_CONTAINER=${DEMO_CONTAINER:-vscode}
DEMO_REMOTE_PROJECT=${DEMO_REMOTE_PROJECT:-/workspace/msoe-ai-club-rockwell-automation-nvidia-hackath-peter-kwaterski-s-team-ab50f3dce9}
DEMO_API_PORT=${DEMO_API_PORT:-8765}
DEMO_WEB_PORT=${DEMO_WEB_PORT:-5173}

API_BASE="http://127.0.0.1:$DEMO_API_PORT"
WEB_BASE="http://127.0.0.1:$DEMO_WEB_PORT"
TUNNEL_LOG=${TMPDIR:-/tmp}/rockwell-demo-tunnel.log
TUNNEL_PID=""

say() {
    printf '[demo] %s\n' "$*"
}

fail() {
    printf '[demo] ERROR: %s\n' "$*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || fail "Required command not found: $1"
}

local_api_ready() {
    curl --silent --show-error --fail --max-time 2 "$API_BASE/api/health" >/dev/null 2>&1 \
        && curl --silent --show-error --fail --max-time 2 "$API_BASE/api/capabilities" >/dev/null 2>&1
}

frontend_ready() {
    if [[ -x /mnt/c/Windows/System32/curl.exe ]]; then
        /mnt/c/Windows/System32/curl.exe --silent --show-error --fail --max-time 2 \
            "$WEB_BASE" >/dev/null 2>&1 \
            && /mnt/c/Windows/System32/curl.exe --silent --show-error --fail --max-time 2 \
                "$WEB_BASE/api/health" >/dev/null 2>&1
    else
        curl --silent --show-error --fail --max-time 2 "$WEB_BASE" >/dev/null 2>&1 \
            && curl --silent --show-error --fail --max-time 2 \
                "$WEB_BASE/api/health" >/dev/null 2>&1
    fi
}

remote_api_ready() {
    brev exec "$BREV_INSTANCE" \
        "curl -fsS --max-time 2 http://127.0.0.1:$DEMO_API_PORT/api/health >/dev/null" \
        >/dev/null 2>&1
}

port_is_open() {
    (exec 3<>"/dev/tcp/127.0.0.1/$1") >/dev/null 2>&1
}

cleanup() {
    if [[ -n "$TUNNEL_PID" ]] && kill -0 "$TUNNEL_PID" 2>/dev/null; then
        say "Stopping the local Brev tunnel..."
        kill -- "-$TUNNEL_PID" 2>/dev/null || kill "$TUNNEL_PID" 2>/dev/null || true
        wait "$TUNNEL_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT TERM

require_command brev
require_command curl
require_command npm
require_command setsid

[[ -d "$FRONTEND_DIR" ]] || fail "Frontend directory not found: $FRONTEND_DIR"

say "Checking Brev instance '$BREV_INSTANCE'..."
brev exec "$BREV_INSTANCE" "docker inspect $DEMO_CONTAINER >/dev/null" >/dev/null \
    || fail "Could not reach container '$DEMO_CONTAINER' on '$BREV_INSTANCE'"

if remote_api_ready; then
    say "Backend API is already healthy on Brev."
else
    say "Backend API is not responding; starting it in the Isaac container..."
    REMOTE_START="cd $DEMO_REMOTE_PROJECT && source scripts/env.local.sh && exec fanuc_er4ia_sim/.venv/bin/python fanuc_er4ia_sim/demo_api.py --port $DEMO_API_PORT > /tmp/rockwell-demo-api.log 2>&1"
    brev exec "$BREV_INSTANCE" \
        "docker exec -d $DEMO_CONTAINER bash -lc '$REMOTE_START'" >/dev/null

    for _attempt in $(seq 1 30); do
        if remote_api_ready; then
            break
        fi
        sleep 1
    done
    if ! remote_api_ready; then
        brev exec "$BREV_INSTANCE" \
            "docker exec $DEMO_CONTAINER tail -n 80 /tmp/rockwell-demo-api.log" >&2 || true
        fail "Backend API did not become healthy"
    fi
    say "Backend API started."
fi

if local_api_ready; then
    say "A working local API tunnel already exists on port $DEMO_API_PORT."
else
    if port_is_open "$DEMO_API_PORT"; then
        fail "Local port $DEMO_API_PORT is occupied by something other than the demo API"
    fi
    say "Forwarding localhost:$DEMO_API_PORT to Brev..."
    : >"$TUNNEL_LOG"
    setsid brev port-forward "$BREV_INSTANCE" \
        -p "$DEMO_API_PORT:$DEMO_API_PORT" >"$TUNNEL_LOG" 2>&1 &
    TUNNEL_PID=$!

    for _attempt in $(seq 1 40); do
        if local_api_ready; then
            break
        fi
        if ! kill -0 "$TUNNEL_PID" 2>/dev/null; then
            sed -n '1,120p' "$TUNNEL_LOG" >&2
            fail "Brev port forwarding stopped unexpectedly"
        fi
        sleep 0.5
    done
    if ! local_api_ready; then
        sed -n '1,120p' "$TUNNEL_LOG" >&2
        fail "The local API tunnel did not become healthy"
    fi
    say "API tunnel is ready."
fi

cd "$FRONTEND_DIR"
say "Installing/updating React dependencies..."
npm install

if frontend_ready; then
    say "React is already available at $WEB_BASE"
    if [[ -n "$TUNNEL_PID" ]]; then
        say "Keep this terminal open to retain the tunnel; press Ctrl+C to stop it."
        wait "$TUNNEL_PID"
    fi
    exit 0
fi

# WSLENV is needed when WSL resolves npm to the Windows Node installation.
export DEMO_API_URL="$API_BASE"
case ":${WSLENV:-}:" in
    *:DEMO_API_URL:*) ;;
    *) export WSLENV="${WSLENV:+$WSLENV:}DEMO_API_URL" ;;
esac

say "Starting React at $WEB_BASE"
say "The Brev backend will remain running after this script exits."
say "Press Ctrl+C to stop React and any tunnel created by this script."
npm run dev -- --port "$DEMO_WEB_PORT" --open
