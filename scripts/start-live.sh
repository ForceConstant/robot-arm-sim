#!/usr/bin/env bash
# Start the live arm dashboard on this host, idempotently.
#
# The launcher used to live in /tmp and disappeared on every reboot, which cost
# a diagnosis cycle.  Configuration comes from live/live_config.json; the port
# defaults to 8080.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
LOG="${LIVE_LOG:-/tmp/live-arm.log}"
PY="${PYTHON:-.venv/bin/python}"
PORT="${LIVE_PORT:-8080}"

if curl -fsS --max-time 2 "http://127.0.0.1:${PORT}/state" >/dev/null 2>&1; then
    echo "live app already answering on ${PORT}"
    exit 0
fi

pkill -f "live/app.py" 2>/dev/null || true
sleep 1
PYTHONPATH="$ROOT" nohup "$PY" live/app.py >"$LOG" 2>&1 &
for _ in $(seq 1 30); do
    if curl -fsS --max-time 2 "http://127.0.0.1:${PORT}/state" >/dev/null 2>&1; then
        echo "live app up on ${PORT} (log $LOG)"
        exit 0
    fi
    sleep 1
done
echo "live app failed to start; tail of $LOG:"
tail -20 "$LOG"
exit 1
