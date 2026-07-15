#!/usr/bin/env bash
# Stop the Factorio side (bridge, then server — which saves on stop).
# Jill can stay up: her factory tools/sensor go quiet, nothing errors
# loudly, and everything reconnects on the next up.sh.
set -uo pipefail
cd "$(dirname "$0")"

PID=$(pgrep -f '\.venv/bin/python -m bridge' || true)
if [ -n "$PID" ]; then kill "$PID" && echo "bridge stopped"; else echo "bridge not running"; fi

docker compose stop
