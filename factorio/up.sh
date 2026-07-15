#!/usr/bin/env bash
# Bring up the Factorio side (server + bridge), idempotently. Safe to run
# while Jill is already up: her sensor and tools find the bridge on their
# next poll (<=2 min) — no restart of anything Jill-side needed.
set -euo pipefail
cd "$(dirname "$0")"

docker compose up -d

echo -n "waiting for RCON "
for _ in $(seq 1 30); do
    if .venv/bin/python -c "
import factorio_rcon
pw = open('data/config/rconpw').read().strip()
factorio_rcon.RCONClient('127.0.0.1', 27015, pw).send_command('/silent-command rcon.print(1)')
" 2>/dev/null; then break; fi
    echo -n "."; sleep 2
done
echo " up"

if pgrep -f '\.venv/bin/python -m bridge' > /dev/null; then
    echo "bridge already running"
else
    mkdir -p logs
    nohup .venv/bin/python -m bridge >> logs/bridge.log 2>&1 &
    sleep 3
fi
curl -sf localhost:3004/status > /dev/null && echo "bridge ok (:3004)" \
    || { echo "bridge NOT responding — see logs/bridge.log"; exit 1; }
