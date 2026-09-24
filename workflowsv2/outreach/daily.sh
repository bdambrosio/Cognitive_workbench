#!/usr/bin/env bash
# The day's outreach work: start the page if it is not up, run `runner.py daily`,
# and say where to look. Run it from anywhere:
#
#     workflowsv2/outreach/daily.sh [--pool 5] [--want 3]
#
# The four API keys are read from the environment, and from ~/.bashrc for any
# that are not set there (a shell that is not interactive does not load it).
set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PY="$REPO/zenoh_venv/bin/python3"
PORT=8810
LOG="$REPO/workflowsv2/outreach/prospects"
mkdir -p "$LOG"

for k in ATTIO_API_KEY EXA_API_KEY TAVILY_API_KEY CLAUDE_API_KEY; do
  if [ -z "${!k:-}" ]; then
    v="$(grep -m1 "^export $k=" "$HOME/.bashrc" | sed -e "s/^export $k=//" -e 's/^["'"'"']//' -e 's/["'"'"']$//')"
    [ -n "$v" ] && export "$k=$v"
  fi
  [ -n "${!k:-}" ] || { echo "$k is not set, here or in ~/.bashrc"; exit 1; }
done

if ! curl -s -m 5 "http://127.0.0.1:5000/v1/models" > /dev/null; then
  echo "The local model server on port 5000 is not answering; start it, or pass --model <yaml>."
  case " $* " in *" --model "*) ;; *) exit 1 ;; esac
fi

if ! curl -s -m 5 "http://127.0.0.1:$PORT/api/run" > /dev/null; then
  setsid nohup "$PY" "$REPO/workflowsv2/outreach/app.py" --port "$PORT" > "$LOG/app.log" 2>&1 < /dev/null &
  echo "Started the page."
fi

"$PY" "$REPO/workflowsv2/outreach/runner.py" daily "$@" 2> "$LOG/daily.log"
status=$?
[ $status -eq 0 ] || { echo "The daily run failed; the last lines of $LOG/daily.log:"; tail -5 "$LOG/daily.log"; }
echo "Review and send at http://127.0.0.1:$PORT"
exit $status
