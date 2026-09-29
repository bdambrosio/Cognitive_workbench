#!/usr/bin/env bash
# The day's outreach work: start the page if it is not up, run `runner.py daily`,
# and say where to look. Run it from anywhere:
#
#     workflowsv2/outreach/daily.sh [--strong 5] [--most 15]
#
# The three API keys are read from the environment, and from ~/.config/secrets.env
# for any that are not set there. A shell that is not interactive does not load
# ~/.bashrc, which is what loads that file.

set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PY="$REPO/zenoh_venv/bin/python3"
PORT=8810
LOG="$REPO/workflowsv2/outreach/prospects"
mkdir -p "$LOG"

for k in EXA_API_KEY TAVILY_API_KEY CLAUDE_API_KEY; do
  if [ -z "${!k:-}" ]; then
    v="$(grep -m1 "^export $k=" "$HOME/.config/secrets.env" | sed -e "s/^export $k=//" -e 's/^["'"'"']//' -e 's/["'"'"']$//')"
    [ -n "$v" ] && export "$k=$v"
  fi
  [ -n "${!k:-}" ] || { echo "$k is not set, here or in ~/.config/secrets.env"; exit 1; }
done

if ! curl -s -m 5 "http://127.0.0.1:5000/v1/models" > /dev/null; then
  echo "The local model server on port 5000 is not answering; start it, or pass --model <yaml>."
  case " $* " in *" --model "*) ;; *) exit 1 ;; esac
fi

if ! curl -s -m 5 "http://127.0.0.1:$PORT/api/run" > /dev/null; then
  # The page's email settings (SMTP_USER, SMTP_PASS, MAIL_FROM, POSTAL_ADDRESS);
  # without the file the page offers no email and sends nothing.
  [ -f "$HOME/.config/tuuyi-outreach.env" ] && { set -a; . "$HOME/.config/tuuyi-outreach.env"; set +a; }
  setsid nohup "$PY" "$REPO/workflowsv2/outreach/app.py" --port "$PORT" > "$LOG/app.log" 2>&1 < /dev/null &
  echo "Started the page."
fi

"$PY" "$REPO/workflowsv2/outreach/runner.py" daily "$@" 2> "$LOG/daily.log"
status=$?
[ $status -eq 0 ] || { echo "The daily run failed; the last lines of $LOG/daily.log:"; tail -5 "$LOG/daily.log"; }
echo "Review and send at http://127.0.0.1:$PORT"
exit $status
