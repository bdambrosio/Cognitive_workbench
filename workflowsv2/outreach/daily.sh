#!/usr/bin/env bash
# The day's outreach work: start the page if it is not up, run `runner.py daily`,
# then `after_daily.py`, which reads the mailbox for opt-outs and replies, sends
# the first messages that can go by email, and emails a summary. Run it from
# anywhere:
#
#     workflowsv2/outreach/daily.sh [--emails 9] [--most 15]
#
# The API keys come from ~/.config/secrets.env, loaded whole; a value there
# replaces one already in the environment. A shell that is not interactive does
# not load ~/.bashrc, which is what loads that file for a terminal.
#
# When it ends, however it ends, it leaves a report in Jill's inbox folder
# (JILL_INBOX), which her job-reports sensor delivers to her: everything this
# script printed, marked ok or failed.

set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PY="$REPO/zenoh_venv/bin/python3"
PORT=8810
LOG="$REPO/workflowsv2/outreach/prospects"
INBOX="${JILL_INBOX:-$REPO/scenarios/jill_chat/Jill/inbox}"
STARTED="$(date +%Y-%m-%dT%H:%M:%S)"
mkdir -p "$LOG"

main() {
[ -f "$HOME/.config/secrets.env" ] && { set -a; . "$HOME/.config/secrets.env"; set +a; }
for k in EXA_API_KEY TAVILY_API_KEY CLAUDE_API_KEY OPENAI_API_KEY FINDYMAIL_API_KEY PROSPEO_API_KEY; do
  [ -n "${!k:-}" ] || { echo "$k is not set in ~/.config/secrets.env"; exit 1; }
done

if ! curl -s -m 5 "http://127.0.0.1:5000/v1/models" > /dev/null; then
  echo "The local model server on port 5000 is not answering; start it, or pass --model <yaml>."
  case " $* " in *" --model "*) ;; *) exit 1 ;; esac
fi

# The email settings (SMTP_USER, SMTP_PASS, MAIL_FROM, POSTAL_ADDRESS), for the
# page and for after_daily.py; without the file neither sends anything.
[ -f "$HOME/.config/tuuyi-outreach.env" ] && { set -a; . "$HOME/.config/tuuyi-outreach.env"; set +a; }
if ! curl -s -m 5 "http://127.0.0.1:$PORT/api/run" > /dev/null; then
  setsid nohup "$PY" "$REPO/workflowsv2/outreach/app.py" --port "$PORT" > "$LOG/app.log" 2>&1 < /dev/null &
  echo "Started the page."
fi

"$PY" "$REPO/workflowsv2/outreach/runner.py" daily "$@" 2> "$LOG/daily.log"
status=$?
failed=""
[ $status -eq 0 ] || { echo "The daily run failed; the last lines of $LOG/daily.log:"; tail -5 "$LOG/daily.log"; failed="--run-failed"; }
"$PY" "$REPO/workflowsv2/outreach/after_daily.py" $failed 2> "$LOG/after_daily.log" \
  || { status=1; echo "The step after the daily run reported a problem; the last lines of $LOG/after_daily.log:"; tail -3 "$LOG/after_daily.log"; }
exit $status
}

# main runs in a subshell here, so each `exit` above ends main, not the script.
OUT="$(mktemp)"
main "$@" | tee "$OUT"
status=${PIPESTATUS[0]}
"$PY" "$REPO/src/utils/job_report.py" --inbox "$INBOX" --job outreach-daily \
  --status "$([ "$status" -eq 0 ] && echo ok || echo failed)" \
  --started "$STARTED" --body-file "$OUT" > /dev/null \
  || echo "The report for Jill could not be written to $INBOX."
rm -f "$OUT"
exit "$status"
