#!/usr/bin/env bash
#
# mirror.sh — view the desktop Cognitive Workbench (CLI + affect + canvas) on
# this laptop. No code changes to the app: an SSH tunnel forwards the desktop's
# loopback display ports here, so the affect page's hardcoded 127.0.0.1 URL and
# the canvas page's 127.0.0.1-only CSP are satisfied unchanged.
#
# Usage:
#   ./mirror.sh            # = up : tunnel + open affect/canvas + attach CLI
#   ./mirror.sh down       # close the tunnel (leaves the desktop running)
#   ./mirror.sh wb-start   # start the desktop workbench inside tmux (remotable)
#   ./mirror.sh wb-stop    # cleanly shut the desktop workbench down
#   ./mirror.sh wb-restart # wb-stop then wb-start
#   ./mirror.sh status     # what's up, here and there
#
# First time: `ssh-add --apple-use-keychain ~/.ssh/id_ed25519` so there are no
# passphrase prompts.
set -u

HOST="${CWB_HOST:-bruce@192.168.68.76}"
SESSION="cwb"
SOCK="$HOME/.ssh/cwb-mirror.sock"
PORTS=(8787 8788 8789)

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AFFECT="file://$REPO/src/affect/display/static/index.html"
CANVAS="file://$REPO/src/canvas/display/static/index.html"
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

# --- Desktop launch command (Bruce's canonical command). wb-start runs this via
# a login+interactive shell (bash -lic) so it inherits ~/.bashrc — notably
# HF_HOME=/data/huggingface (bge-small embedding cache) and the other env vars
# the agent needs. A bare ssh/tmux command runs a NON-interactive shell, which
# never sources ~/.bashrc (that caused the bge-small cache PermissionError).
# The LLM is a SEPARATE OpenAI-compatible server on 127.0.0.1:5000 you start
# manually each day; jill-chat.yaml has sgl_model_path:null so the launcher
# CONNECTS to it rather than loading a model. wb-start does NOT start the LLM.
# (--resource-browser omitted: it's a local UI not needed from the laptop.) ---
WB_DIR="/home/bruce/Downloads/Cognitive_workbench"
WB_PY="$WB_DIR/zenoh_venv/bin/python3"
WB_INNER="cd $WB_DIR/src && exec $WB_PY launcher.py jill-chat.yaml --cli --affect --canvas --autonomy"

# ── tunnel (SSH ControlMaster, so up/down are idempotent) ──────────────
tunnel_active() { ssh -S "$SOCK" -O check "$HOST" >/dev/null 2>&1; }

tunnel_up() {
  if tunnel_active; then echo "tunnel: already up"; return; fi
  local fwd=(); local p
  for p in "${PORTS[@]}"; do fwd+=(-L "$p:127.0.0.1:$p"); done
  ssh -M -S "$SOCK" -fNT -o ExitOnForwardFailure=yes "${fwd[@]}" "$HOST" \
    && echo "tunnel: up (${PORTS[*]})" \
    || { echo "tunnel: FAILED to start — check 'ssh $HOST'"; return 1; }
}

tunnel_down() {
  if tunnel_active; then ssh -S "$SOCK" -O exit "$HOST" >/dev/null 2>&1; echo "tunnel: closed"; \
  else echo "tunnel: not up"; fi
}

open_views() {
  if [ -x "$CHROME" ]; then
    "$CHROME" --app="$AFFECT" --window-size=320,320 --window-position=60,60 \
      --user-data-dir=/tmp/cwb-affect >/dev/null 2>&1 &
    "$CHROME" --app="$CANVAS" --window-size=820,640 --window-position=540,60 \
      --user-data-dir=/tmp/cwb-canvas >/dev/null 2>&1 &
    echo "views: affect + canvas opened (Chrome app windows)"
  else
    open "$AFFECT"; open "$CANVAS"
    echo "views: opened in default browser"
  fi
}

# ── commands ───────────────────────────────────────────────────────────
cmd_up() {
  tunnel_up || return 1
  open_views
  if ssh -S "$SOCK" "$HOST" "tmux has-session -t $SESSION" >/dev/null 2>&1; then
    echo "cli: attaching (Ctrl-b d detaches and leaves everything running)…"
    ssh -S "$SOCK" -t "$HOST" "tmux attach -t $SESSION"
  else
    echo "cli: no '$SESSION' tmux session on the desktop — not attachable."
    echo "     start a remotable one with:  $0 wb-start"
  fi
}

cmd_down() { tunnel_down; echo "(browser tabs will show 'disconnected — retrying'; close them when done)"; }

wb_start() {
  if ! ssh "$HOST" "command -v tmux" >/dev/null 2>&1; then
    echo "workbench: tmux is not installed on the desktop (needed for a remotable CLI)."
    echo "           install it:  ssh $HOST 'sudo apt update && sudo apt install -y tmux'"
    return 1
  fi
  if ssh "$HOST" "tmux has-session -t $SESSION" >/dev/null 2>&1; then
    echo "workbench: '$SESSION' tmux session already running — attach with: $0 up"; return
  fi
  if ssh "$HOST" "pgrep -f 'launcher.py jill-chat'" >/dev/null 2>&1; then
    echo "workbench: an instance is already running OUTSIDE tmux (can't attach to that one)."
    echo "           stop it at the desktop first, then re-run: $0 wb-start"; return 1
  fi
  if ssh "$HOST" "tmux new-session -d -s $SESSION \"bash -lic '$WB_INNER'\""; then
    echo "workbench: launched in tmux '$SESSION'. Verify it actually came up with: $0 status"
    echo "           (tmux closes the session if the launch line exits — if 'status' shows no"
    echo "            process, the WB_INNER line in this script needs fixing.)"
  else
    echo "workbench: failed to start — see the error above."
  fi
}

wb_stop() {
  if ! ssh "$HOST" "tmux has-session -t $SESSION" >/dev/null 2>&1; then
    echo "workbench: no '$SESSION' tmux session."
    echo "           if one is running outside tmux, stop it at the desktop"
    echo "           (Ctrl-D or double Ctrl-C in its terminal)."
    return
  fi
  echo "workbench: sending clean shutdown (Ctrl-D) — SGLang drain can take ~30s…"
  if ssh "$HOST" "tmux send-keys -t $SESSION C-d; \
       for i in \$(seq 1 45); do tmux has-session -t $SESSION 2>/dev/null || exit 0; sleep 1; done; exit 1"; then
    echo "workbench: stopped cleanly."
  else
    echo "workbench: still up after 45s. Hard-kill:  ssh $HOST 'tmux kill-session -t $SESSION'"
  fi
}

status() {
  if tunnel_active; then echo "tunnel: up"; else echo "tunnel: down"; fi
  ssh "$HOST" "tmux has-session -t $SESSION 2>/dev/null \
       && echo \"desktop: tmux '$SESSION' running\" \
       || echo 'desktop: no $SESSION tmux session'; \
     printf 'workbench proc: '; pgrep -af 'launcher.py jill-chat' || echo none" 2>/dev/null \
    || echo "desktop: unreachable"
}

case "${1:-up}" in
  up)         cmd_up ;;
  down)       cmd_down ;;
  wb-start)   wb_start ;;
  wb-stop)    wb_stop ;;
  wb-restart) wb_stop; wb_start ;;
  status)     status ;;
  *) echo "usage: $0 {up|down|wb-start|wb-stop|wb-restart|status}"; exit 1 ;;
esac
