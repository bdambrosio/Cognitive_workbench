#!/usr/bin/env bash
#
# run.sh — launch the Cognitive Workbench chat on macOS (Anthropic backend).
#
# Bundles the macOS-specific setup discovered while porting from Linux:
#   * sources secrets from an untracked .env (keeps keys out of git)
#   * sets the OpenMP workaround so the embedding model doesn't segfault
#   * activates the project virtualenv
#   * runs launcher.py from src/ with the jill-chat-mac.yaml scenario
#   * points the affect/canvas widgets at a real browser binary
#
# Secrets/overrides: optional, in an untracked `.env` beside this script.
# The current scenario (jill-chat-mac.yaml) talks to a LAN vLLM server, which
# needs no API key. .env is only needed for the optional Telegram bridge:
#     # export TELEGRAM_BOT_TOKEN=...
#     # export TELEGRAM_ALLOWED_CHAT_IDS=...
#
# Usage:
#     ./run.sh                 # default flags below
#     ./run.sh --debug         # extra args are passed through to launcher.py

set -eo pipefail

# Repo root = the directory this script lives in.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

# Local secrets / overrides (untracked).
if [[ -f "$REPO_DIR/.env" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/.env"
fi

# The current scenario points at a LAN vLLM server, which needs no API key.
# If you switch jill-chat-mac.yaml back to a key-based backend (anthropic/
# openrouter), export that key in .env or your shell — the launcher fails
# loudly at startup if a required key is missing.

# macOS native-library workarounds:
#   KMP_DUPLICATE_LIB_OK — torch and faiss each bundle their own OpenMP
#     runtime; without this the bge-small load segfaults on Apple Silicon.
#   OMP_NUM_THREADS=1     — avoids the duplicated runtimes oversubscribing.
export KMP_DUPLICATE_LIB_OK="${KMP_DUPLICATE_LIB_OK:-TRUE}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

# Virtualenv.
if [[ ! -d "$REPO_DIR/.venv" ]]; then
  echo "ERROR: no virtualenv at $REPO_DIR/.venv" >&2
  echo "       Create it with:" >&2
  echo "         python3.12 -m venv .venv && source .venv/bin/activate \\" >&2
  echo "           && pip install -r <(grep -v '^sglang' requirements.txt)" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$REPO_DIR/.venv/bin/activate"

# Chromium-family browser for the --affect / --canvas widget windows.
# Override with CWB_BROWSER if Chrome lives elsewhere or you use Brave/Edge.
BROWSER="${CWB_BROWSER:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"

# Default flags. Edit this line to drop --affect/--canvas, etc.
FLAGS=(--cli --autonomy --affect --canvas --browser "$BROWSER")

# Add the Telegram bridge only when its env vars are present, so it doesn't
# error-and-skip when unconfigured.
if [[ -n "${TELEGRAM_BOT_TOKEN:-}" && -n "${TELEGRAM_ALLOWED_CHAT_IDS:-}" ]]; then
  FLAGS+=(--telegram)
fi

cd "$REPO_DIR/src"
exec python launcher.py jill-chat-mac.yaml "${FLAGS[@]}" "$@"
