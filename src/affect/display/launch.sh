#!/usr/bin/env bash
# Launch the affect widget as a standalone Chromium/Chrome app window.
# No tabs, no URL bar — just the P5.js canvas. Connects to ws://127.0.0.1:8787
# served by `python -m affect.display`.
#
# Optional env vars:
#   AFFECT_SIZE       window size, default 460,460
#   AFFECT_POS        window position, default 60,60
#   AFFECT_BROWSER    explicit browser binary (skip the search)
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="file://$HERE/static/index.html"
SIZE="${AFFECT_SIZE:-460,460}"
POS="${AFFECT_POS:-60,60}"

# Use a fresh user-data-dir so the WM doesn't inherit a previous
# "maximized" window state (the cause of snap chromium ignoring
# --window-size). Cleaned up by /tmp's usual policy.
PROFILE="/tmp/jill-affect-$$"

if [ -n "$AFFECT_BROWSER" ]; then
    exec "$AFFECT_BROWSER" --app="$URL" \
        --window-size="$SIZE" --window-position="$POS" \
        --user-data-dir="$PROFILE" "$@"
fi

for cmd in chromium chromium-browser google-chrome google-chrome-stable \
           brave-browser microsoft-edge; do
    if command -v "$cmd" >/dev/null 2>&1; then
        exec "$cmd" --app="$URL" \
            --window-size="$SIZE" --window-position="$POS" \
            --user-data-dir="$PROFILE" "$@"
    fi
done

echo "No chromium/chrome-family browser found on PATH." >&2
echo "Open this URL manually in any browser:" >&2
echo "  $URL" >&2
exit 1
