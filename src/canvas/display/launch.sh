#!/usr/bin/env bash
# Launch the canvas widget as a standalone Chromium/Chrome app window.
# Connects to ws://127.0.0.1:8788 served by `python -m canvas.display`.
#
# The canvas bridge needs CANVAS_CHARACTER set; the launcher.py --canvas
# flag is usually easier than running this directly.
#
# Optional env vars:
#   CANVAS_SIZE       window size, default 820,640
#   CANVAS_POS        window position, default 540,60
#   CANVAS_BROWSER    explicit browser binary (skip the search)
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="file://$HERE/static/index.html"
SIZE="${CANVAS_SIZE:-820,640}"
POS="${CANVAS_POS:-540,60}"
PROFILE="/tmp/jill-canvas-$$"

# Fresh profile per launch ⇒ Chrome would show its first-run welcome and
# default-browser prompt every time. Suppress both — this is an app-mode
# display window, not a general browsing session.
COMMON_FLAGS=(--no-first-run --no-default-browser-check)

if [ -n "$CANVAS_BROWSER" ]; then
    exec "$CANVAS_BROWSER" --app="$URL" \
        --window-size="$SIZE" --window-position="$POS" \
        --user-data-dir="$PROFILE" "${COMMON_FLAGS[@]}" "$@"
fi

for cmd in chromium chromium-browser google-chrome google-chrome-stable \
           brave-browser microsoft-edge; do
    if command -v "$cmd" >/dev/null 2>&1; then
        exec "$cmd" --app="$URL" \
            --window-size="$SIZE" --window-position="$POS" \
            --user-data-dir="$PROFILE" "${COMMON_FLAGS[@]}" "$@"
    fi
done

echo "No chromium/chrome-family browser found on PATH." >&2
echo "Open this URL manually:" >&2
echo "  $URL" >&2
exit 1
