"""Canvas: rich-display surface for Jill (markdown / HTML rendering).

The `display` ReAct tool publishes content to a per-character Zenoh key;
a separate display process subscribes and pushes payloads to a chromium
app-mode window over WebSocket. Markdown is rendered client-side via
marked.js; HTML is set verbatim into the document body.

Run the display bridge standalone:
    CANVAS_CHARACTER=jill python -m canvas.display
or let `launcher.py --canvas` spawn it for you.
"""
