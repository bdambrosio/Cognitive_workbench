# UI Guide

*(Rewritten 2026-07-19 against the live ChatLoop runtime; the previous
version documented the deleted OODA-era Activation Field / Classic UI
and a Task & Concern Manager that no longer exists.)*

The live interaction surfaces:

| Surface | Flag | Where |
|---|---|---|
| CLI chat | `--cli` | terminal |
| Resource Browser | `--resource-browser` | http://localhost:3001 |
| Affect display | `--affect` | widget window (WS :8787) |
| Canvas display | `--canvas` | widget window (WS :8788, image proxy :8789) |
| Telegram bridge | `--telegram` | Telegram DM |

---

## CLI (`--cli`)

The primary surface: interactive terminal chat driving the ReAct turn
loop. Slash commands (`/help` for the list) are the main way to inspect
and manage agent state — `/concerns`, `/recall`, `/status`, `/note`,
`/img`, `/set-external-repo`, … See [commands.md](commands.md) for the
full reference.

---

## Resource Browser (`--resource-browser`, port 3001)

Web browser over the agent's persistent state: memories, notes,
collections, concerns, traces. Read-write: inline content editing,
resource deletion, and concern management from the UI. Can also run
standalone: `python src/resource_browser.py`. See
[RESOURCE_BROWSER.md](RESOURCE_BROWSER.md).

(The Graph tab queries the deleted executive node and is currently
non-functional — see [cognitive_graph_explorer.md](cognitive_graph_explorer.md).)

---

## Affect display (`--affect`)

A small always-on-top widget window rendering the agent's
processing-state as a face/mood surface (`src/affect/`). Transport:
ChatLoop publishes to Zenoh `cognitive/affect/state`; `python -m
affect.display` bridges to a WebSocket fanout (default `127.0.0.1:8787`,
`AFFECT_WS_HOST`/`AFFECT_WS_PORT`) consumed by
`src/affect/display/static/index.html`. The launcher flag wires all of
this up and opens the window (`--affect-size`, `--affect-pos`,
`--browser` control the window). Design note: the face is a relational
read of processing state only — no content or companion signals on this
surface.

---

## Canvas display (`--canvas`)

A rich-display widget window (`src/canvas/`) the agent can push HTML and
images to mid-turn (the `display` tool). Same bridge pattern as affect:
WebSocket on `:8788` (`CANVAS_WS_PORT`) plus a localhost image proxy on
`:8789` (`CANVAS_HTTP_PORT`). `--canvas-size`, `--canvas-pos` control
the window.

For viewing all of these from another machine, see
[REMOTE_VIEWER_DESIGN.md](REMOTE_VIEWER_DESIGN.md) (`mirror.sh` tunnels
:8787/:8788/:8789).

---

## Telegram bridge (`--telegram`)

DM bridge to the same chat loop (`src/telegram_bridge.py`). Requires
`TELEGRAM_BOT_TOKEN` and `TELEGRAM_ALLOWED_CHAT_IDS`.

---

## Legacy: `--ui` (port 3000)

`--ui` still launches the OODA-era FastAPI display
(`fastapi_action_display.py`). Nothing in it reflects chat-mode state
(no goals, bindings, or OODA pulse exist in the live runtime); it is
retained for archaeology and is of limited use in `mode: chat`.

---

## Browser Extension

An optional Chrome extension (`browser_extension/`) that captures page
visits (URL, title, timestamp) and posts them to a local HTTP listener
on port 5004; the `browser-visits` sensor polls the listener and
delivers visits to the agent as sensor turns.

1. Chrome → Extensions → Enable Developer Mode
2. "Load unpacked" → select `browser_extension/`
3. The listener starts automatically when a character declares a
   `browser-visits` sensor.

---

## Sensors

Declared per-character in the scenario YAML (see
[configuration.md](configuration.md)). Available under `src/sensors/`:
`tick`, `browser-visits`, `rss-watcher`, `obsidian-clipper`,
`factorio-telemetry`. Each carries a `SKILL.md` describing its
parameters; results arrive as user-like turns in the chat loop.
