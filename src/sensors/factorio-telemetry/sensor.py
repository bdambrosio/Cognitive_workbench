"""factorio-telemetry — event ingress from the Factorio game bridge.

Returns plain-text content so the chat loop ingests events as a normal
turn (source `sensor:factorio-telemetry`) and Jill can respond with the
fac-* tools in the same ReAct loop. Jill's own chat lines are skipped
(she would otherwise hear herself and loop).
"""
import json
import logging
import os
import sys
from pathlib import Path

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import BridgeError, bridge_get  # noqa: E402

logger = logging.getLogger(__name__)

_STATE_PATH = Path("~/.cache/cognitive/factorio-telemetry/state.json").expanduser()
_MAX_CHAT_EVENTS = 10


def _load_state():
    if not _STATE_PATH.exists():
        return None
    try:
        return json.loads(_STATE_PATH.read_text())
    except (json.JSONDecodeError, OSError) as e:
        logger.warning(f"factorio-telemetry: unreadable state, reinitializing: {e}")
        return None


def _save_state(state):
    try:
        _STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _STATE_PATH.write_text(json.dumps(state, indent=2))
    except OSError as e:
        logger.warning(f"factorio-telemetry: could not write state: {e}")


def _snapshot(watch_items):
    """One poll of the bridge. Returns (chat, telemetry)."""
    chat = bridge_get("/chat")
    telemetry = bridge_get("/telemetry")
    return chat, telemetry


def run(context):
    watch_items = context.get("parameters", {}).get("watch_items") or []

    try:
        chat, telemetry = _snapshot(watch_items)
    except BridgeError as e:
        logger.warning(f"factorio-telemetry: {e}")
        return {"status": "nothing", "content": "", "metadata": {}}

    players = sorted(telemetry.get("players") or [])
    production = telemetry.get("production") or {}
    if not isinstance(production, dict):
        production = {}
    alert_positions = {}
    for a in telemetry.get("alerts") or []:
        if not isinstance(a, dict):
            continue
        kind = f"{a.get('entity')}: {a.get('issue')}"
        positions = alert_positions.setdefault(kind, set())
        if a.get("x") is not None and a.get("y") is not None:
            positions.add((a["x"], a["y"]))
    alert_kinds_now = sorted(alert_positions)
    flows_live = {
        item: bool(
            (production.get(item) or {}).get("produced_1m")
            or (production.get(item) or {}).get("produced_10m")
        )
        for item in watch_items
    }

    state = _load_state()
    if state is not None and chat.get("last_seq", 0) < state.get("chat_seq", 0):
        # Chat seq went backwards: the world was reset or the mod's chat
        # buffer restarted. Rebaseline silently rather than missing all
        # chat until seq catches up.
        logger.info("factorio-telemetry: chat seq regressed (world reset?) — rebaselining")
        state = None
    if state is None:
        # First run: baseline silently — no history replay.
        _save_state({
            "chat_seq": chat.get("last_seq", 0),
            "players": players,
            "alert_kinds": alert_kinds_now,
            "flows_live": flows_live,
        })
        logger.info("factorio-telemetry: initialized baseline state")
        return {"status": "nothing", "content": "", "metadata": {}}

    events = []

    # game chat from anyone but Jill
    new_chat = [e for e in chat.get("entries") or []
                if e.get("seq", 0) > state.get("chat_seq", 0)
                and e.get("speaker") != "Jill"]
    for entry in new_chat[:_MAX_CHAT_EVENTS]:
        events.append(f"game chat — {entry.get('speaker')}: {entry.get('message')}")
    if len(new_chat) > _MAX_CHAT_EVENTS:
        events.append(f"(and {len(new_chat) - _MAX_CHAT_EVENTS} more chat lines)")

    # players joining/leaving
    before = set(state.get("players") or [])
    for name in sorted(set(players) - before):
        events.append(f"{name} joined the game")
    for name in sorted(before - set(players)):
        events.append(f"{name} left the game")

    # watched production flow stopped
    prior_flows = state.get("flows_live") or {}
    for item in watch_items:
        if prior_flows.get(item) and not flows_live.get(item):
            events.append(f"{item} production has stopped (no flow in the last minute)")

    # new alert kinds
    new_kinds = sorted(set(alert_kinds_now) - set(state.get("alert_kinds") or []))
    for kind in new_kinds:
        positions = sorted(alert_positions.get(kind) or [])
        where = ""
        if positions:
            where = " — at " + ", ".join(f"({x}, {y})" for x, y in positions)
        events.append(f"new alert: {kind}{where}")

    state.update({
        "chat_seq": max(state.get("chat_seq", 0), chat.get("last_seq", 0)),
        "players": players,
        "alert_kinds": alert_kinds_now,
        "flows_live": flows_live,
    })
    _save_state(state)

    if not events:
        return {"status": "nothing", "content": "", "metadata": {}}

    text = "Factorio events since the last check:\n" + "\n".join(f"- {e}" for e in events)
    return {"status": "ok", "content": text,
            "metadata": {"event_count": len(events)}}
