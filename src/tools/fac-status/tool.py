"""fac-status — position, players, last action, recent game chat."""
import logging
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.factorio_link import bridge_get, fmt_pos, run_tool  # noqa: E402

_log = logging.getLogger(__name__)

_CHAT_TAIL = 5


def _impl():
    st = bridge_get("/status")
    lines = [
        f"You are at {fmt_pos(st['position'])}"
        + (" (walking)" if st.get("walking") else " (idle)")
        + f", server tick {st.get('tick')}."
    ]
    players = st.get("players") or []
    lines.append("Players online: " + (", ".join(players) if players else "none"))
    last = st.get("last_activity")
    if last:
        outcome = "ok" if last.get("ok") else f"FAILED ({last.get('error', '?')})"
        lines.append(f"Your last action: {last.get('endpoint')} {last.get('params')} — {outcome}")
    if st.get("last_hard_stop_s_ago") is not None:
        lines.append(f"NOTE: a hard stop was triggered {st['last_hard_stop_s_ago']}s ago.")
    chat = bridge_get("/chat").get("entries") or []
    if chat:
        lines.append(f"Recent game chat (last {min(len(chat), _CHAT_TAIL)}):")
        lines.extend(f"  {e.get('speaker')}: {e.get('message')}" for e in chat[-_CHAT_TAIL:])
    else:
        lines.append("No game chat yet.")
    return {"status": "ok", "text": "\n".join(lines)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    return run_tool(_impl, logger or _log)
