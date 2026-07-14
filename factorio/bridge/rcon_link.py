"""RCON transport to the fle-bridge mod's remote interface."""
import json
import logging
import re
import threading

import factorio_rcon
from slpp import slpp

from .policy import ALLOWED_ACTIONS

log = logging.getLogger(__name__)


class ActionError(Exception):
    """A remote action errored inside the mod (Lua error, message extracted)."""


class PolicyError(Exception):
    """Action not in the human-safe allowed set."""


# Lua action errors arrive as:
#   Cannot execute command. Error: Error when running interface function
#   fle_bridge.call: __fle-bridge__/control.lua:NNNN: <message>
#   stack traceback: ...
_LUA_ERROR = re.compile(
    r"interface function fle_bridge\.call:\s*(?:\S+:\d+:\s*)?(?P<msg>.*?)(?:\n\s*stack traceback:|\Z)",
    re.S,
)


def normalize(value):
    """Strip serialize.lua's embedded quotes ('"stone-furnace"' -> 'stone-furnace'), recursively."""
    if isinstance(value, str):
        v = value.strip()
        if len(v) >= 2 and v[0] == '"' and v[-1] == '"':
            return v[1:-1]
        return value
    if isinstance(value, dict):
        return {k: normalize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [normalize(v) for v in value]
    return value


def lua_table(value):
    """Decode a Lua-serialized table string (get_entities / inspect_inventory dumps).

    The mod dumps these as Lua literals, not JSON; slpp is the decoder fle
    itself uses. Int-keyed tables ({1=..., 2=...}) become lists.
    """
    decoded = slpp.decode(value) if isinstance(value, str) else value
    if isinstance(decoded, dict) and decoded and all(isinstance(k, int) for k in decoded):
        decoded = [decoded[k] for k in sorted(decoded)]
    return normalize(decoded)


class RconLink:
    """Serialized RCON client with one reconnect retry per command."""

    def __init__(self, host: str, port: int, password: str):
        self._host, self._port, self._password = host, port, password
        self._client = None
        self._lock = threading.Lock()

    def _ensure(self):
        if self._client is None:
            self._client = factorio_rcon.RCONClient(self._host, self._port, self._password)
        return self._client

    def raw(self, command: str) -> str:
        with self._lock:
            try:
                return self._ensure().send_command(command)
            except (factorio_rcon.RCONBaseError, OSError) as e:
                log.warning("RCON transport error, reconnecting: %s", e)
                self._client = None
                return self._ensure().send_command(command)

    def sc(self, lua: str) -> str:
        """Run a /silent-command chunk; returns whatever it rcon.print()s."""
        return self.raw("/silent-command " + lua)

    def call(self, action: str, *args):
        """remote.call('fle_bridge', 'call', action, ...) with JSON-encoded args.

        Returns the action's (normalized) return value, or None for no output.
        Raises ActionError with the extracted Lua message on action failure.
        """
        if action not in ALLOWED_ACTIONS:
            raise PolicyError(f"action '{action}' is not in the human-safe allowed set")
        # ensure_ascii=False: JSON \uXXXX escapes are invalid Lua string escapes.
        encoded = ", ".join(json.dumps(a, ensure_ascii=False) for a in args)
        sep = ", " if encoded else ""
        resp = self.sc(
            "rcon.print(helpers.table_to_json({remote.call('fle_bridge', 'call', '%s'%s%s)}))"
            % (action, sep, encoded)
        )
        if resp is None or resp == "":
            return None
        if resp.startswith("Cannot execute command"):
            m = _LUA_ERROR.search(resp)
            raise ActionError(m.group("msg").strip() if m else resp.strip())
        out = json.loads(resp)
        return normalize(out[0]) if out else None
