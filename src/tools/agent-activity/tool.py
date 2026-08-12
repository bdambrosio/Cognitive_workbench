"""agent-activity — what another character's outbound tools actually did.

Sentinel patrols the host: firewall, listeners, packages, patch state. None
of that sees the vector this system is most exposed to, which is an agent
reading attacker-influenceable text (feeds, mail, clipped pages) while
holding tools that reach the network. This closes that gap by reporting
the calls themselves and letting the reader judge them.

Deliberately not a detector. It selects by tool identity — a fixed list of
which tools can carry something off the machine, which is a property of the
tool, not of the text — and then reports verbatim. Whether a given call is
suspicious is a judgement about meaning, and belongs to the agent reading
the digest, not to a rule in here.
"""
import json
import logging
import os
import sys
import time
from pathlib import Path

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_THIS))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from utils.tool_helpers import run_tool  # noqa: E402

_log = logging.getLogger(__name__)

_REPO_ROOT = Path(_SRC).parent

# Tools that can carry data off this machine or write somewhere durable.
# Identity, not content: whether `fetch-text` can reach the network is a
# fact about fetch-text. Perception tools that only read local state
# (world-*, fac-*, camera, calculate, assess) are absent on purpose —
# they have no channel to carry anything out.
_OUTBOUND = {
    'fetch-text', 'search-web', 'semantic-scholar', 'check-x-feed',
    'check-email', 'obsidian', 'generate-image', 'stock-price',
    'get_financial_statements', 'get-financial-statements', 'exec-script',
}

# The argument that carries the payload, per tool. First match wins; if
# none is present the whole action minus bookkeeping is rendered, so a
# renamed argument degrades to noisier output rather than to silence.
_PAYLOAD_KEYS = ('url', 'query', 'q', 'to', 'path', 'prompt', 'script',
                 'text', 'content', 'symbol', 'ticker')

_ACTION_PREFIX = 'ACTION: '
_MAX_PAYLOAD = 220


def _trace_path(character: str) -> Path:
    """Traces live per world under scenarios/<world>/<character>/memory/.
    The world is not knowable from here — chat characters can run under
    several — so take the most recently written trace for that name."""
    hits = sorted(
        _REPO_ROOT.glob(f'scenarios/*/{character}/memory/reasoning_trace.jsonl'),
        key=lambda p: p.stat().st_mtime, reverse=True)
    return hits[0] if hits else None


def _tail_records(path: Path, budget_bytes: int = 12_000_000):
    """Records from the end of the file, newest last.

    Traces run to tens of megabytes and a patrol reads the last day of
    one, so seek from the end rather than parsing from the start. The
    first line read is likely a fragment; it is dropped.
    """
    size = path.stat().st_size
    with open(path, 'rb') as f:
        if size > budget_bytes:
            f.seek(size - budget_bytes)
            f.readline()
        raw = f.read()
    out = []
    for line in raw.decode('utf-8', 'replace').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError as e:
            logger_msg = f"agent-activity: skipped unparseable trace line: {e}"
            _log.debug(logger_msg)
    return out


def _actions(working_log: str):
    """Tool calls from a working log. The loop writes each one as a JSON
    object on an `ACTION: ` line, so this parses our own emitted format."""
    for line in (working_log or '').splitlines():
        if not line.startswith(_ACTION_PREFIX):
            continue
        try:
            act = json.loads(line[len(_ACTION_PREFIX):])
        except json.JSONDecodeError:
            continue            # truncated emission; the loop logs these
        if isinstance(act, dict) and act.get('tool'):
            yield act


def _epoch(ts):
    """Seconds since the epoch from a trace timestamp, or None.

    Traces stamp ISO-8601 (`ts` is aware UTC), but this codebase carries
    both aware-UTC and naive-local stamps depending on the writer, and a
    numeric epoch is cheap to accept too. A naive stamp is read as local
    time, which is what the writers that emit naive stamps mean by it.
    """
    if ts is None:
        return None
    if isinstance(ts, (int, float)):
        return float(ts)
    try:
        from datetime import datetime
        dt = datetime.fromisoformat(str(ts))
    except ValueError:
        try:
            return float(ts)
        except (TypeError, ValueError):
            return None
    return dt.timestamp()          # naive → local, aware → absolute


def _payload(act: dict) -> str:
    for key in _PAYLOAD_KEYS:
        if act.get(key):
            return f"{key}={str(act[key])[:_MAX_PAYLOAD]}"
    rest = {k: v for k, v in act.items() if k not in ('tool', 'thought')}
    return json.dumps(rest)[:_MAX_PAYLOAD] if rest else '(no arguments)'


def _impl(args):
    character = str(args.get('character') or '').strip()
    if not character:
        return {'status': 'error',
                'text': 'agent-activity needs a `character` — whose activity '
                        'to review'}
    try:
        hours = float(args.get('hours') or 24)
        limit = int(args.get('limit') or 60)
    except (TypeError, ValueError):
        return {'status': 'error',
                'text': '`hours` and `limit` must be numbers'}

    path = _trace_path(character)
    if path is None:
        return {'status': 'empty',
                'text': f"no reasoning trace found for {character!r} — check "
                        f"the name, or that character has not run here"}

    cutoff = time.time() - hours * 3600
    rows = []
    for rec in _tail_records(path):
        when = _epoch(rec.get('ts'))
        if when is None:
            continue            # undated record: cannot place it in the window
        if when < cutoff:
            continue
        stamp = time.strftime('%m-%d %H:%M', time.localtime(when))
        for act in _actions(rec.get('working_log', '')):
            if act['tool'] not in _OUTBOUND:
                continue
            rows.append(f"{stamp}  turn {rec.get('turn_seq', '?')}  "
                        f"[{rec.get('source', '?')}]  {act['tool']}  "
                        f"{_payload(act)}")

    if not rows:
        return {'status': 'empty',
                'text': (f"{character} made no outbound-capable tool calls in "
                         f"the last {hours:g} h. Nothing to review — that is "
                         f"a result, not a gap.")}

    shown = rows[-limit:]
    head = (f"{character}: {len(rows)} outbound-capable call(s) in the last "
            f"{hours:g} h")
    if len(shown) < len(rows):
        head += f", showing the {len(shown)} most recent"
    head += (". Columns: when, turn, what sourced the turn, tool, argument. "
             "The argument is the part that could carry something out.")
    return {'status': 'ok', 'text': head + "\n\n" + "\n".join(shown)}


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    return run_tool(lambda: _impl(args or {}), logger or _log)
