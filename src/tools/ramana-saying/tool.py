"""
ramana-saying — return one genuine, attributed saying of Ramana Maharshi.

Samples a single question -> answer exchange from the bundled, verbatim Talks
pool (data/sayings.jsonl, built by build_pool.py) and returns it as markdown
with source attribution. No LLM call here: the ReAct loop adds its own framing
and renders the card via the `display` tool.

A small recent-history file avoids repeating the same saying back-to-back.
"""

import json
import logging
import random
from pathlib import Path
from typing import Any, Dict

_logger = logging.getLogger(__name__)

_POOL_PATH = Path(__file__).parent / "data" / "sayings.jsonl"
_RECENT_PATH = Path("~/.cache/cognitive/ramana-saying/recent.json").expanduser()
_RECENT_KEEP = 20


def _load_pool() -> list[dict]:
    sayings = []
    for line in _POOL_PATH.read_text().splitlines():
        line = line.strip()
        if line:
            sayings.append(json.loads(line))
    return sayings


def _load_recent() -> list[str]:
    if not _RECENT_PATH.exists():
        return []
    try:
        return json.loads(_RECENT_PATH.read_text())
    except (json.JSONDecodeError, OSError) as e:
        _logger.warning(f"ramana-saying: could not read recent history, ignoring: {e}")
        return []


def _save_recent(recent: list[str]) -> None:
    try:
        _RECENT_PATH.parent.mkdir(parents=True, exist_ok=True)
        _RECENT_PATH.write_text(json.dumps(recent[-_RECENT_KEEP:]))
    except OSError as e:
        _logger.warning(f"ramana-saying: could not write recent history: {e}")


def _format(saying: dict) -> str:
    context = (saying.get("context") or "").strip()
    question = (saying.get("question") or "").strip()
    answer = (saying.get("answer") or "").strip()
    talk = saying.get("talk") or "Talks with Sri Ramana Maharshi"
    lines = []
    if context:
        lines.append(f"*{context}*")
    if question:
        lines.append(f"**Q.** {question}")
    lines.append(f"> {answer}")
    lines.append(f"\n— Sri Ramana Maharshi, *Talks with Sri Ramana Maharshi* ({talk})")
    return "\n\n".join(lines)


def react_invoke(args: Dict[str, Any], *, character_name=None, backend=None, logger=None) -> Dict[str, Any]:
    log = logger or _logger
    try:
        pool = _load_pool()
    except OSError as e:
        log.error(f"ramana-saying: could not load sayings pool {_POOL_PATH}: {e}")
        return {"status": "error", "text": f"sayings pool unavailable: {e}"}

    if not pool:
        log.error(f"ramana-saying: sayings pool {_POOL_PATH} is empty")
        return {"status": "error", "text": "sayings pool is empty"}

    recent = _load_recent()
    candidates = [s for s in pool if s.get("id") not in recent] or pool
    saying = random.choice(candidates)

    sid = saying.get("id")
    if sid:
        recent.append(sid)
        _save_recent(recent)

    log.info(f"ramana-saying: selected {sid} ({saying.get('talk')})")
    return {"status": "ok", "text": _format(saying)}
