#!/usr/bin/env python3
"""Reading a driven turn: what the agent replied, and how the turn ended.

EXTRACTED FROM `workflowsv2/claims_audit/runner.py` 2026-08-30, unchanged, when
audit_postprocess became the third caller. audit_review already imported them
from there and inherited what that costs: that module calls
`logging.basicConfig` at import, so importing it from another entry point
reconfigures the host's logging. This module does neither.

Both are small and neither is obvious, which is why they are shared rather than
retyped — see the docstrings.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

REPO = Path(__file__).resolve().parent.parent


def last_exit_reason(world: str, agent: str) -> Optional[str]:
    """Read it off the trace, not the loop. `exit_reason` is a local inside
    _process_user_turn and is never stored on the object — getattr for it
    returns None, which reads as "not a yield" and would end every run after
    one leg."""
    p = (REPO / "scenarios" / world / agent / "memory" /
         "reasoning_trace.jsonl")
    if not p.exists():
        return None
    last = None
    for line in p.open(errors="replace"):
        line = line.strip()
        if line:
            last = line
    if not last:
        return None
    try:
        return json.loads(last).get("exit_reason")
    except json.JSONDecodeError:
        return None


def latest_reply(loop, source: str) -> str:
    turns = loop.store.get_recent_turns(source, limit=4, scope="all")
    # The store marks a turn by `direction` ("in" from the source, "out"
    # from the agent); it has no `name` field. The first version tested a
    # field that was never there and passed only because the last turn is
    # the agent's (found 2026-09-03 while building history for the page).
    for t in reversed(turns):
        if t.get("direction") == "out":
            return str(t.get("text", ""))
    return ""


