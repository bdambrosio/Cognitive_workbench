"""One schema-constrained LLM call, parsed, with everything that could have
silently degraded it recorded.

CONSOLIDATED 2026-09-02 from identical copies in `claims_audit/runner.py` and
`audit_review/runner.py` when a third stage (audit_materiality) needed it.

OUTSIDE THE ReAct LOOP, DELIBERATELY. `REACT_ACTION_SCHEMA` leaves `text`
unconstrained by design, and a large payload stuffed into an action field is a
documented death spiral — seven identical retries and six minutes for zero
progress (subagents/subagent.py). These calls use no tools and answer under
their own schema instead. It is the shape `src/tools/look-at-target/tool.py`
already uses, and the shape `measure/form_grid`'s adjudicate arm holds form in.

TWO MESSAGES, NO HISTORY. The call sees the system text and the user text and
nothing the ChatLoop did in its legs. A stage that needs the legs' reading to
reach the call must put it in `user` itself (the audit hands over its evidence
traces; the review hands over each citation's lines).
"""
from __future__ import annotations

import json
from typing import Any, Callable, Dict, Optional

from utils.json_utils import repair_json_string


def emit(loop, system: str, user: str, schema: Dict[str, Any],
         max_tokens: int,
         salvage: Optional[Callable[[str], Optional[Dict[str, Any]]]] = None
         ) -> Dict[str, Any]:
    """Returns `{"raw", "obj", "parse", "parse_error", "finish", "attempts",
    "response_format_dropped"}`. `parse` is one of parsed, repaired, salvaged,
    unparseable. `salvage`, when given, is tried after `repair_json_string`
    fails and may recover the complete elements of a truncated response.
    """
    before = set(getattr(loop.backend, "_param_drops", set()))
    messages = [{"role": "system", "content": system},
                {"role": "user", "content": user}]

    # RETRY AN EMPTY COMPLETION, ONCE. The local route returns "" with
    # finish=stop often enough that `react.py` carries its own retry for it
    # (REACT_MAX_FORMAT_RETRIES) — and moving the emission outside that loop
    # left the resilience behind. doc9 lost a whole run to it: phase 1 clean,
    # 75 kB of evidence gathered, and a zero-character adjudication.
    #
    # An empty answer is NOT truncation: finish reads `stop`, so nothing was
    # cut and a larger budget would not help. Retrying the identical request is
    # the mitigation because the cause is sampling, not the prompt.
    attempts, raw = [], ""
    for _ in range(2):
        # The read timeout scales with the budget: at 40 tok/s, the slowest
        # rate seen on either route, max_tokens takes max_tokens/40 seconds.
        # Never below the backend's default.
        raw = loop.backend.chat(messages, max_tokens=max_tokens,
                                response_schema=schema,
                                timeout_s=max(300.0, max_tokens / 40.0))
        attempts.append({
            "chars": len(raw or ""),
            "finish": getattr(loop.backend, "last_finish_reason", None),
            # Recorded to tell "generated nothing" from "generated into the
            # reasoning channel and lost on the way out". backend.py already
            # tracks this; nothing new is being measured here.
            "reasoning_chars": getattr(loop.backend,
                                       "last_reasoning_chars", None)})
        if (raw or "").strip():
            break
    dropped = sorted(set(getattr(loop.backend, "_param_drops", set())) - before)
    obj, how, err = None, None, None
    try:
        obj, how = json.loads(raw), "parsed"
    except Exception as e:                                     # noqa: BLE001
        err = f"{type(e).__name__}: {e}"
        repaired = repair_json_string(raw)
        if isinstance(repaired, dict):
            obj, how = repaired, "repaired"
        elif salvage is not None:
            obj = salvage(raw)
            how = "salvaged" if obj else "unparseable"
        else:
            obj, how = None, "unparseable"
    return {"raw": raw, "obj": obj, "parse": how, "parse_error": err,
            "finish": getattr(loop.backend, "last_finish_reason", None),
            "attempts": attempts,
            "response_format_dropped": dropped}
