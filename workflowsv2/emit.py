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

import copy
import json
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Callable, Dict, List, Optional, Sequence

from utils.json_utils import repair_json_string

# CALLS IN FLIGHT, PER PROCESS. None means one call at a time, the way every
# runner worked before 2026-10-19 and the way a measurement run must still
# work: a run whose calls compete for one rate limit is not a sample of the
# model. The site's production jobs set it with each runner's `--parallel`
# (Bruce, 2026-10-19). The cap is here, around the call, rather than in each
# pool, so pools started inside other pools (materiality's replicates) cannot
# exceed it.
_in_flight: Optional[threading.BoundedSemaphore] = None
_workers = 1


def set_parallel(n: int) -> None:
    """Allow up to `n` calls at once in this process; 1 restores serial."""
    global _in_flight, _workers
    _workers = max(1, int(n))
    _in_flight = threading.BoundedSemaphore(_workers) if _workers > 1 else None


class _LoopView:
    """A loop with its own copy of the backend, for one task on a worker
    thread. The backend records each call's finish reason and reasoning size
    on itself, and `emit` reads them back after the call; two threads sharing
    one backend would read each other's. The copy is shallow, so route-wide
    state (parameters the route dropped or renamed) stays shared; the retry
    counters start empty and are added back to the loop's backend when the
    task ends."""

    def __init__(self, loop):
        self._loop = loop
        self.backend = copy.copy(loop.backend)
        self.backend.transient_events = {}
        self.backend.transient_exhausted = 0

    def __getattr__(self, name):
        return getattr(self._loop, name)

    def fold_back(self) -> None:
        own = self._loop.backend
        for status, n in self.backend.transient_events.items():
            own.transient_events[status] = own.transient_events.get(status, 0) + n
        own.transient_exhausted += self.backend.transient_exhausted


def run_tasks(loop, tasks: Sequence[Callable[[Any], Any]],
              on_done: Optional[Callable[[int, Any], None]] = None) -> List[Any]:
    """Run each task, `task(loop)`, and return the results in task order.

    Serial unless `set_parallel` allowed more: then each task runs on a
    worker thread with a `_LoopView` in place of the loop. `on_done(i,
    result)` runs on this thread as each task finishes, in finishing order;
    a runner writes its partial file there. The first task to raise ends the
    call with that exception, once the other tasks have finished.
    """
    if _workers <= 1 or len(tasks) <= 1:
        results = []
        for i, task in enumerate(tasks):
            results.append(task(loop))
            if on_done is not None:
                on_done(i, results[-1])
        return results
    lock = threading.Lock()

    def run(task):
        view = _LoopView(loop)
        try:
            return task(view)
        finally:
            with lock:
                view.fold_back()

    results: List[Any] = [None] * len(tasks)
    with ThreadPoolExecutor(max_workers=min(_workers, len(tasks))) as pool:
        futures = {pool.submit(run, t): i for i, t in enumerate(tasks)}
        for fut in as_completed(futures):
            i = futures[fut]
            results[i] = fut.result()
            if on_done is not None:
                on_done(i, results[i])
    return results


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
        if _in_flight is not None:
            _in_flight.acquire()
        try:
            raw = loop.backend.chat(messages, max_tokens=max_tokens,
                                    response_schema=schema,
                                    timeout_s=max(300.0, max_tokens / 40.0))
        finally:
            if _in_flight is not None:
                _in_flight.release()
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
    # Only response_format decides whether the answer was schema-constrained.
    # A route that rejects another parameter (gpt-6-luna rejects top_p,
    # 2026-09-22) still constrains the answer, and that drop is logged by the
    # backend.
    dropped = sorted(d for d in set(getattr(loop.backend, "_param_drops", set())) - before
                     if d == "response_format")
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
