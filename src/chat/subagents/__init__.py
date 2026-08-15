"""Tool subagents invoked from the chat ReAct loop.

Each module here is a thin, persona-less ReAct loop reached through one
builtin tool in `react.py`. From the calling agent's vantage a call is a
single step: it emits a query string and gets a synthesized answer back;
the subagent's own iterations are opaque to the caller's trace and are
written to a per-call trace file instead.

  recall.py         → `recall` tool, over the per-world memory dir
  code_subagent.py  → `inspect` and `inspect_external` tools, over a
                      fenced source root
  security.py       → `security` tool, typed host/LAN probes

`recall.py` is the canonical template for a new subagent (see README).
All share the trace format in `utils/subagent_trace.py`; the loop bodies
are still per-module and are the subject of the planned consolidation.
"""
