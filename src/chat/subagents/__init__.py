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

All three subclass `Subagent` in `subagent.py`, which owns the control
loop; a subclass supplies a label, an iteration cap, a system prompt, and
an ordered map of primitives. Trace format is shared via
`utils/subagent_trace.py`.

`recall.py` is the canonical template for a new subagent (see README).
What is deliberately NOT shared is the primitive implementations: recall
reads a gitignored world directory, code_subagent reads a git checkout
with `git ls-files` / ripgrep. Same verb names, different code, on
purpose — see the note in `subagent.py`.
"""
