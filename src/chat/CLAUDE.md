# Chat subproject — coding conventions

This file scopes conventions specific to `src/chat/` (the chat-mode ReAct
loop, active-recall subagent, and per-turn evaluator). Repo-wide rules are
in the root `CLAUDE.md`.

## ReAct tool-observation convention

Every observation string emitted by a chat ReAct tool **must** start with one of three prefixes so the LLM can discriminate success from failure without keyword-spotting:

- `OK: <content>` — tool succeeded; content follows
- `EMPTY: <reason>` — tool ran cleanly but produced no usable result (no search hits, remember found nothing, empty argument, etc.)
- `ERROR: <reason>` — tool failed (unavailable, raised an exception, returned a malformed result, rejected its arguments). Treat as "this tool is currently broken — do not retry the same call."

Why prose-prefix rather than structured returns: the consumer is an LLM, not a state machine. As long as the prefix is unambiguous, the LLM handles discrimination semantically without code-side typing. Structured returns (à la `_create_uniform_return` in the planner) buy nothing here while costing every tool author a wrapper.

### Authoring rules

**Discovered tools (`src/tools/*/tool.py`) do not write the prefix.** They
return `{status: 'ok'|'empty'|'error', text: ...}` and
`_dispatch_discovered_tool` stamps `OK:`/`EMPTY:`/`ERROR:` from `status`.
Writing the prefix into `text` yourself produces `OK: OK: …`.

For the built-ins:

- Each `_run_*` method in `chat_loop.py` returns a string already prefixed.
- The dispatch site in `_run_react_loop` does not wrap or post-process — it assigns directly: `obs = self._run_X(...)`.
- For tools where availability or argument validity is checked at the dispatch site (e.g. `search`), the dispatch site emits the prefix itself.
- Diagnostic helpers (e.g. `_diagnose_process_text_args`) return naked reason sentences; the dispatch wraps them with `ERROR:`.
- The convention is also surfaced to the LLM in the ReAct system prompt under "## Observation format" so the agent knows how to read these.

### Resolved 2026-08-08: it scaled, but not the way this doc predicted

The original trigger was "revisit if the suite grows past ~8–10 tools and
prose conventions drift." The suite is now 36 tools and the convention has
not drifted — but not because prose discipline held. It held because the
discipline moved into code: discovered tools declare a `status` and
`_dispatch_discovered_tool` stamps the prefix, so a tool author cannot get
it wrong. That is the runtime layer this section was holding in reserve,
arrived at from the other direction.

What remains prose-only is the *meaning* of the three states, and that is
the right level — the consumer is an LLM, and the distinction between
"ran, found nothing" and "is broken, do not retry" is semantic.

Revisit only if a failure appears that the LLM cannot solve from prose —
e.g. needing to retry transient errors but stop on permanent ones under
policy outside the model's discretion.
