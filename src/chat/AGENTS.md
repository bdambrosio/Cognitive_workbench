# Chat subproject — coding conventions

This file scopes conventions specific to `src/chat/` (the chat-mode ReAct loop, active-recall subagent, and per-turn evaluator). For repo-wide
conventions see `src/AGENTS.md`.

## ReAct tool-observation convention

Every observation string emitted by a chat ReAct tool **must** start with one of three prefixes so the LLM can discriminate success from failure without keyword-spotting:

- `OK: <content>` — tool succeeded; content follows
- `EMPTY: <reason>` — tool ran cleanly but produced no usable result (no search hits, remember found nothing, empty argument, etc.)
- `ERROR: <reason>` — tool failed (unavailable, raised an exception, returned a malformed result, rejected its arguments). Treat as "this tool is currently broken — do not retry the same call."

Why prose-prefix rather than structured returns: the consumer is an LLM, not a state machine. As long as the prefix is unambiguous, the LLM handles discrimination semantically without code-side typing. Structured returns (à la `_create_uniform_return` in the planner) buy nothing here while costing every tool author a wrapper.

### Authoring rules

- Each `_run_*` method in `chat_loop.py` returns a string already prefixed.
- The dispatch site in `_run_react_loop` does not wrap or post-process — it assigns directly: `obs = self._run_X(...)`.
- For tools where availability or argument validity is checked at the dispatch site (e.g. `search`), the dispatch site emits the prefix itself.
- Diagnostic helpers (e.g. `_diagnose_process_text_args`) return naked reason sentences; the dispatch wraps them with `ERROR:`.
- The convention is also surfaced to the LLM in the ReAct system prompt under "## Observation format" so the agent knows how to read these.

### When to revisit

This convention is intentionally lightweight and unenforced. Move to a runtime layer (typed structured returns, validation in dispatch) when either:

- The tool suite grows past ~8–10 tools and prose conventions actually drift in practice, OR
- A failure mode appears that the LLM cannot solve from prose (e.g. needing to programmatically retry on transient errors but stop on permanent ones with policy outside the LLM's discretion).

Today's failure (search-web tool unavailable, observation conflated three conditions into one string) was prose-fixable — the prefix split is the right level.
