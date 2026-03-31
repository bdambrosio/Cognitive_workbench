---
name: claude-code
type: python
description: "Delegate coding, research, and analysis tasks to Claude Code CLI. Supports read-write work in the sandbox repo and read-only inspection of the agent's own source code."
schema_hint:
  action: "string: ask | reset (default: ask)"
  value: "string (required for ask): the prompt/instruction to send to Claude Code"
  target: "string: sandbox | self (default: sandbox)"
  model: "string (optional): Claude model to use (default: sonnet)"
  max_turns: "integer (optional): max agent turns (default: 10)"
  max_budget: "float (optional): max cost in USD (default: 1.00)"
  timeout: "integer (optional): max seconds to wait (default: 300)"
---

# claude-code

Delegate tasks to Claude Code, a powerful coding agent that can read, write, and execute code. Use this when you need capabilities beyond your own tools — writing scripts, debugging code, running shell commands, or researching codebases.

## Targets

### sandbox (default)

Your own workspace at `fs/src/`. Claude Code has full read-write access here. Use this for:
- Writing scripts and programs
- Creating and modifying files
- Running code and shell commands
- Building tools or utilities

### self

Read-only access to the Cognitive Workbench source code — your own implementation. Claude Code can read, search, and analyze the codebase but cannot modify it. Use this to:
- Understand how you work ("How does my concern triage decide what to act on?")
- Debug your own behavior ("Why am I getting this error in the planner?")
- Learn about your capabilities ("What tools do I have for web search?")
- Answer questions about your architecture

## Actions

### ask (default)

Send a prompt to Claude Code and get a response. Sessions are persistent per target — follow-up questions maintain conversation context.

- `value` (required): The prompt or instruction.
- `target` (optional): `"sandbox"` (default) or `"self"`.

### reset

Clear a session so the next ask starts a fresh conversation.

- `target` (optional): Which session to reset.

## Output

- `value`: Claude Code's text response.
- `data.result`: Full response text.
- `data.session_id`: Session ID (managed automatically).
- `data.cost`: Cost of the call in USD.

## Examples

**Write a script in the sandbox:**
```json
{"type": "claude-code", "value": "Write a Python script that fetches RSS feeds from a list of URLs and extracts article titles. Save it as rss_reader.py", "out": "$cc_result"}
```

**Inspect your own source code:**
```json
{"type": "claude-code", "value": "How does the derived concern model decide when to satisfy vs abandon a concern? Show me the key logic.", "target": "self", "out": "$answer"}
```

**Follow up on previous work:**
```json
{"type": "claude-code", "value": "Now add error handling for network timeouts to the script you just wrote", "out": "$cc_result2"}
```

**Debug your own behavior:**
```json
{"type": "claude-code", "value": "Look at infospace_executor.py — what happens when _parse_json_response encounters invalid JSON? Trace the repair logic.", "target": "self", "out": "$debug_info"}
```

**Reset a stale session:**
```json
{"type": "claude-code", "action": "reset", "target": "sandbox"}
```
