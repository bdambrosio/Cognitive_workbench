---
name: claude-code
type: python
description: "Delegate coding, research, and analysis tasks to Claude Code CLI. Use target=self to ground any claims about your own architecture, capabilities, or behavior in actual source code rather than generating from general knowledge."
schema_hint:
  action: "string: ask | reset (default: ask)"
  target: "string: sandbox | self (default: sandbox)"
  prompt: "string (required for ask): the instruction to send to Claude Code"
  model: "string (optional): Claude model to use (default: sonnet)"
  max_turns: "integer (optional): max agent turns (default: 10)"
  max_budget: "float (optional): max cost in USD (default: 1.00)"
  timeout: "integer (optional): max seconds to wait (default: 300)"
---

# claude-code

Delegate tasks to Claude Code, a powerful coding agent that can read, write, and execute code. Use this when you need capabilities beyond your own tools — writing scripts, debugging code, running shell commands, or researching codebases.

**IMPORTANT — self-knowledge rule:** Whenever you need to explain, reason about, or make claims about your own architecture, behavior, capabilities, or implementation, use `target: "self"` to inspect your actual source code first. Do NOT generate explanations about yourself from general knowledge — your answers must be grounded in your real codebase. This applies to:
- Explaining how you work to users
- Answering questions about your cognitive architecture (OODA loop, concerns, planning, etc.)
- Debugging your own behavior or errors
- Assessing what you can or cannot do
- Making claims about your own capabilities or limitations

## Targets

### sandbox (default)

Your own workspace at `fs/src/`. Claude Code has full read-write access here. Use this for:
- Writing scripts and programs
- Creating and modifying files
- Running code and shell commands
- Building tools or utilities

### self

Read-only access to the Cognitive Workbench source code — your own implementation. Claude Code can read, search, and analyze the codebase but cannot modify it. Use `target: "self"` whenever you need to:
- **Explain yourself**: "How does my OODA loop work?" — query the code, don't guess
- **Verify your capabilities**: "Can I search the web?" — check what tools actually exist
- **Debug your behavior**: "Why did my planner choose that tool?" — trace the actual logic
- **Ground your claims**: Before telling a user "I work by doing X", confirm X is true in the code
- **Understand your limits**: "What happens when my JSON parsing fails?" — read the repair logic

### When to use self vs generating from memory

| Situation | Use self | Generate directly |
|---|---|---|
| "Explain your OODA loop" | Yes — query code | No |
| "How do you select tools?" | Yes — query code | No |
| "What's 2+2?" | No | Yes — general knowledge |
| "Summarize this article" | No | Yes — use other tools |
| "What concerns do you have?" | No | Yes — check living state |
| "How does your planner work?" | Yes — query code | No |

## Actions

### ask (default)

Send a prompt to Claude Code and get a response. Sessions are persistent per target — follow-up questions maintain conversation context.

- `target` (optional): `"sandbox"` (default) or `"self"`.
- `prompt` (required): The instruction to send to Claude Code.

### reset

Clear a session so the next ask starts a fresh conversation.

- `target` (optional): Which session to reset.

## Output

- `value`: Claude Code's text response.

## Examples

**Explain your own architecture (ALWAYS use self for this):**
```json
{"type": "claude-code", "target": "self", "prompt": "Explain how the OODA loop cognitive control works in this codebase. Cover the main phases, key components, and control flow.", "out": "$explanation"}
```

**Verify a capability claim before making it:**
```json
{"type": "claude-code", "target": "self", "prompt": "What tools do I have for web search? What are their capabilities and limitations?", "out": "$capabilities"}
```

**Debug your own behavior:**
```json
{"type": "claude-code", "target": "self", "prompt": "Look at infospace_executor.py — what happens when _parse_json_response encounters invalid JSON? Trace the repair logic.", "out": "$debug_info"}
```

**Write a script in the sandbox:**
```json
{"type": "claude-code", "prompt": "Write a Python script that fetches RSS feeds from a list of URLs and extracts article titles. Save it as rss_reader.py", "out": "$cc_result"}
```

**Reset a stale session:**
```json
{"type": "claude-code", "action": "reset", "target": "sandbox"}
```
