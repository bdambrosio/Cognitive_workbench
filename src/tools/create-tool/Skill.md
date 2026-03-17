---
name: create-tool
type: python
description: "Generate a new tool. Python tools go to staging for review. Instruction tools (no code) are created and registered immediately."
schema_hint:
  name: "string (tool name, e.g. bluesky-instructions)"
  description: "string (one-line description)"
  requirements: "string (what it does, what APIs/libs/env vars/params/output)"
  tool_type: "string (optional): 'python' (default, staged) or 'instruction' (no code, immediate)"
---

# create-tool

Generate a new Cognitive Workbench tool. Supports two modes:

- **python** (default): LLM generates `tool.py` + `Skill.md`, written to `src/tools-staging/{name}/` for review.
- **instruction**: No code generation. The `requirements` text becomes the Skill.md body. Written directly to `src/tools/{name}/` and hot-registered immediately.

## Input

- `name`: Tool name (used as directory and action type, e.g. `bluesky-instructions`)
- `description`: One-line description for the tool catalog
- `requirements`: For python tools: full specification. For instruction tools: the instructional content that becomes the tool output.
- `tool_type`: `"python"` (default) or `"instruction"`

## Output

Success: summary with file paths. For python tools, includes promotion command.

Failure: reason (syntax error, LLM failure, name conflict).

## Behavior

**Python tools:**
- Loads reference tools as examples for the LLM
- Generates `tool.py` and runs `py_compile` to catch syntax errors
- Writes to `src/tools-staging/{name}/` for review
- To activate: review, `mv` to `src/tools/`, restart

**Instruction tools:**
- Writes Skill.md directly to `src/tools/{name}/`
- Hot-registers into running system (available immediately for next goal)
- No restart needed

**Limits:** Only one create-tool call allowed per planner session.

## Common Workflows

**Create an instruction tool from research results:**
```json
{"type":"create-tool","name":"bluesky-instructions","description":"Instructions for retrieving Bluesky engagement metrics via public API","requirements":"Use fetch-text to call https://public.api.bsky.app/xrpc/app.bsky.actor.getProfile?actor={HANDLE} for profile metrics and app.bsky.feed.getAuthorFeed for post engagement. No auth required.","tool_type":"instruction","out":"$result"}
```

**Generate a new Python tool:**
```json
{"type":"create-tool","name":"post-bluesky","description":"Post text to Bluesky social network","requirements":"Use atproto library. Read BLUESKY_HANDLE and BLUESKY_PASSWORD from env. Accept 'text' parameter with post content (max 300 chars). Return success/failure with post URI if successful.","out":"$result"}
```
