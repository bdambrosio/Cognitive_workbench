---
name: create-tool
type: python
description: "Generate a new tool (tool.py + Skill.md) using the LLM and save it to src/tools-staging/ for review. After review, promote with mv and restart."
schema_hint:
  name: "string (tool name, e.g. post-bluesky)"
  description: "string (one-line description)"
  requirements: "string (what it does, what APIs/libs/env vars/params/output)"
---

# create-tool

Generate a new Cognitive Workbench tool using the LLM. The tool is written to `src/tools-staging/{name}/` for review before activation.

## Input

- `name`: Tool name (used as directory and action type, e.g. `post-bluesky`)
- `description`: One-line description for the Skill.md and tool catalog
- `requirements`: Full specification — what the tool does, what library or API it calls, what env vars it reads, what parameters it accepts, and what it returns

## Output

Success: summary Note with file paths and the promotion command to run.

Failure: reason (syntax error in generated code, LLM failure, name already staged).

## Behavior

- Loads 3 reference tools (`stock-price`, `run-script`, `calculate`) as examples for the LLM
- Generates `tool.py` and runs `py_compile` to catch syntax errors before saving
- Generates `Skill.md` with correct frontmatter
- Writes to `src/tools-staging/{name}/` — NOT directly to `src/tools/`
- To activate: review the files, `mv src/tools-staging/{name} src/tools/{name}`, restart the character

## Common Workflows

**Generate a new API wrapper tool:**
```json
{"type":"create-tool","name":"post-bluesky","description":"Post text to Bluesky social network","requirements":"Use atproto library. Read BLUESKY_HANDLE and BLUESKY_PASSWORD from env. Accept 'text' parameter with post content (max 300 chars). Return success/failure with post URI if successful.","out":"$result"}
{"type":"say","target":"User","value":"$result"}
```

**Generate a data-fetching tool:**
```json
{"type":"create-tool","name":"fetch-rss","description":"Fetch and parse an RSS feed, returning recent items as a Note","requirements":"Use feedparser library. Accept 'url' parameter. Return a Note with the 10 most recent items as JSON (title, link, published, summary).","out":"$result"}
```

## Offline Analysis

When the planner fails with `missing_affordance`, opportunities are logged to `logs/planner_history/create_tool_opportunities.jsonl`.

To process the log and generate tools from inferred specs:

```bash
cd src
python3 tools/create-tool/analyze.py --list          # List entries
python3 tools/create-tool/analyze.py --last 3        # Process last 3
python3 tools/create-tool/analyze.py --entry 42      # Process specific seq
```

Uses `VLLM_URL` and `VLLM_MODEL` env vars, or `--vllm-url` and `--vllm-model`.
