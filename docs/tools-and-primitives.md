# Tools & Primitives

The Cognitive Workbench provides two layers of capability: **infospace primitives** (built-in operations over Notes, Collections, and Relations) and **tools** (pluggable extensions loaded from `src/tools/` or world-specific directories).

## Infospace Primitives

Primitives are built into the Executor (`infospace_executor.py`) and always available. They operate on the infospace memory model.

### Resource CRUD

| Primitive | Description |
|-----------|-------------|
| `create-note` | Create a Note with string content |
| `create-collection` | Create a Collection from a list of Note IDs |
| `persist` | Save a transient resource to disk |
| `load` | Load a persistent resource from disk |
| `remove` | Delete a resource (and its Relations) |

### Search & Discovery

| Primitive | Description |
|-----------|-------------|
| `discover-notes` | Semantic search over Notes (uses FAISS) |
| `discover-collections` | Semantic search over Collections |
| `search-within-collection` | Search for Notes within a specific Collection |
| `index` | Add a resource to the FAISS index |
| `organize` | Auto-organize resources by topic |

### Collection Operations

| Primitive | Description |
|-----------|-------------|
| `map` | Apply a transformation to each Note in a Collection |
| `filter-structured` | Filter Notes by metadata criteria |
| `project` | Select specific fields from Notes |
| `pluck` | Extract a single field from each Note |
| `flatten` | Flatten nested Collections |
| `head` | Take the first N Notes from a Collection |
| `sort` | Sort Notes by a field |
| `join` | Join two Collections on a key |
| `union` | Set union of two Collections |
| `intersection` | Set intersection |
| `difference` | Set difference |
| `split` | Split a Note into multiple Notes |
| `add` | Add a Note to a Collection |
| `size` | Count items in a Collection |

### Relations & Metadata

| Primitive | Description |
|-----------|-------------|
| `create-relation` | Create a typed directed edge between resources |
| `find-relations` | Find Relations by type, source, or target |
| `related` | Get resources related to a given resource |
| `get-metadata` | Read metadata from a resource's linked meta-Note |
| `set-metadata` | Write metadata to a resource's linked meta-Note |

### Communication & Reasoning

| Primitive | Description |
|-----------|-------------|
| `say` | Send a message to the user or another agent |
| `ask` | Send a question and wait for a response (triggers envisioning on the receiver) |
| `think` | Internal reflection (logged but not sent to anyone) |
| `bind` | Bind a variable name to a resource in the plan bindings |
| `coerce` | Convert a resource to a different format |

## Tool System Architecture

### Tool Loading

Tools are discovered at startup by `tool_loader.py`:

1. Scan `src/tools/` — core tools, always loaded
2. Scan `src/world-tools/<world_name>/` — world-specific tools, loaded when `world_config.world_name` is set

Each tool directory must contain:
- **`Skill.md`** (or `SKILL.md`) — tool interface definition with YAML frontmatter
- **`tool.py`** — implementation with a `tool()` entry function

### Skill.md Format

```yaml
---
name: my-tool
type: python                  # python | plan | instruction | prompt_augmentation | code_execution
description: "Brief description of what this tool does"
schema_hint:
  param_name: "type and description"
examples:
  - "Example usage"
situational: false            # true = auto-called for context building
hidden: false                 # true = available but hidden from planner catalog
---

Detailed documentation for the LLM planner.
Usage patterns, parameter details, expected outputs.
```

### Tool Types

| Type | Description |
|------|-------------|
| `python` | Python function executed by the Executor |
| `plan` | Reusable plan sequence defined in `plan.json` |
| `instruction` | Text injected into the planner prompt |
| `prompt_augmentation` | Augments the planning context |
| `code_execution` | Arbitrary code execution |

### tool.py Convention

```python
def tool(input_value=None, **kwargs):
    """Tool entry point. Must be named 'tool'."""
    executor = kwargs.get("executor")
    # ... implementation ...
    return executor._create_uniform_return(
        "success",
        value="display text for humans",
        data="structured content for code blocks",
        resource_id="Note_123",  # or None
    )
```

### Uniform Return Format

All tool results are wrapped in a `uniform_return` dict:

| Field | Type | Description |
|-------|------|-------------|
| `status` | `"success"` or `"failed"` | Whether the tool succeeded |
| `value` | string | Human-readable display text |
| `data` | string or list | Structured content: string for Notes, list of `{"text":..., "metadata":...}` for Collections |
| `resource_id` | string or None | ID of the created/modified resource (e.g., `"Note_15"`) |
| `reason` | string | Error description (when status is "failed") |

Code blocks in the planner can inspect `r["data"]` for structured access and `r["value"]` for display.

## Core Tools

These live in `src/tools/` and are always available:

### Research & Web

| Tool | Description |
|------|-------------|
| `search-web` | Web search via Google Custom Search API. Requires `GOOGLE_API_KEY` and `GOOGLE_CX` |
| `semantic-scholar` | Search academic papers, fetch citations, abstracts via Semantic Scholar API |
| `fetch-text` | Fetch and extract text from a URL (uses Playwright for JS-heavy pages) |

### Email & Communication

| Tool | Description |
|------|-------------|
| `check-email` | Read Gmail inbox via IMAP. Requires `GMAIL_ADDRESS` and `GMAIL_APP_PASSWORD` |
| `send-email` | Send email via Gmail SMTP. Requires `GMAIL_ADDRESS` and `GMAIL_APP_PASSWORD` |
| `post-bluesky` | Post to Bluesky social network via AT Protocol |
| `bluesky-instructions` | Returns API instructions for retrieving Bluesky engagement metrics |

### Document Processing

| Tool | Description |
|------|-------------|
| `extract` | LLM-guided extraction or transformation from a single Note |
| `extract-references` | Extract bibliographic references from PDF files (uses GROBID) |
| `extract-struct` | Extract structured metadata (title, authors, year) from text using LLM |

### Analysis & Synthesis

| Tool | Description |
|------|-------------|
| `assess` | Boolean test of text content against a natural language predicate |
| `synthesize` | Cross-document integration, comparison, and reporting from Collections |
| `filter-semantic` | Semantic filter: evaluate Collection items against a predicate |
| `format-citation` | Format paper citations to BibTeX format |

### Knowledge Management

| Tool | Description |
|------|-------------|
| `generate-note` | Generate new text or code content from scratch using LLM |
| `search-obsidian` | Search Obsidian notes via Local REST API. Requires `OBSIDIAN_MCP_URL` |
| `text-find` | Locate pattern or substring and return position with context |

### Computation

| Tool | Description |
|------|-------------|
| `calculate` | Evaluate mathematical expressions (uses SymPy) |
| `word-count` | Count words in text |

### Financial

| Tool | Description |
|------|-------------|
| `stock-price` | Get current stock quote (price, change, volume). Requires `ALPHA_VANTAGE_API_KEY` |

### System & Management

| Tool | Description |
|------|-------------|
| `run-script` | Run a shell script from `src/scripts/` (see below) |
| `create-tool` | Generate a new tool definition (Python or instruction-based) |
| `check-health` | Collect system and cognitive health metrics with status classifications |
| `manage-goals` | Create, list, update, and delete scheduled goals programmatically |

## The run-script Tool

The `run-script` tool executes shell scripts stored in `src/scripts/`. It's designed for fire-and-forget operations like triggering external processes, sending notifications, or running maintenance tasks.

### How it works

1. Accepts a `script_name` parameter (without `.sh` extension)
2. Looks up `src/scripts/{script_name}.sh`
3. Runs it under `bash` in a new session (does **not** wait for completion)
4. Returns immediately with "Script {name} started"

### Adding a script

Create a `.sh` file in `src/scripts/`:

```bash
# src/scripts/daily_report.sh
#!/bin/bash
echo "Generating daily report..." >> /tmp/cwb_reports.log
# your script logic here
```

Then invoke from a goal:
```
goal: Run the daily report script
```

The planner will call: `{"type": "run-script", "script_name": "daily_report"}`

### Safety

- Script names are validated: alphanumeric, hyphens, and underscores only
- Scripts must pre-exist in `src/scripts/` — the tool cannot create or modify scripts
- Execution is fire-and-forget: stdout/stderr go to `/dev/null`

## Plan Tools

Plan tools define reusable action sequences in a `plan.json` file alongside their `Skill.md`:

```json
{
  "plan": [
    {"type": "search-web", "query": "$input", "out": "$results"},
    {"type": "summarize", "target": "$results", "out": "$summary"},
    {"type": "create-note", "name": "research_summary", "value": "$summary", "out": "$note"}
  ],
  "out": "$note"
}
```

Parameters are passed as bound variables: the main input becomes `$input`, and additional arguments become `$key` variables.

See [Tool Development Guide](TOOL_DEVELOPMENT_GUIDE.md) for full details on creating tools.

## World-Specific Tools

When a scenario sets `world_config.world_name`, tools from `src/world-tools/<world_name>/` are loaded alongside core tools. The planner sees them grouped by source (e.g., `#MINECRAFT`, `#FS`).

### Available Worlds

| World | Tools | Description |
|-------|-------|-------------|
| `minecraft` | mc-observe-blocks, mc-navigate, mc-craft, mc-equip, mc-say, mc-map-query, ... | Minecraft bot control |
| `fs` | fs-list, fs-read, fs-grep, fs-find, fs-head, fs-stat | Sandboxed filesystem access |
| `osworld` | osworld-execute, osworld-observe, osworld-status, osworld-reset | Desktop automation |
| `scienceworld` | scienceworld-act, scienceworld-reset | Science simulation |
| `infolab` | fs-list, fs-read, fs-grep, fs-find, fs-head, fs-stat | Default infospace world with sandboxed filesystem |

## Tool Catalog in the Planner

The planner receives a tool catalog organized by source:

```
#MINECRAFT
mc-observe-blocks: Observe nearby blocks in the Minecraft world
mc-navigate: Navigate to coordinates or named location
...

#INFOSPACE CORE
search-web: Search the web via Google Custom Search
semantic-scholar: Search academic papers
create-note: Create a Note with string content
...
```

Situational tools (marked `situational: true`) are auto-called at the start of planning to build context (e.g., `mc-observe-blocks` in Minecraft runs automatically so the planner knows the agent's surroundings).

## Next

- [Architecture](architecture.md) — how the planner uses tools
- [Configuration](configuration.md) — scenario YAML and world config
- [Tool Development Guide](TOOL_DEVELOPMENT_GUIDE.md) — creating new tools
