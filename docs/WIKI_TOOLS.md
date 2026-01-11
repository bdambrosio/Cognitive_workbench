# Tools (Wiki Page Draft)

This page describes tools at two levels:
- **Planner-facing (conceptual)**: the planner reasons over an Infospace of Notes and Collections.
- **Executor-facing (engineering)**: tools are executed by the runtime and results are wrapped for logging/UI.

## Infospace: the planner’s data plane

### Notes

A **Note** is a single persisted resource that contains either:
- **Text** (free-form narrative, excerpts, logs, etc.), or
- A **structured object** (dict/JSON) for machine-readable state/results.

Notes are the primary unit of memory and interchange between tools and planner.

### Collections

A **Collection** is an ordered list of Notes. Common producers:
- web / academic search tools
- filters and transforms (`filter-*`, `map`, `project`, `pluck`, joins)
- batching (group related Notes for later summarization or extraction)

### Why this matters

Most “built-in” tools are CRUD + processing primitives over Notes and Collections:
- **CRUD**: create notes, load notes, save notes, store results
- **Processing**: map/filter/join/project/transform/summarize/extract

The planner should generally treat **structured data as structured Notes**, and use Collections to batch work.

## Tool definitions

### `Skill.md` (contract)

Each tool has a `Skill.md` describing what it does:
- name, type, and short description (frontmatter)
- inputs / outputs (contract)
- behavior and “when to use” guidance (tool selection)

### `tool.py` (implementation)

Python tools implement a `tool(...)` entrypoint and return results via the executor.

### Tool categories

- **Core tools**: `src/tools/`
- **World tools**: `src/world-tools/<world_name>/` (e.g., Minecraft)
- **Primitives**: `src/primitives/` (low-level operators used by plans/tools)

## Canonical Note and Collection fields (recommended)

Tools vary, but many Notes follow these conventions:
- **`text`**: primary content string (for text Notes)
- **`format`**: `"text" | "markdown" | "html" | "json"` (or similar)
- **`metadata.*`**: source/provenance fields such as:
  - `metadata.uri` (source URL)
  - `metadata.domain`
  - `metadata.title`, `metadata.authors`, `metadata.year`

If a tool returns structured JSON, it should place the structured object in a Note (or return it directly as `data` inside `uniform_return`, described below).

## `resource_id` (Infospace-level outputs)

Many tools return a `resource_id` when they create/persist a Note or Collection. Treat it as “a handle to the new resource” and pass it to downstream tools as `target`.

(The specific shape of IDs and where they are stored/loaded is managed by the runtime.)

## uniform_return (executor-level envelope)

All tools communicate results back to the executor/UI using a standardized wrapper returned by `InfospaceExecutor._create_uniform_return()`:
- `result["status"]`: `"success"` | `"failed"`
- `result["data"]`: raw value (dict/list/etc.) on success; on failure, reason/value detail
- `result["value"]`: formatted/truncated string for display/logging
- `result["reason"]`: failure reason string (failure only)

### Practical guidance

- **Programmatic consumers**: use `result["data"]`.
- **UI / logs**: expect `result["value"]` to be truncated and human-oriented.

## Tool selection guidance (what belongs in descriptions)

Frontmatter `description:` should help tool selection:
- good: “Extract fields from unstructured text” / “Search the web and return excerpts”
- good: scope clarifications like “does not report items” (for overlapping tools)
- avoid: implementation details that don’t affect tool choice (e.g., internal training/learning notes)

