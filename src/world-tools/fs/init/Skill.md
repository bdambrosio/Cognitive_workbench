---
name: init
type: python
description: "Initialize filesystem world by building catalog for prompt context"
---

# Filesystem Initialization Tool

Automatically executed during executor initialization to build a filesystem catalog that provides the agent with awareness of available files and directories in the sandbox.

## Purpose

This tool scans the filesystem sandbox (`scenarios/<world_name>/fs/`) and creates a structured catalog that is automatically included in planner prompts. This gives the agent initial awareness of:
- Directory structure
- Available files and their types
- Directory documentation (if present)

## Behavior

1. **Lightweight Scan**: Performs a limited-depth scan (max depth 2) with entry limits (30 per directory) to avoid performance issues
2. **Directory Documentation**: Reads `Skill.md`, `README.md`, or `DIRECTORY.md` files in directories and includes summaries
3. **Format Detection**: Identifies file types (text, json, binary, pdf) from extensions and samples
4. **Catalog Storage**: Stores the catalog in `world_state['_prompt_sections']['filesystem_catalog']` for generic retrieval by the executor

## Automatic Execution

- This tool is automatically executed during executor initialization if present
- Tool name must be `init` or `<world_name>-init` (e.g., `fs-init`)
- Runs after full tool catalog is loaded
- Executes via `execute_action_with_log()` so it's logged and published

## Output

The catalog is formatted as text and includes:
- Root directory path
- Directory tree structure (limited depth)
- File listings with type and size
- Directory documentation summaries (where available)
- Note about using fs-* tools for detailed exploration

## Limitations

- Limited depth scan (max 2 levels) to control token cost
- Entry limits (30 per directory) to avoid overwhelming prompts
- Performance warning logged if directories exceed 30 entries

## Integration

The catalog is retrieved generically by `executor.get_world_prompt_context()` and included in planner prompts without any world-specific code in the core planner.
