# Saved Plans Directory

This directory contains plans that have been saved as reusable tools. These are **manual invocation only** - they do not appear in the automatic tool catalog used by the incremental planner.

## Structure

Each saved plan is stored in its own subdirectory:

```
saved_plans/
├── research-papers/
│   ├── SKILL.md          # Tool documentation
│   └── plan.json         # Executable plan with goal
├── create-collection/
│   ├── SKILL.md
│   └── plan.json
└── ...
```

## Saving a Plan

1. Execute a goal in the UI
2. Navigate to the **Plan** tab for the character
3. Click **💾 Save as Tool**
4. Fill in the tool name (kebab-case) and description
5. Review detected parameters
6. Click **Save Tool**

## Plan Format

### SKILL.md
- Standard tool documentation with frontmatter
- `manual_only: true` flag prevents auto-discovery
- Parameters are auto-detected from the plan
- Includes original goal for context

### plan.json
```json
{
  "goal": "Original goal text",
  "plan": [...],  // Array of plan actions
  "out": "$result",
  "parameters": {
    "param1": "default_value1",
    "param2": "default_value2"
  },
  "saved_at": "2025-11-23T10:00:00Z"
}
```

## Future: Execution

In the future, `infospace_executor` will be extended to execute these saved plans directly:

```json
{
  "type": "research-papers",
  "args": {
    "query": "LLM agents",
    "limit": 10
  },
  "out": "$result",
  "expect": "..."
}
```

## Promotion to Full Tool

To promote a saved plan to a full auto-discoverable tool:

```bash
mv saved_plans/my-plan tools/my-plan
```

Then edit `SKILL.md` to remove `manual_only: true` (or set to `false`).

