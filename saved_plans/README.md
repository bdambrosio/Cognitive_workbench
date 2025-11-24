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

## Execution

Saved plans can be executed directly from the UI or via API:

### UI Execution

1. Navigate to the **Saved** tab in the character sidebar
2. Browse available saved plans
3. Click the **▶️ Execute** button next to the plan you want to run
4. The plan will execute with the character's current bindings
5. Switch to the **Plan** tab to watch execution
6. Check the **Bindings** tab to see variables created

### API Execution

```bash
curl -X POST http://localhost:3000/api/execute_saved_plan \
  -H "Content-Type: application/json" \
  -d '{"plan_name": "goal-search-for-berkeley-ca-weather", "character": "Jill"}'
```

### Cascade Pattern

Plans preserve bindings across executions, enabling composition:

```
Plan A: search-papers    → Creates $papers (Collection)
Plan B: filter-papers    → Uses $papers, creates $focused
Plan C: summarize-papers → Uses $focused, creates $summary
```

Execute them in sequence to build complex workflows from simple, reusable plans.

View current bindings in the **Bindings** tab to see what variables are available.

## Promotion to Full Tool

To promote a saved plan to a full auto-discoverable tool:

```bash
mv saved_plans/my-plan tools/my-plan
```

Then edit `SKILL.md` to remove `manual_only: true` (or set to `false`).

