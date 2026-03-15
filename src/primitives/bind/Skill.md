---
name: bind
type: primitive
description: Bind a variable to an existing resource without creating or changing content
---

# Bind

## INPUT CONTRACT

- `target` (required): `$variable`, resource ID, or named resource
- `out` (required): Destination `$variable`

**REQUIREMENTS:**
- Target must reference an existing resource
- `out` must be a `$variable`

## BEHAVIOR

Aliases an existing Note/Collection/Relation to a new variable. Bind does **not** mutate resources — it creates a new name pointing to the same underlying resource.

Use cases:
- Rename a variable for clarity (e.g., `$draft` → `$final_report`)
- Create a stable reference before a variable is rebound
- Reference a resource by ID when you don't have it in a variable

## EXAMPLES

```json
{"type": "bind", "target": "$draft", "out": "$final_report"}
```
```json
{"type": "bind", "target": "Note_42", "out": "$summary"}
```

## OUTPUT

Returns `success`. The `out` variable now points to the same resource as `target`.

## FAILURE SEMANTICS

- Target resource not found
- Missing `target` or `out` parameter

## ANTI-PATTERNS

- Using `bind` to copy content — bind creates an alias, not a copy. Both variables point to the same resource.
- Using `bind` when `load` is needed — bind doesn't return content to the planner context, it only creates a variable alias.
