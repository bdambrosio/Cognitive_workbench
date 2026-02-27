---
name: filter-semantic
type: python
description: "Semantic filter: evaluate each Collection item against a `predicate` string and return matching items (target, predicate, [mode])"
---

# filter-semantic

Apply flexible, natural-language filtering to Collections. Evaluates each item against the `predicate` string using LLM.

## Input

- `target`: Collection ID or variable (required)
- `predicate`: String describing filter criteria (required)
- `mode`: "include" (return matches) or "exclude" (return non-matches) (optional, default: "include")

## Output

Success (`status: "success"`):
- `value`: Summary string (e.g., "3 items [Note_1, Note_2, Note_3]")
- `resource_id`: New Collection ID containing matching items

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- Evaluates each item individually using LLM
- Supports semantic predicates ("has a positive tone"), boolean logic ("A AND B"), numeric/date predicates
- Empty result returns new empty Collection

## Planning Notes

- Use for content-based filtering beyond simple field comparisons
- For field-based selection, use `project` or `pluck` instead
- Predicate should describe what to match, not what to exclude (unless using `mode: "exclude"`)

## Examples

```json
{"type":"filter-semantic","target":"$collection","predicate":"contains code or implementation details","out":"$filtered"}
{"type":"filter-semantic","target":"$collection","predicate":"mentions safety AND published after 2024","out":"$filtered"}
{"type":"filter-semantic","target":"$collection","predicate":"purely theoretical","mode":"exclude","out":"$practical_only"}
```
