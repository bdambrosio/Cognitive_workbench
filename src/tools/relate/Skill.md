---
name: relate
type: python
description: "Compare two Notes or Collections to find similarities, differences, and relationships. Use to identify semantic relationships between documents"
---

# relate

Perform semantic comparison of two Notes or Collections to identify overlaps, contradictions, unique elements, and conceptual relationships.

## Input

- `target`: First Note/Collection to compare
- `other`: Second Note/Collection to compare (required)
- `instruction`: Optional natural language guidance (e.g., "focus on methodology differences")

## Output

Success (`status: "success"`):
- `value`: JSON string containing comparison analysis with:
  - `similarity_score`: Float 0-1
  - `shared_themes`: Array of strings
  - `unique_to_first`: Array of strings
  - `unique_to_second`: Array of strings
  - `contradictions`: Array of `{aspect, first, second}`
  - `relationship`: Relationship type string
  - `summary`: Text summary

Failure (`status: "failed"`):
- `reason`: `"missing_parameters"` | `"llm_generate_failed"`

## Behavior

- Auto-flattens Collections and fetches Note content
- Uses LLM for semantic comparison
- Relationship types: duplicate, elaborates, contradicts, complements, supersedes, synthesizes, exemplifies, critiques

## Examples

```json
{"type":"relate","target":"$note1","other":"$note2","out":"$comparison"}
{"type":"relate","target":"$scaling_laws","other":"$compute_optimal","instruction":"identify how scaling laws inform compute-optimal training","out":"$comparison"}
```
