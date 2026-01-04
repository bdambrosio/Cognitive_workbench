---
name: mc-observe-items
type: python
description: "Exhaustive enumeration of ALL item entities within radius R. Only reports item entities (dropped items), not blocks or other entities."
schema_hint:
  value: "ignored"
  radius: "optional observation radius for items (default: 5, max: 12)"
  entities_radius: "optional entities observation radius (default: 5, max: 12)"
  out: "$variable"
examples:
  - '{"type":"mc-observe-items","out":"$items"}'
  - '{"type":"mc-observe-items","radius":5,"out":"$items"}'
---

Tool: mc-observe-items

Purpose:
Summarize nearby item entities (dropped items) and pickup feasibility.

Returns:
A structured SUMMARY with:
- pose
- item summary (total, types, nearest, by_distance)
- pickup feasibility (in_range, nearby, far)
- confidence level

Guarantees:
- SUMMARY always present and ≤ 1024 chars
- Raw exhaustive data stored out-of-band in `data.items.nearby`
- conf=low indicates incomplete observation
- ALL item entities within radius R are returned (exhaustive, not sampled)
- Safe negation: If an item type is not listed, it is not present nearby

Planner must rely ONLY on SUMMARY fields.
