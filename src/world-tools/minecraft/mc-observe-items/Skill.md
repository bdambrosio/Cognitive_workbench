---
name: mc-observe-items
type: python
description: "Exhaustive enumeration of ALL item entities within radius R. Only reports item entities (dropped items), not blocks or other entities"
---

# Minecraft Observe Items Tool

Enumerates all visible item entities (dropped items) within observation radius. Provides structured summary with pose, item counts, nearest items, and pickup affordances.

## Purpose

Exhaustive item observation for inventory management and item collection planning. Returns both human-readable SUMMARY text and machine-readable structured data. Only reports item entities (dropped items), not blocks or other entities.

## Input

- `radius`: Optional integer observation radius (default: `5`, max: `12`)
- `entities_radius`: Optional integer observation radius (default: `5`, max: `12`). If both provided, `entities_radius` takes precedence.
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Multi-line SUMMARY text (human-readable, ≤ 1024 chars)
- `data`: Structured observation dict (machine-readable). Key fields:
  - `success`: Boolean
  - `pose`: `{x, y, z, yaw, pitch}` (floats)
  - `items`: `{total: int, types: {string: int}, nearest: {string: float}, by_distance: [dict], nearby: [dict]}` (nearby is exhaustive raw list)
  - `pickup`: `{in_range: [string], nearby: [string], far: [string]}`
  - `conf`: `"high"` | `"med"` | `"low"`
  - `note`: String (human-readable summary)

## Behavior & Performance

- Exhaustive: Enumerates ALL visible item entities within radius (not sampled)
- Radius limits: Default 5 blocks, maximum 12 blocks
- Confidence levels: `high` (complete, fast), `med` (complete but slow/many items), `low` (incomplete observation)
- Pickup affordances: Categorizes items by pickup range (in_range, nearby, far)

## Guidelines

- Use `mc-observe-blocks` for block observation (this tool only reports item entities)
- `pickup.in_range` indicates items that can be picked up immediately
- `conf=low` indicates incomplete observation - may need retry or larger radius
- Raw exhaustive data stored in `data.items.nearby` for detailed analysis

## Usage Examples

Standard observation:
```json
{"type":"mc-observe-items","out":"$items"}
```

With custom radius:
```json
{"type":"mc-observe-items","radius":8,"out":"$items"}
```
