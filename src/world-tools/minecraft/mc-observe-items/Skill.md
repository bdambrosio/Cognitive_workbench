---
name: mc-observe-items
type: python
description: "Visible item entities within radius R (default/max 7), filtered by forward cone and line-of-sight."
---

# Minecraft Observe Items Tool

Enumerates all visible item entities (dropped items) within observation radius. Provides structured summary with pose, item counts, nearest items, and pickup affordances.

## Purpose

Visible item observation for inventory management and item collection planning. Returns both human-readable SUMMARY text and machine-readable structured data.

## Input

- `radius`: Optional integer observation radius (default: `7`, max: `7`)
- `entities_radius`: Optional integer observation radius (default: `7`, max: `7`). If both provided, `entities_radius` takes precedence.
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Multi-line SUMMARY text (human-readable, ≤ 1024 chars)
- `data`: Structured observation dict (machine-readable). Key fields:
  - `success`: Boolean
  - `pose`: `{x, y, z, yaw, pitch}` (floats)
  - `items`: `{total: int, types: {string: int}, nearest: {string: float}, by_distance: [dict], nearby: [dict]}`
    - Each entry in `items.nearby` includes: `name`, `type`, `position`, `dx`, `dy`, `dz`, `distance`, `item_name`, `item_count`
  - `pickup`: `{in_range: [string], nearby: [string], far: [string]}`
  - `conf`: `"high"` | `"med"` | `"low"`
  - `note`: String (human-readable summary)

## Behavior & Performance

- Enumerates visible item entities within radius, filtered by view cone (yaw ±60°, pitch -60..+90) and line-of-sight
- Radius limits: Default 7 blocks, maximum 7 blocks
- Confidence levels: `high` (complete, fast), `med` (complete but slow/many items), `low` (incomplete observation)
- Pickup affordances: Categorizes items by pickup range (in_range, nearby, far)

## Guidelines

- Use `mc-observe-blocks` for block observation (this tool only reports item entities)
- `pickup.in_range` indicates items that can be picked up immediately
- `conf=low` indicates incomplete observation - may need retry or larger radius
- Raw data stored in `data.items.nearby` for detailed analysis

## Usage Examples

Standard observation:
```json
{"type":"mc-observe-items","out":"$items"}
```

With custom radius:
```json
{"type":"mc-observe-items","radius":7,"out":"$items"}
```
