---
name: mc-observe-entities
type: python
description: "Visible entities within radius R (default/max 7), filtered by view cone and line-of-sight."
---

# Minecraft Observe Entities Tool

Enumerates all visible entities (mobs, players, items, etc.) within observation radius. Provides structured summary with pose, entity counts by category and type, nearest entities, and distance-sorted lists.

## Purpose

Visible entity observation for awareness of mobs, players, and items. Returns both human-readable SUMMARY text and machine-readable structured data.

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
  - `entities`: `{total: int, by_category: {string: int}, types: {string: int}, nearest: {string: float}, by_distance: [dict], nearby: [dict]}`
    - Each entry in `entities.nearby` includes: `name`, `type`, `position`, `dx`, `dy`, `dz`, `distance` (items may include `item_name`, `item_count`)
  - `conf`: `"high"` | `"med"` | `"low"`
  - `note`: String (human-readable summary)

## Behavior & Performance

- Enumerates visible entities within radius, filtered by view cone (yaw ±60°, pitch -60..+90) and line-of-sight
- Radius limits: Default 7 blocks, maximum 7 blocks
- Confidence levels: `high` (complete, fast), `med` (complete but slow/many entities), `low` (incomplete observation)
- Entity categorization: Groups entities by category (mob, player, item, other)

## Guidelines

- Use `mc-observe-items` for item-only observation (this tool reports all entity types)
- Use `mc-observe-blocks` for block observation (this tool only reports entities)
- `entities.by_category` provides quick overview of entity types present
- `entities.nearest` shows closest entity of each type
- `conf=low` indicates incomplete observation - may need retry or larger radius
- Raw exhaustive data stored in `data.entities.nearby` for detailed analysis

## Usage Examples

Standard observation:
```json
{"type":"mc-observe-entities","out":"$entities"}
```

With custom radius:
```json
{"type":"mc-observe-entities","radius":7,"out":"$entities"}
```
