---
name: mc-observe-entities
type: python
description: "Exhaustive enumeration of ALL entities within radius R. Reports all entity types: mobs, players, items, etc."
---

# Minecraft Observe Entities Tool

Enumerates all visible entities (mobs, players, items, etc.) within observation radius. Provides structured summary with pose, entity counts by category and type, nearest entities, and distance-sorted lists.

## Purpose

Exhaustive entity observation for awareness of mobs, players, and items in the environment. Returns both human-readable SUMMARY text and machine-readable structured data. Reports all entity types: mobs, players, items, etc.

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
  - `entities`: `{total: int, by_category: {string: int}, types: {string: int}, nearest: {string: float}, by_distance: [dict], nearby: [dict]}` (nearby is exhaustive raw list)
  - `conf`: `"high"` | `"med"` | `"low"`
  - `note`: String (human-readable summary)

## Behavior & Performance

- Exhaustive: Enumerates ALL visible entities within radius (not sampled)
- Radius limits: Default 5 blocks, maximum 12 blocks
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
{"type":"mc-observe-entities","radius":8,"out":"$entities"}
```
