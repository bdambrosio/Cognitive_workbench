---
name: mc-drop
type: python
description: "Drop items from inventory into the world as entities - embodied manipulation. Returns dropped items"
---

# Minecraft Drop Tool

Drops items from inventory into the world as item entities. Embodied manipulation that physically places items in the world.

## Purpose

Item dropping for inventory management, item sharing, and world modification. Creates item entities in the world that can be picked up by the bot or other players.

## Input

- `value`: Item name to drop (preferred)
- `item`: Item name to drop (alternative to value)
- `count`: Integer (optional, default: all)
- `scatter`: Boolean (optional, default: false)

## Output

Returns uniform_return format with:
- `value`: Text summary of dropped items
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `dropped`: Array of `{item: string, count: int}`

## Behavior & Performance

- Drops items as entities in the world
- Default behavior drops all items of specified type
- Scatter option spreads items when dropping multiple

## Guidelines

- Use `mc-inventory` first to check available items
- Item name must match exactly
- Dropped items become entities that can be picked up
- Use `mc-observe-items` to see dropped items

## Usage Examples

Drop one item:
```json
{"type":"mc-drop","item":"stone","count":1,"out":"$drop"}
```

Drop all items of type:
```json
{"type":"mc-drop","value":"dirt","out":"$drop_all"}
```
