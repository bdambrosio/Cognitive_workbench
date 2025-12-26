---
name: mc-drop
type: python
description: "Drop items from inventory into the world as entities - embodied manipulation. Returns dropped items."
schema_hint:
  value: "item name (preferred)"
  item: "item name (alternative to value)"
  count: "number of items to drop (int, optional, default: all)"
  scatter: "scatter items (bool, optional, default: false)"
  out: "$variable"
examples:
  - '{"type":"mc-drop","item":"stone","count":1,"out":"$drop"}'
  - '{"type":"mc-drop","value":"dirt","out":"$drop_all"}'
---

# Minecraft Drop Tool

## Input
- `value` or `item`: Item name to drop (required)
- `count`: Number of items to drop (optional, default: all)
- `scatter`: Whether to scatter items (optional, default: false)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: summary of dropped items
  - `metadata`: raw response including:
    - `success`: boolean - whether drop succeeded
    - `dropped`: array of items dropped with `item` and `count`
    - `error_code`: string - failure reason code if unsuccessful (e.g., "item_not_in_inventory", "insufficient_count")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-unequip","slot":"hand","out":"$unequip"}
{"type":"mc-drop","item":"stone","count":1,"out":"$drop"}
```

## Cognitive Contract
- Embodied manipulation - removes items from inventory and spawns item entities
- Dropped items are no longer equipable
- Dropped items must be re-picked up to use again
- Drop ≠ unequip (unequip moves to inventory, drop removes from inventory entirely)
- No implicit drop - inventory never silently spills

