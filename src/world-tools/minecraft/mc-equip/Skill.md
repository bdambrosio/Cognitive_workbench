---
name: mc-equip
type: python
description: "Equip an item from inventory into hand or offhand. Returns success/failure."
schema_hint:
  value: "item name (preferred)"
  item: "item name (alternative to value)"
  slot: "hand or offhand (default: hand)"
  out: "$variable"
examples:
  - '{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"}'
  - '{"type":"mc-equip","value":"wooden_pickaxe","slot":"hand","out":"$tool"}'
---

# Minecraft Equip Tool

## Input
- `value` or `item`: Item name to equip (required)
- `slot`: "hand" or "offhand" (optional, default: "hand")

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether equip succeeded
    - `equipped`: string - item that was equipped
    - `error_code`: string - failure reason code if unsuccessful (e.g., "item_not_in_inventory", "invalid_slot", "already_equipped")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"}
{"type":"mc-place","item":"stone","rel_x":0,"rel_y":-1,"rel_z":0,"face":"top","out":"$place"}
```

## Cognitive Contract
- Embodied state change - modifies equipped item
- Required before mc-place (item must be equipped)
- May be required before mc-dig (tool-dependent)
- Fails loudly if item not in inventory

