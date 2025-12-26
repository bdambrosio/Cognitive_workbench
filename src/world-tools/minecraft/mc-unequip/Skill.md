---
name: mc-unequip
type: python
description: "Clear or swap equipped item. Returns success/failure."
schema_hint:
  value: "ignored"
  slot: "hand or offhand (default: hand)"
  out: "$variable"
examples:
  - '{"type":"mc-unequip","slot":"hand","out":"$unequip"}'
---

# Minecraft Unequip Tool

## Input
- `slot`: "hand" or "offhand" (optional, default: "hand")

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether unequip succeeded

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-unequip","slot":"hand","out":"$unequip"}
{"type":"mc-equip","item":"wooden_pickaxe","slot":"hand","out":"$equip"}'
```

## Cognitive Contract
- Embodied state change - clears equipped item
- Item returns to inventory (does not drop)

