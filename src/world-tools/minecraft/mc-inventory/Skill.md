---
name: mc-inventory
type: python
description: "Observe inventory contents and equipped items - epistemic, read-only. Returns inventory slots and equipped items."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-inventory","out":"$inv"}'
---

# Minecraft Inventory Tool

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted inventory information
  - `metadata`: raw inventory data including:
    - `slots`: array of slot objects with `slot`, `item`, `count`
    - `equipped`: object with `hand` and `offhand` items

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
- Epistemic observation - read-only, does not modify state
- Required before any action that consumes or uses items
- Inventory state must be observed or cached before item-dependent actions

