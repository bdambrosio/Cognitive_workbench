---
name: mc-craft
type: python
description: "Perform crafting via Mineflayer recipe system. Returns success/failure."
schema_hint:
  value: "recipe/item name (preferred)"
  recipe: "recipe/item name (alternative to value)"
  count: "number of items to craft (int, default: 1)"
  out: "$variable"
examples:
  - '{"type":"mc-craft","recipe":"minecraft:stick","count":4,"out":"$craft"}'
  - '{"type":"mc-craft","value":"minecraft:wooden_pickaxe","count":1,"out":"$tool"}'
---

# Minecraft Craft Tool

## Input
- `value` or `recipe`: Recipe/item name (required)
- `count`: Number of items to craft (optional, default: 1)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether craft succeeded
    - `item`: string - item that was crafted
    - `count`: int - number of items crafted
    - `error_code`: string - failure reason code if unsuccessful (e.g., "missing_ingredients", "no_crafting_interface", "recipe_not_known")

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-open","rel_x":0,"rel_y":0,"rel_z":1,"out":"$open"}
{"type":"mc-craft","recipe":"minecraft:stick","count":4,"out":"$craft"}
{"type":"mc-close","out":"$close"}
```

## Cognitive Contract
- High-level crafting via Mineflayer recipe system
- Verifies ingredients in inventory
- Fails if recipe unavailable or ingredients missing
- May require crafting interface open (crafting table) for some recipes

