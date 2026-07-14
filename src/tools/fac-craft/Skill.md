---
name: fac-craft
description: Hand-craft items in Factorio from materials in your inventory (belts, inserters, chests, gears...). Crafts intermediate ingredients recursively when you have the raw materials.
args:
  item: required string — item to craft, e.g. "transport-belt", "burner-inserter", "iron-chest"
  count: optional number (default 1) — how many
---

# fac-craft

Craft items via the game bridge (FLE's recursive crafter: missing
intermediates are crafted from raw materials when possible).

```json
{"thought": "need belts for the run", "tool": "fac-craft", "args": {"item": "transport-belt", "count": 10}}
```
