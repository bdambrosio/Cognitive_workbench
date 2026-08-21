---
name: fac-pickup
description: Pick up a placed entity (with its contents) or loose items lying on the ground in Factorio into your inventory — e.g. clear ore piled up in front of a blocked drill. You must be within reach of the position (walk there first).
args:
  prototype: required string — entity or item name to pick up (e.g. "iron-ore", "stone-furnace")
  x: required number — position of the entity or ground items
  y: required number — see x
---

# fac-pickup

Pick up the named entity or ground-item stack at (x, y) via the game bridge.

## Behavior

- Placed entities come up with their contents (chest inventory, belt
  items) plus the entity itself. Loose ground items (e.g. ore a blocked
  drill spilled) are picked up as the item name.
- Fails if your inventory cannot hold the items, or nothing matching
  `prototype` is at the position. Failure carries the standard
  deviation report.

```json
{"thought": "clear the ore blocking the drill output", "tool": "fac-pickup", "args": {"prototype": "iron-ore", "x": -54, "y": -31}}
```
