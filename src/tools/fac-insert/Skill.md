---
name: fac-insert
description: Put items from your Factorio inventory into a machine or container (fuel a furnace with coal, load ore, stock a chest). You must be within reach of the target entity.
args:
  item: required string — item name, e.g. "coal", "iron-ore"
  count: required number — how many to insert
  x: required number — target entity position x
  y: required number — target entity y
  target: required string — target entity prototype, e.g. "stone-furnace", "iron-chest"
---

# fac-insert

Transfer items from your inventory into an entity via the game bridge.

## Behavior

- The target entity must exist at (x, y) and accept the item (furnaces
  take fuel + smeltable ore; chests take anything).
- Returns the target's state after the insert (contents included).
- Failure carries the standard deviation report.

```json
{"thought": "fuel the furnace", "tool": "fac-insert", "args": {"item": "coal", "count": 10, "x": 48, "y": 60, "target": "stone-furnace"}}
```
