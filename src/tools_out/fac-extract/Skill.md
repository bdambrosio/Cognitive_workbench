---
name: fac-extract
description: Take items out of a Factorio machine or container into your inventory (collect smelted plates from a furnace, unload a chest). You must be within reach of the source entity.
args:
  item: required string — item name, e.g. "iron-plate"
  count: required number — how many to take
  x: required number — source entity position x
  y: required number — source entity y
  target: required string — source entity prototype, e.g. "stone-furnace", "iron-chest"
---

# fac-extract

Transfer items from an entity into your inventory via the game bridge.

## Behavior

- The source entity must exist at (x, y) and contain the item (check
  with fac-observe — entity contents are listed).
- Failure carries the standard deviation report.

```json
{"thought": "collect the plates", "tool": "fac-extract", "args": {"item": "iron-plate", "count": 10, "x": 48, "y": 60, "target": "stone-furnace"}}
```
