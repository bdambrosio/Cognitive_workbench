---
name: fac-harvest
description: Mine resources or chop trees in Factorio — iron ore, copper ore, coal, stone, wood — into your inventory. You must be within reach of the resource (walk to the patch first).
args:
  x: required number — position of the resource
  y: required number — see x
  count: optional number (default 1) — how many to harvest
---

# fac-harvest

Harvest from the resource under/near (x, y) via the game bridge.

## Behavior

- Harvests whatever resource is at the position (fac-observe shows
  patches). Trees give wood.
- Failure carries the standard deviation report ("Nothing within reach
  to harvest" usually means walk closer or wrong position).

```json
{"thought": "need ore for smelting", "tool": "fac-harvest", "args": {"x": 12, "y": -30, "count": 20}}
```
