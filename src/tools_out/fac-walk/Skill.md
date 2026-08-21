---
name: fac-walk
description: Walk your Factorio character to a position. Real walking at game speed (~9 tiles/s) — long hauls take real time and the call blocks until arrival, failure, or timeout. You must be within ~10 tiles of things to act on them.
args:
  x: required number — destination x
  y: required number — destination y
---

# fac-walk

Pathfind and walk to (x, y) via the game bridge. Visible presence: the
character genuinely walks; other players see you coming.

## Behavior

- Blocks until arrival (typical) or returns a failure with a deviation:
  `infeasible` (no path — water, cliffs, dense entities), `timeout`
  (too far for the bridge's walk timeout; you stop where you got to),
  or `stopped` (a human said "Jill, stop" — do not restart movement
  until things are clarified).
- Arriving within ~1.5 tiles of the target counts as arrival.

```json
{"thought": "get within reach of the furnace", "tool": "fac-walk", "args": {"x": 48, "y": 62}}
```
