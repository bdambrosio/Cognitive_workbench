---
name: world-move
description: Walk somewhere in the shared world — to a coordinate, or toward another occupant such as Bruce. Returns immediately with the distance and how long the walk will take; your body keeps walking after the call returns.
args:
  toward: optional string — the name of an occupant to walk to (e.g. "Bruce"). Stops a couple of metres short of them.
  x: optional number — destination x in metres (use with z; ignored if `toward` is given)
  z: optional number — destination z in metres (use with x)
---

# world-move

Post a movement goal. **The call returns as soon as the goal is
accepted, not when you arrive** — walking 30 m takes about 18 seconds of
world time, far longer than a turn. Your body walks in the background
while you go on thinking and talking.

Give either `toward` (an occupant's name) or an `x`/`z` pair.

## What that means for how you use it

- Do not call this repeatedly to "make progress" — one call is the whole
  walk. Calling again replaces the goal and restarts from where you are.
- If you want to know whether you arrived, call `world-look` on a later
  turn. The reply to this call cannot tell you.
- There is no pathfinding. If the straight line runs into water or a
  slope too steep to climb, you stop there rather than going around.
  A later `world-look` will show you short of where you aimed.

## Failure cases worth knowing

- A destination outside the world, in water, or on a cliff is rejected
  outright, with the reason. Nothing moves.
- `toward` someone already standing next to you is rejected — you are
  there.

## Examples

```json
{"thought": "go and stand with Bruce", "tool": "world-move", "toward": "Bruce"}
```

```json
{"thought": "head for the clearing to the north", "tool": "world-move", "x": 12, "z": 60}
```
