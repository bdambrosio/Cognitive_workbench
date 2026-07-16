---
name: fac-place
description: Place one entity from your Factorio inventory at a position (stone-furnace, iron-chest, burner-inserter, transport-belt, ...). You must be within ~10 tiles; the item must be in your inventory. For belt/pipe/pole RUNS use fac-connect instead.
args:
  prototype: required string — entity prototype name, e.g. "stone-furnace"
  x: required number — target x
  y: required number — target y
  direction: optional string — north | east | south | west (default north)
---

# fac-place

Place a single entity via the game bridge.

## Behavior

- Consumes one `prototype` from your inventory (fac-inventory to check).
- Enforces character reach (~10 tiles): walk close first (fac-walk).
- Failure carries a deviation report: `world_changed` (something
  appeared there since you observed — coordinate, maybe say something),
  `stale_model` (you never observed there recently — fac-observe and
  retry), `infeasible` (collision/invalid with current knowledge).
- Inserters: direction is where the inserter FACES; they pick up from
  the tile behind and drop to the tile in front.
- Catching a machine's output: drills and inserters eject at their
  `outputs to (x, y)` tile shown by fac-observe — place the receiving
  belt/chest EXACTLY there. Entities are wider than one tile (a
  burner-mining-drill is 2x2); tiles adjacent to its center are usually
  inside it and will fail with "blocked by: <the entity itself>".
- Placement grid: 1x1 entities (belt, inserter) sit at coordinates
  ending in .5; 2x2 entities (drill, stone-furnace, chest is 1x1) at
  whole numbers. Pass observed coordinates through EXACTLY — never
  round. An off-grid position is rejected with the valid candidates.

```json
{"thought": "furnace goes here", "tool": "fac-place", "args": {"prototype": "stone-furnace", "x": 48, "y": 60, "direction": "east"}}
```
