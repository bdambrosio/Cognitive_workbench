---
name: fac-connect
description: Build a validated run of transport-belt, pipe, or electric poles between two Factorio positions — the construction macro. Pathfinds around obstacles and places every segment. Use dry_run first to check feasibility and cost.
args:
  from_x: required number — start position x
  from_y: required number — start y
  to_x: required number — end position x
  to_y: required number — end y
  prototype: optional string (default "transport-belt") — what to build the run from, e.g. "transport-belt", "pipe", "small-electric-pole"
  dry_run: optional bool (default false) — validate and count segments without building
---

# fac-connect

The transactional construction macro (cribbed from FLE's
connect_entities): one call builds the whole run or fails as a unit.

## Behavior

- `dry_run: true` reports feasibility and how many segments the run
  needs — your "check my plan" affordance. Use it before committing.
- The real run consumes segments from your inventory (fac-inventory).
- Positions are ground positions. Belts do not connect INTO machines —
  furnaces and chests need an inserter between belt and machine
  (fac-place a burner-inserter).
- Failure carries the standard deviation report (world_changed /
  stale_model / infeasible).

```json
{"thought": "check the belt run cost first", "tool": "fac-connect", "args": {"from_x": 48, "from_y": 66, "to_x": 60, "to_y": 66, "dry_run": true}}
```
