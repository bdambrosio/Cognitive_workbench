---
name: fac-nearest
description: Find the nearest resource in Factorio — iron-ore, copper-ore, coal, stone, water, wood — within 500 tiles of your character. Use when you need ore/fuel and don't know where a patch is; fac-observe then shows the patch details once you're near.
args:
  resource: required string — "iron-ore" (or "iron"), "copper-ore", "coal", "stone", "uranium-ore", "water", "wood"
---

# fac-nearest

Nearest-resource scan via the game bridge (500 tiles around your
character).

## Behavior

- Returns the position of the closest matching resource tile and its
  distance from you. Walk there (fac-walk), then fac-observe to see the
  patch's extent and size, then fac-harvest or place a miner.
- "iron", "copper", "uranium" are accepted shorthand for the -ore names.
- Errors if nothing matches within 500 tiles.

```json
{"thought": "Bruce wants iron — where's the nearest patch?", "tool": "fac-nearest", "args": {"resource": "iron-ore"}}
```
