---
name: fac-rotate
description: Rotate a placed Factorio entity (inserter, belt segment, ...) to face a direction. Use when something is oriented wrong — e.g. an inserter moving items the wrong way.
args:
  prototype: required string — entity prototype at the position, e.g. "burner-inserter"
  x: required number — entity position x
  y: required number — entity y
  direction: required string — north | east | south | west
---

# fac-rotate

Rotate an existing entity via the game bridge.

```json
{"thought": "inserter faces the wrong way", "tool": "fac-rotate", "args": {"prototype": "burner-inserter", "x": 49, "y": 61, "direction": "west"}}
```
