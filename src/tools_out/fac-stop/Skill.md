---
name: fac-stop
description: Immediately stop your Factorio character's current movement. Safety control — use when asked to stop, when you realize you're heading somewhere wrong, or before reassessing after a surprise.
args: {}
---

# fac-stop

Abort the current walk via the game bridge. (The bridge also triggers
this by itself when anyone says "Jill, stop" in game chat — if that
happened, fac-status will show a recent hard stop; hold movement until
things are clarified with the human.)

```json
{"thought": "abort — that's Bruce's build area", "tool": "fac-stop"}
```
