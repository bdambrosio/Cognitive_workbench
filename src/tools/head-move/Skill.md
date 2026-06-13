---
name: head-move
description: Move the ChatterBot head — aim the pan/tilt camera or play an expressive gesture. The bot is a stationary companion head; this points its gaze, it does NOT drive or navigate. Use when the user asks you to look somewhere, turn toward/away, look up/down, re-center, or nod/shake/scan. Angles are degrees 0-180 with 90 centered (pan 0=full right, 180=full left; tilt 0=down, 180=up, mounting-dependent). Give pan and/or tilt for absolute aim, OR a gesture (not both). Returns the confirmed pose once the head settles.
args:
  pan: optional number 0-180 — absolute pan (horizontal) angle; 90 is center. Omit to leave pan unchanged.
  tilt: optional number 0-180 — absolute tilt (vertical) angle; 90 is center. Omit to leave tilt unchanged.
  gesture: optional string — one of nod, shake, scan, center. Takes precedence over pan/tilt. nod=yes, shake=no, scan=sweep the room, center=return to 90/90.
  smooth: optional bool (default true) — ease toward the target vs. snap directly. Only applies to pan/tilt moves.
---

# head-move

Publishes a single `chatter/head/cmd` to the ChatterBot Pi and waits for the
head to report it has `arrived` before returning the settled pose. Either aim
(pan/tilt) or a gesture, never both — the Pi gives a gesture precedence and
ignores pan/tilt when a gesture is present.

This is gaze control for a stationary head, not locomotion. There is no
forward/back/drive — the bot does not move through the world.

If you want to *see* what the head is now pointing at, follow this with
`camera-capture`; the camera rides the head, so capture after the move settles.

## Examples

```json
{"thought": "user asked me to look up at them", "tool": "head-move", "tilt": 130}
```

```json
{"thought": "look toward the doorway on the left", "tool": "head-move", "pan": 150, "tilt": 95}
```

```json
{"thought": "agree warmly", "tool": "head-move", "gesture": "nod"}
```
