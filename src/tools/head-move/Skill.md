---
name: head-move
description: Aim the ChatterBot head — the pan/tilt camera on Bruce's desk, in the physical room. This points the camera; it does not drive, navigate, or gesture. Use when you want to look somewhere in the room (toward the door, up at someone standing, down at the desk) or return to the resting pose. Angles are degrees. Pan 10-170, 90 straight ahead, larger is further left. Tilt 30-150, 113 is level, smaller is up, larger is down. Give pan and/or tilt, or center true. Returns the confirmed pose once the head settles.
args:
  pan: optional number 10-170 — absolute pan (horizontal) angle; 90 is straight ahead, 170 is full left, 10 is full right. Omit to leave pan unchanged.
  tilt: optional number 30-150 — absolute tilt (vertical) angle; 113 is level, 30 is fully up, 150 is about 45 degrees down. Omit to leave tilt unchanged.
  center: optional bool — return to the resting pose (pan 90, tilt 113). Overrides pan and tilt.
  smooth: optional bool (default true) — ease toward the target rather than snap.
---

# head-move

Publishes a single `chatter/head/cmd` to the ChatterBot Pi and waits for the
head to report it has `arrived` before returning the settled pose.

This is aiming a camera on a stationary head, not locomotion and not
expression. There are no gestures: the head does not nod, shake, or sweep,
and it never moves on its own. A spoken turn carries a note of roughly
where the speaker is and the pan that faces them; use it here if you want
to look at them.

To see what the head is now pointing at, follow this with `camera-capture`;
the camera rides the head, so capture after the move settles.

## Examples

```json
{"thought": "someone is standing; tilt up to see their face", "tool": "head-move", "tilt": 90}
```

```json
{"thought": "look toward the doorway on the left", "tool": "head-move", "pan": 150, "tilt": 105}
```

```json
{"thought": "done looking around; rest the head", "tool": "head-move", "center": true}
```
