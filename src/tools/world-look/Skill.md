---
name: world-look
description: Look around the shared world — the ground you are standing on, and who else is present, how far away they are and whether they are facing you. Your primary perception when you are embodied in the world.
args:
  radius: optional number (default 30, max 120) — metres of surrounding terrain to summarise
---

# world-look

Your situational read of the shared world you, Bruce, and any other
characters occupy. Returns where you are standing, what the ground is
like there, and every other occupant with their distance, compass
bearing, whether they are moving, and whether they are looking at you.

You are one body among several. Bruce walks around under his own control
and can be anywhere; the only way to know where he is now is to look.

## When to use

- Before moving, so you know what you are moving toward.
- When Bruce mentions where he is, or asks what you can see.
- After a `world-move` finishes, to confirm where you ended up — a move
  stops early if the straight path is blocked.

## Looking is not moving

This tool only tells you what is there. If the request was to go
somewhere, to join someone, or to follow — call `world-move` in the same
turn, after this. Do not reply "walking over now" or "on my way" unless
you have actually posted the goal: your body does not move because you
said it would, and the person waiting will watch you stand still.

## Notes

- Coordinates are metres, `(x, z)`, with the world centred on `(0, 0)`.
- "in your view" means roughly within your forward arc, not that anything
  is unobstructed — trees are not occlusion-tested.
- If the world is not running the call reports that plainly; it is not an
  error you should retry.

## Example

```json
{"thought": "check where Bruce is before I walk", "tool": "world-look"}
```

```json
{"thought": "get a wider read of the terrain", "tool": "world-look", "radius": 80}
```
