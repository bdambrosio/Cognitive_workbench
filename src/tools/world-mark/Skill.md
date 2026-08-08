---
name: world-mark
description: Leave a labelled marker in the shared world, at your feet or at a coordinate. Markers persist and are visible to anyone who can see that spot — a way to point at a place without saying a coordinate out loud.
args:
  label: required string — short name for the spot (e.g. "water here", "meet at this rock")
  x: optional number — place it at this x instead of at your feet (use with z)
  z: optional number — see x
---

# world-mark

Leave something in the world instead of saying something about it.

A marker stays where it is put. That matters across the gap between how
fast Bruce moves and how slowly you take turns: a marker dropped now is
still there when he next looks, and it points at a place without either
of you having to read coordinates aloud.

## When it earns its keep

- Marking somewhere you want to return to, or want him to find.
- Answering "where?" with a place rather than a number.
- Leaving a trail — several marks describe a route.

## What others see

Markers obey the same sight limits as people: one left over a rise is
not visible from the other side. `world-look` reports the ones in view,
with their label, who placed it, and how far away it is. Someone has to
be able to see the spot for the marker to tell them anything.

Bruce can drop markers too, from the world window. If one appears that
you did not place, he put it there.

## Examples

```json
{"thought": "mark where I am so he can find me", "tool": "world-mark", "label": "waiting here"}
```

```json
{"thought": "flag the clearing we discussed", "tool": "world-mark", "label": "the clearing", "x": 40, "z": -12}
```
