---
name: fac-observe
description: Look around in Factorio — entities near a position (or near you), plus production telemetry and alerts. Your primary perception in the factory; observe before building or when an action reports a deviation.
args:
  x: optional number — center of the observation; omit (with y) to observe around your character
  y: optional number — see x
  radius: optional number (default 15, max 50) — tiles around the center
---

# fac-observe

The situational read: entities in range, production flows, alerts.
Composes the bridge's /status, /observe and /telemetry into one call.

## Behavior

- Lists entities within `radius` of the center — name, position, facing,
  status, warnings, contents. Same-name swarms (belt runs, poles) are
  summarized with a count and extent.
- Observing also updates the bridge's observation cache, which is what
  deviation classification compares against — observe first, act second.
- Appends production telemetry (per-item flows over the last minute) and
  any recent alerts.
- Resource patches in range are included as summary lines — e.g.
  "iron-ore patch (~608 tiles, ~436829 units) @ (-39, -22)" (position is
  the patch centroid). Use fac-nearest when no patch is in range.
- Water and cliffs are terrain, not entities: an empty listing does not
  guarantee buildable ground.

```json
{"thought": "what is around the furnace area?", "tool": "fac-observe", "args": {"x": 48, "y": 62, "radius": 10}}
```
