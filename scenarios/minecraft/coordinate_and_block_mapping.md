# Coordinate System and Block Mapping Rules

Defines how continuous player positions map to discrete block coordinates.

---

## Coordinate Types

- Player position:
  - Continuous (floating point)
  - Example: (-127.06, 71.00, -100.66)

- Block position:
  - Discrete (integer grid)
  - Example: (-127, 70, -100)

---

## Block Occupancy Rule

A block at (x, y, z) occupies the unit cube:
[x, x+1) × [y, y+1) × [z, z+1)

---

## Derived Locations

### Block Beneath Agent

Given agent position (px, py, pz):

- Block beneath =
  - (floor(px), floor(py) - 1, floor(pz))

Example:
- Agent at (-127.06, 71.00, -100.66)
- Block beneath = (-127, 70, -100)

---

## Observation Implications

- The block beneath the agent:
  - May be missing from *visual block lists* (cone/LOS/caps)
  - Is usually available via navigation fields (`support` / `nav_surface` / `adjacent_blocks`) when using `mc-observe`
  - Still exists unless explicitly removed

---

## Agent Guidance

- Always compute target block positions explicitly.
- Do not rely on observation sampling to infer block absence.
- Coordinate math is more reliable than visual sampling.
