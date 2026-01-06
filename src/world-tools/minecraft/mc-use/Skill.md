---
name: mc-use
type: python
description: "Right-click style interaction using equipped item. Returns success/failure"
---

# Minecraft Use Tool

Right-click style interaction using equipped item. Activates blocks, uses tools, or interacts with world objects.

## Purpose

Block activation and tool use. Uses the currently equipped item in hand to interact with blocks or entities at specified position.

## Input

- Egocentric position (preferred): `forward`, `right`, `up` (floats)
- Absolute position: `x`, `y`, `z` (floats, all required if using absolute)
- Relative position: `rel_x`, `rel_y`, `rel_z` (floats, all required if using relative)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success/failure message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Uses equipped item in hand slot
- Fails if no item equipped or item cannot be used at target location
- Position must be within reach

## Guidelines

- Use `mc-equip` first to equip appropriate item
- Use `mc-inventory` to check available items
- Egocentric coordinates are preferred for relative positioning
- Absolute coordinates provide precise global positioning

## Usage Examples

Use equipped item forward:
```json
{"type":"mc-use","forward":1,"up":0,"right":0,"out":"$use"}
```

Use at absolute coordinates:
```json
{"type":"mc-use","x":100,"y":64,"z":200,"out":"$activate"}
```
