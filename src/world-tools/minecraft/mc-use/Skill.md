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

- Position: `dx`, `dy`, `dz` (world-relative offsets from agent, floats, all required)
  - See coordinate system documentation in jill-minecraft.yaml for details
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
- All positions use world-relative coordinates `dx, dy, dz` (see coordinate system in jill-minecraft.yaml)

## Usage Examples

Use equipped item at block directly east:
```json
{"type":"mc-use","dx":1,"dy":0,"dz":0,"out":"$use"}
```

Use at block directly south:
```json
{"type":"mc-use","dx":0,"dy":0,"dz":1,"out":"$activate"}
```
