---
name: mc-attack
type: python
description: "Left-click style interaction (entities or blocks). Returns success/failure"
---

# Minecraft Attack Tool

Left-click style interaction for attacking entities or breaking blocks. Returns success/failure status.

## Purpose

Combat and block breaking. Can target entities (mobs, players) or blocks depending on target specification.

## Input

- `target`: Dict with either:
  - `entity_id`: String (for entity attack)
  - `dx`, `dy`, `dz`: Floats (for block attack, world-relative offsets from agent)
    - See coordinate system documentation in jill-minecraft.yaml for details
- Exactly one targeting mode must be specified
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success/failure message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Entity attack: Targets specific entity by ID
- Block attack: Targets block at specified position
- Requires appropriate tool for efficient block breaking

## Guidelines

- Use entity_id for attacking mobs or players
- Use world-relative coordinates `dx, dy, dz` for block attacks (see coordinate system in jill-minecraft.yaml)
- Tool efficiency matters for block breaking (use appropriate tool)
- Multiple attacks may be needed to break blocks or defeat entities

## Usage Examples

Attack block directly south:
```json
{"type":"mc-attack","target":{"dx":0,"dy":0,"dz":1},"out":"$attack"}
```

Attack entity:
```json
{"type":"mc-attack","target":{"entity_id":"zombie_123"},"out":"$attack"}
```
