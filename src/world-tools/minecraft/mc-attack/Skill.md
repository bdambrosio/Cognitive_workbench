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
  - `forward`, `right`, `up`: Floats (for block attack, egocentric)
  - `rel_x`, `rel_y`, `rel_z`: Floats (for block attack, legacy)
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
- Use egocentric coordinates (forward, right, up) for block attacks
- Tool efficiency matters for block breaking (use appropriate tool)
- Multiple attacks may be needed to break blocks or defeat entities

## Usage Examples

Attack block forward:
```json
{"type":"mc-attack","target":{"forward":1,"right":0,"up":0},"out":"$attack"}
```

Attack entity:
```json
{"type":"mc-attack","target":{"entity_id":"zombie_123"},"out":"$attack"}
```
