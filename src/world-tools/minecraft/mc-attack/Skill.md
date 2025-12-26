---
name: mc-attack
type: python
description: "Left-click style interaction (entities or blocks). Returns success/failure."
schema_hint:
  value: "ignored"
  target: "dict with entity_id (string) OR forward/right/up (floats) OR rel_x/rel_y/rel_z (floats)"
  forward: "blocks forward (float, egocentric - alternative top-level arg)"
  right: "blocks right (float, egocentric - alternative top-level arg)"
  up: "blocks up (float, egocentric - alternative top-level arg)"
  out: "$variable"
examples:
  - '{"type":"mc-attack","target":{"forward":1,"right":0,"up":0},"out":"$attack"}'
  - '{"type":"mc-attack","target":{"entity_id":"zombie_123"},"out":"$attack"}'
---

# Minecraft Attack Tool

## Input
- `target`: dict with either:
  - `entity_id`: string (for entity attack)
  - `forward`, `right`, `up`: floats (for block attack, egocentric)
  - `rel_x`, `rel_y`, `rel_z`: floats (for block attack, legacy)
- Exactly one targeting mode must be specified

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether attack succeeded

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-attack","target":{"forward":1,"right":0,"up":0},"out":"$attack"}
{"type":"mc-attack","target":{"entity_id":"zombie_123"},"out":"$attack_entity"}
```

## Cognitive Contract
- Left-click style interaction
- Can target entities (by entity_id) or blocks (by position)
- Mutually exclusive targeting modes (entity OR block, not both)

