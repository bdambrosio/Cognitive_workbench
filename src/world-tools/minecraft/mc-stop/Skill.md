---
name: mc-stop
type: python
description: "Stop all movement - cancel embodied motion. Critical for safety, interrupts, reflection pauses."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-stop","out":"$stop"}'
---

# Minecraft Stop Tool

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: acknowledgement message
  - `metadata`: raw response data

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-move","forward":true,"duration":10.0,"out":"$move"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-stop","out":"$stop"}
```

## Cognitive Contract
- Critical for safety - stops all movement immediately
- Used for interrupts - when Jill needs to change plans
- Used for reflection pauses - when Jill needs to think
- Returns ok - immediate acknowledgement

