---
name: mc-respawn
type: python
description: "Reset embodiment after death - lifecycle/recovery. Returns ok."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-respawn","out":"$respawn"}'
---

# Minecraft Respawn Tool

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
{"type":"mc-status","out":"$status"}
{"type":"mc-respawn","out":"$respawn"}
{"type":"mc-status","out":"$status_after"}
```

## Cognitive Contract
- Resets embodiment after death
- Jill uses it when health drops unexpectedly
- Jill uses it when embodiment invariants are violated
- Returns ok - immediate acknowledgement

