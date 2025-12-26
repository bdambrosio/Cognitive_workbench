---
name: mc-close
type: python
description: "Close currently open UI. Returns success/failure."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-close","out":"$close"}'
---

# Minecraft Close Tool

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: success/failure message
  - `metadata`: raw response including:
    - `success`: boolean - whether close succeeded

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-open","rel_x":0,"rel_y":0,"rel_z":1,"out":"$open"}
{"type":"mc-craft","recipe":"minecraft:stick","count":4,"out":"$craft"}
{"type":"mc-close","out":"$close"}
```

## Cognitive Contract
- Closes currently open UI
- Required after mc-craft if UI was opened
- No-op if no UI is open

