---
name: mc-status
type: python
description: "Fast heartbeat + sanity check for Minecraft bot. Returns connection status, position, orientation, health/food, and current action."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-status","out":"$status"}'
---

# Minecraft Status Tool

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted status information
  - `metadata`: raw status data including:
    - `connected`: boolean - whether bot is connected to server
    - `position`: dict with x, y, z coordinates
    - `orientation`: dict with yaw, pitch (radians)
    - `health`: dict with current, max, food values
    - `current_action`: string - current action being performed (or null if idle)

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-status","out":"$status"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-move","forward":true,"duration":2.0,"out":"$move_result"}
{"type":"mc-status","out":"$status_after"}
```

## Cognitive Contract
- Jill uses this to confirm the world exists and she is embodied
- Detects "something is happening" via current_action
- Fast, non-blocking check for connection and basic state

