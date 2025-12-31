---
name: mc-status
type: python
description: "Fast heartbeat + sanity check for Minecraft bot. Returns connection status, position, orientation, health, food (hunger), and current action."
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
  - `text`: formatted status information including:
    - Connection status
    - Position (x, y, z)
    - Orientation (yaw, pitch)
    - Health: current/max (e.g., "20/20")
    - Food: hunger level (e.g., "20/20")
    - Current action
  - `metadata`: raw status data including:
    - `connected`: boolean - whether bot is connected to server
    - `position`: dict with x, y, z coordinates
    - `orientation`: dict with yaw, pitch (radians)
    - `health`: number - current health (0-20)
    - `food`: number - current food/hunger level (0-20)
    - `action`: dict with type and note fields

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
- Critical for survival mode: monitor health and food (hunger) levels
- Food level affects health regeneration and starvation damage

