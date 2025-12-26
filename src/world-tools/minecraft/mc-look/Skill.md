---
name: mc-look
type: python
description: "Reorient bot's view - adjust yaw and pitch in radians."
schema_hint:
  value: "ignored"
  yaw: "yaw angle in radians (float, required)"
  pitch: "pitch angle in radians (float, required)"
  out: "$variable"
examples:
  - '{"type":"mc-look","yaw":1.57,"pitch":0.0,"out":"$look"}'
  - '{"type":"mc-look","yaw":0.0,"pitch":-0.5,"out":"$look_down"}'
---

# Minecraft Look Tool

## Input
- `value`: ignored
- `yaw`: float - yaw angle in radians (required)
- `pitch`: float - pitch angle in radians (required)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: acknowledgement message
  - `metadata`: raw response data

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-look","yaw":1.57,"pitch":0.0,"out":"$look"}
{"type":"mc-observe-blocks","out":"$obs"}
```

## Cognitive Contract
- Reorients perception - changes what bot sees
- Returns ok - immediate acknowledgement
- Used to scan environment, look at specific blocks/entities

