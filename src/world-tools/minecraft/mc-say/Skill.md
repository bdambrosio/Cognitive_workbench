---
name: mc-say
type: python
description: "Send chat message in Minecraft - social/debugging/alignment. Returns acknowledgement only."
schema_hint:
  value: "message text"
  message: "message text (alternative to value)"
  out: "$variable"
examples:
  - '{"type":"mc-say","value":"Hello!","out":"$ack"}'
  - '{"type":"mc-say","message":"Debug message","out":"$ack"}'
---

# Minecraft Say Tool

## Input
- `value`: message text to send (preferred)
- `message`: message text to send (alternative to value)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: acknowledgement message
  - `metadata`: raw response data

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-say","value":"Starting exploration","out":"$ack"}
{"type":"mc-observe-blocks","out":"$obs"}
```

## Cognitive Contract
- Returns acknowledgement only - no need for replies
- Jill never blocks waiting for chat
- Used for social interaction, debugging, and alignment

