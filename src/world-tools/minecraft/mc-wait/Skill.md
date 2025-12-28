---
name: mc-wait
type: python
description: "Synchronous wait for 1 second using time.sleep. Useful for timing delays or pacing actions."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"mc-wait","out":"$wait"}'
---

# Minecraft Wait Tool

## Input
- `value`: ignored
- `out`: variable name to store result

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: wait completion message
  - `metadata`: raw wait data including:
    - `wait_duration_seconds`: duration waited (currently fixed at 1 second)
    - `status`: completion status ("completed")

## Configuration
- No external configuration required
- Uses Python `time.sleep()` for synchronous blocking wait

## Common Workflow
```json
{"type":"mc-move","forward":true,"duration":2.0,"out":"$move"}
{"type":"mc-wait","out":"$wait"}
{"type":"mc-observe-blocks","out":"$obs"}
```

## Cognitive Contract
- **SYNCHRONOUS**: This tool blocks for exactly 1 second before returning
- **TIMING**: Useful for pacing actions, allowing world state to settle, or creating deliberate delays
- **NO MINECRAFT API**: Does not interact with Minecraft server, purely Python timing
- **CONSTANT DURATION**: Currently fixed at 1 second (no parameters)

