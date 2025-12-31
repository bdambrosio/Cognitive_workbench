---
name: mc-move
type: python
description: "Initiate locomotion - moves for specified duration or until collision. SYNCHRONOUS/BLOCKING. Returns final status."
schema_hint:
  value: "ignored"
  forward: "move forward (bool)"
  back: "move backward (bool)"
  left: "strafe left (bool)"
  right: "strafe right (bool)"
  jump: "jump while moving (bool)"
  sprint: "sprint while moving (bool)"
  duration: "movement duration in seconds (float, bounded)"
  check_collision: "stop on collision (bool, default true)"
  out: "$variable"
examples:
  - '{"type":"mc-move","forward":true,"duration":2.0,"out":"$move"}'
  - '{"type":"mc-move","forward":true,"sprint":true,"duration":5.0,"check_collision":true,"out":"$run"}'
---

# Minecraft Move Tool

## Input
- `value`: ignored
- `forward`: bool - move forward
- `back`: bool - move backward
- `left`: bool - strafe left
- `right`: bool - strafe right
- `jump`: bool - jump while moving
- `sprint`: bool - sprint while moving
- `duration`: float - movement duration in seconds (bounded)
- `check_collision`: bool - whether to stop early if blocked (default: true)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: result summary (success, collision, or fell)
  - `metadata`: raw response including:
    - `status`: "success", "collision", or "fell"
    - `actual_duration_ms`: how long the move actually lasted
    - `final_position`: bot position after move
    - `reason`: failure reason if collision or fall occurred

## Configuration
- Configured via `world_config.port` (defaults to `http://localhost:3003`)
- Can be overridden with `world_config.url` or `MINECRAFT_URL` environment variable

## Common Workflow
```json
{"type":"mc-move","forward":true,"duration":2.0,"out":"$move"}
{"type":"mc-status","out":"$status"}
{"type":"mc-observe-blocks","out":"$obs"}
```

## Cognitive Contract
- SYNCHRONOUS: This tool blocks until movement is complete or a collision occurs.
- RETURNS REALITY: The output tells you what actually happened (e.g., "Collision with Stone").
- Use `check_collision: true` (default) to prevent getting stuck against walls.
- Ideal for "manual" exploration (move forward 2s, look, move forward 2s).

