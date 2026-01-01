---
name: mc-status
type: python
description: "Fast heartbeat + sanity check for Minecraft bot. Returns connection status, position, orientation, health, food (hunger), current action, and vertical_state (derived from Y position changes)."
schema_hint:
  value: "ignored | 'double_sample'"
  out: "$variable"
examples:
  - '{"type":"mc-status","out":"$status"}'
  - '{"type":"mc-status","value":"double_sample","out":"$status"}'
---

# Minecraft Status Tool

## Input
- `value` parameter:
  - `"double_sample"` (optional): Perform two status samples with ~0.1s delay to compute vertical_state immediately. Use when you need vertical_state without a prior mc-status call.
  - Otherwise ignored (default: single sample, uses prior call if available)

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted status information including:
    - Connection status
    - Position (x, y, z)
    - Orientation (yaw, pitch)
    - Health: current/max (e.g., "20/20")
    - Food: hunger level (e.g., "20/20")
    - Vertical State: STABLE | FALLING | UNKNOWN (with explanation)
    - Current action
  - `metadata`: raw status data including:
    - `connected`: boolean - whether bot is connected to server
    - `position`: dict with x, y, z coordinates
    - `orientation`: dict with yaw, pitch (radians)
    - `health`: number - current health (0-20)
    - `food`: number - current food/hunger level (0-20)
    - `vertical_state`: string - "STABLE" | "FALLING" | "UNKNOWN"
    - `vertical_state_explanation`: string - evidence/reasoning for the state
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

## Vertical State Derivation

The `vertical_state` field is a derived predicate computed from Y position changes across status samples. Since the minescript backend does not provide `onGround`, this inference fills that gap.

### States

- **STABLE**: Y unchanged across ≥1 tick
  - Evidence: `|dy| ≤ 0.02` over the sample interval
  - Indicates: Bot is on solid ground or stationary in air (e.g., on ladder)

- **FALLING**: Y decreasing
  - Evidence: `dy < -0.02` over the sample interval
  - Indicates: Bot is falling due to gravity

- **UNKNOWN**: Insufficient data
  - First sample (no prior Y to compare)
  - Missing Y position data
  - Y increasing (unusual, may indicate teleport/ascending)

### Usage

- **Single sample mode** (default): Requires a prior `mc-status` call to compute `vertical_state`. First call returns `UNKNOWN`.
- **Double sample mode**: Pass `value: "double_sample"` to perform two samples internally (~0.1s delay) and compute `vertical_state` immediately.

### Example Workflow

```json
{"type":"mc-status","out":"$status1"}
{"type":"mc-status","out":"$status2"}
```

Or for immediate vertical_state:

```json
{"type":"mc-status","value":"double_sample","out":"$status"}
```

## Cognitive Contract
- Jill uses this to confirm the world exists and she is embodied
- Detects "something is happening" via current_action
- Fast, non-blocking check for connection and basic state
- Critical for survival mode: monitor health and food (hunger) levels
- Food level affects health regeneration and starvation damage
- Vertical state helps detect falling (critical for survival) and confirms stable positioning

