---
name: osworld-status
type: python
description: "Get the current status of the OSWorld server. Returns server readiness, uptime, step counter, provider, and screen configuration."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"osworld-status","out":"$status"}'
---

# OSWorld Status Tool (Level 4)

## Input
- No parameters required
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted status information
  - `metadata`: raw status data including:
    - `ready`: boolean - whether environment is initialized
    - `uptime_sec`: server uptime in seconds
    - `step_counter`: number of actions executed
    - `provider`: provider name (e.g., "docker")
    - `headless`: boolean - whether running headless
    - `screen`: dict with width/height

## Configuration
- `OSWORLD_URL` environment variable (defaults to `http://localhost:3002`)
- Or pass `osworld_url` in character config's `osworld_config` section

## Common Workflow
```json
{"type":"osworld-status","out":"$status"}
{"type":"osworld-observe","out":"$obs"}
{"type":"osworld-execute","python":"pyautogui.click(100,200)","out":"$result"}
```

