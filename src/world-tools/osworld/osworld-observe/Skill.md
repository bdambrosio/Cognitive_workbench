---
name: osworld-observe
type: python
description: "Get the current observation from the OSWorld environment. Returns screenshot (base64 PNG) and accessibility tree (JSON)."
schema_hint:
  value: "ignored"
  include_screenshot: "bool (default: true)"
  include_a11y: "bool (default: true)"
  out: "$variable"
examples:
  - '{"type":"osworld-observe","out":"$obs"}'
  - '{"type":"osworld-observe","include_screenshot":false,"out":"$a11y_only"}'
---

# OSWorld Observe Tool (Level 4)

## Input
- `include_screenshot`: bool (default: true) - include screenshot in observation
- `include_a11y`: bool (default: true) - include accessibility tree in observation
- `value` parameter is ignored

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted observation summary
  - `format`: "json"
  - `metadata`: observation data including:
    - `timestamp`: observation timestamp
    - `step_counter`: current step counter
    - `observation.screenshot`: dict with `encoding` ("png") and `data_base64` (base64-encoded PNG)
    - `observation.accessibility_tree`: raw accessibility tree JSON

## Configuration
- `OSWORLD_URL` environment variable (defaults to `http://localhost:3002`)
- Or pass `osworld_url` in character config's `osworld_config` section

## Common Workflow
```json
{"type":"osworld-observe","out":"$obs"}
{"type":"osworld-execute","python":"pyautogui.click(100,200)","out":"$result"}
{"type":"osworld-observe","out":"$obs2"}
```

## Notes
- Screenshot is returned as base64-encoded PNG data
- Accessibility tree is raw JSON from OSWorld
- No interpretation or filtering is performed - raw observation data only

