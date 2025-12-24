---
name: osworld-reset
type: python
description: "Reset the OSWorld environment. Soft mode clears step counter only. Hard mode calls DesktopEnv.reset() to revert to snapshot state."
schema_hint:
  value: "string ('soft' or 'hard')"
  mode: "string ('soft' or 'hard', alternative to value)"
  out: "$variable"
examples:
  - '{"type":"osworld-reset","mode":"soft","out":"$reset"}'
  - '{"type":"osworld-reset","value":"hard","out":"$reset"}'
---

# OSWorld Reset Tool (Level 4)

## Input
- `mode` or `value`: string (required) - "soft" or "hard"
  - `soft`: Clear step counter only, no state change
  - `hard`: Call DesktopEnv.reset() to revert environment to snapshot state

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted reset result
  - `format`: "text"
  - `metadata`: reset data including:
    - `success`: boolean - whether reset succeeded
    - `mode`: string - reset mode used
    - `step_counter`: integer - step counter after reset (0)

## Configuration
- `OSWORLD_URL` environment variable (defaults to `http://localhost:3002`)
- Or pass `osworld_url` in character config's `osworld_config` section

## Common Workflow
```json
{"type":"osworld-reset","mode":"hard","out":"$reset"}
{"type":"osworld-observe","out":"$obs"}
{"type":"osworld-execute","python":"pyautogui.click(100,200)","out":"$result"}
```

## Notes
- **Soft reset**: Only clears step counter. Useful for reflection without losing state.
- **Hard reset**: Reverts environment to snapshot state. Critical for counterfactual replay and reflection.
- Hard reset is the key reflection enabler - allows Jill to explore "what if" scenarios.

