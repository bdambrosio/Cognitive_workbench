---
name: osworld-execute
type: python
description: "Execute Python code (typically pyautogui commands) in the OSWorld environment. Returns execution result with success status, return code, duration, stdout, and stderr."
schema_hint:
  value: "string (Python code)"
  python: "string (Python code, alternative to value)"
  return_observation: "bool (default: false)"
  out: "$variable"
examples:
  - '{"type":"osworld-execute","python":"pyautogui.click(100,200)","out":"$result"}'
  - '{"type":"osworld-execute","value":"pyautogui.typewrite(\"hello\")","return_observation":true,"out":"$result"}'
---

# OSWorld Execute Tool (Level 4)

## Input
- `python` or `value`: Python code string (required) - typically pyautogui commands
- `return_observation`: bool (default: false) - include observation in response
- `value` parameter can be used as alternative to `python`

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: formatted execution result
  - `format`: "text"
  - `metadata`: execution data including:
    - `success`: boolean - whether execution succeeded
    - `returncode`: integer - return code (0 = success)
    - `duration_ms`: integer - execution duration in milliseconds
    - `step_counter`: integer - step counter after execution
    - `stdout`: string - standard output
    - `stderr`: string - standard error
    - `python_code`: string - the executed code
    - `observation`: dict (if return_observation=true) - observation after execution
    - `timestamp`: float (if return_observation=true) - observation timestamp

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
- Python code is executed directly in the OSWorld environment
- Common commands: `pyautogui.click(x, y)`, `pyautogui.typewrite(text)`, `pyautogui.press(key)`
- No retries or corrections - Jill owns error handling
- Execution is synchronous and blocking

