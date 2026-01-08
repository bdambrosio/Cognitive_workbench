---
name: nav-backtrack
type: python
description: "Attempt to return to a recently known safe navigation state using bounded local motion."
---

# nav-backtrack

Recovery tool that searches `world_state("nav")` history for a safe prior cell and attempts to reach it using atomic nav primitives.

## Input

- `max_targets`: Integer maximum number of safe targets to try (default: `3`)

## Output

Success (`status: "success"`):
- `value`: Summary text
- `extra.recovered_to`: `{x, y, z, yaw}` position recovered to
- `extra.method`: `"step"` | `"climb"` | `"descend"`
- `extra.yaw`: Final yaw

Failure (`status: "failed"`):
- `reason`: One of `"NO_HISTORY"`, `"STATUS_FAILED"`, `"NO_SAFE_TARGET"`, `"UNREACHABLE"`

## Behavior

- Reads `world_state("nav")` and chooses up to `max_targets` safe entries
- Uses `nav-turn` + one of: `nav-move`, `nav-climb`, `nav-descend`
- Verifies cell match via `mc-status`

## Planning Notes

- Requires navigation history to be maintained by nav tools
- If recovery fails, consider `mc-place-until-supported` or broader replanning

## Example

```json
{"type":"nav-backtrack","max_targets":3,"out":"$recovery"}
```
