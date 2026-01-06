---
name: nav-backtrack
type: python
description: "Attempt to return to a recently known safe navigation state using bounded local motion."
---

# Navigation Backtrack Tool

Attempt to return to a recently known safe navigation state using bounded local motion.

## Purpose

Recovery tool that searches `world_state("nav")` history for a safe prior cell and attempts to reach it using atomic nav primitives.

## Input

- `max_targets`: Integer maximum number of safe targets to try (default: `3`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary
- `data`: Structured dict. Key fields:
  - `success`: Boolean
  - `failure_reason`: `"NO_HISTORY"` | `"STATUS_FAILED"` | `"NO_SAFE_TARGET"` | `"UNREACHABLE"` (if `success=false`)
  - `recovered_to`: `{x, y, z, yaw}` (if `success=true`)
  - `method`: `"move1"` | `"climb"` | `"descend"` (if `success=true`)
  - `yaw`: Float (if `success=true`)

## Behavior & Performance

- Reads `world_state("nav")` and chooses up to `max_targets` safe entries.
- Uses `nav-turn` + one of: `nav-moveone`, `nav-climb`, `nav-descend`.
- Verifies cell match via `mc-status`.

## Guidelines

- Requires navigation history to be maintained by nav tools.
- If recovery fails, you may need stabilization (`mc-place-until-supported`) or broader replanning.

## Usage Examples

Attempt recovery:
```json
{"type":"nav-backtrack","max_targets":3,"out":"$recovery"}
```

