---
name: nav-move
type: python
description: "Attempt exactly ONE adjacent navigation move forward and report the outcome. Atomic navigation operation suitable for composition."
---

# Navigation Move Tool

Attempt exactly one adjacent forward move (relative to current facing direction) and report the outcome.

## Purpose

Atomic forward navigation primitive. Intended to be composed into higher-level navigation skills (advance, approach-wall, backtrack).

## Input

- `step_duration`: Float seconds for the move (default: `0.2`)
- `max_abs_delta_y`: Float tolerance for Y noise (default: `0.2`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary
- `data`: Structured dict. Key fields:
  - `success`: Boolean
  - `from`: `{x, y, z}` (floats)
  - `to`: `{x, y, z}` (floats) or null
  - `delta_y`: Float (if landed)
  - `support_here`: `"solid"` | `"unsafe"` | other (if observed)
  - `fell`: Boolean
  - `failure_reason`: `"collision"` | `"fell"` | `"support_ambiguous"` | `"observation_failed"` | `"move_failed"` | `"unexpected_vertical_change"`

## Behavior & Performance

- Calls bridge `/act/move` once (forward) with collision checking.
- Validates landing via `mc-status` + `mc-observe-blocks`.
- Snaps to block center after any position change.
- Updates `world_state("nav")` history.

## Guidelines

- Use for single-step movement with verification.
- For multi-step forward motion, prefer `nav-advance`.
- If `fell` occurs, treat as safety event and consider `nav-backtrack`.

## Usage Examples

Move forward one cell:
```json
{"type":"nav-move","out":"$m1"}
```

Use a slower move:
```json
{"type":"nav-move","step_duration":0.25,"out":"$m1"}
```
