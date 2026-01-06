---
name: nav-advance
type: python
description: "Advance forward up to N blocks (target) using nav conventions. Stops early on collision, fall, or ambiguous support."
---

# Navigation Advance Tool

Advance forward by a target number of blocks while maintaining nav safety and snap-to-grid conventions.

## Purpose

Planner-friendly multi-step forward motion. Reduces fragile planner loops while preserving nav invariants (verification, snap-to-grid, nav history).

## Input

- `target`: Integer number of blocks to advance (required)
- `step_duration`: Float seconds per move (default: `0.2`)
- `max_abs_delta_y`: Float tolerance for Y noise (default: `0.2`)
- `value`: Ignored (alternative: if `target` is omitted, `value` may be used)

## Output

Returns uniform_return format with:
- `value`: Text summary with steps completed and stop reason
- `data`: Structured dict. Key fields:
  - `success`: Boolean (true only if all target blocks were completed safely)
  - `steps_completed`: Integer
  - `target`: Integer
  - `stop_reason`: `"TARGET_REACHED"` | `"COLLISION"` | `"FELL"` | `"SUPPORT_AMBIGUOUS"` | `"MOVE_FAILED"` | `"STATUS_FAILED"` | `"OBSERVATION_FAILED"` | `"UNEXPECTED_VERTICAL_CHANGE"`

## Behavior & Performance

- Internally performs up to `target` single moves with verification.
- Snaps to block center after each landed move and updates nav history.
- Stops immediately when unsafe conditions are detected.

## Guidelines

- Use instead of planner loops over `nav-moveone`.
- If `stop_reason` is `FELL`, treat as safety event and consider `nav-backtrack`.

## Usage Examples

Advance 3 blocks:
```json
{"type":"nav-advance","target":3,"out":"$adv"}
```

