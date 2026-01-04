---
name: mc-approach-wall
type: python
description: "Bring the agent into stable adjacency with a solid vertical surface suitable for climbing, carving, alignment, or wall-dependent actions."
schema_hint:
  value: "ignored"
  max_steps: "integer maximum steps for each advance attempt (default: 10)"
  step_duration: "float seconds per step for advance (default: 0.2)"
  placement_item: "item/block name for stabilization (default: dirt)"
  out: "$variable"
examples:
  - '{"type":"mc-approach-wall","max_steps":10,"step_duration":0.2,"placement_item":"dirt","out":"$approach"}'
  - '{"type":"mc-approach-wall","max_steps":5,"out":"$approach"}'
---

# Minecraft Approach-Wall Tool

## Skill: APPROACH_WALL

### Level
Level 3 — Situational skill

### Purpose
Bring the agent into stable adjacency with a solid vertical surface suitable for climbing, carving, alignment, or wall-dependent actions.

### Intent
Transform an open or semi-open local configuration into a wall-adjacent, stably supported configuration without assuming deterministic movement.

### Assumed Preconditions
- Agent has stable footing at start (support classified solid)
- Horizontal movement is possible (not in free fall or deep vertical shaft)
- Level-2 stabilization routines are available

### Success Condition
- A solid block is directly forward at foot or eye level
- Agent remains stably supported
- Orientation is fixed relative to the wall (forward-facing)

### Input
- `value`: ignored
- `max_steps`: integer maximum steps for each advance attempt (default: `10`)
- `step_duration`: float seconds per step for advance (default: `0.2`)
- `placement_item`: item/block name for stabilization (default: `"dirt"`)

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: truncated text summary with outcome
  - `data`: structured data dict containing:
    - `outcome`: One of `"SUCCESS"`, `"NO_WALL_REACHABLE"`, `"SUPPORT_UNSTABILIZABLE"`, `"FALL_DETECTED"`
    - `max_steps`: Maximum steps per advance attempt
    - `step_duration`: Duration per step (seconds)
    - `placement_item`: Item used for stabilization
    - `log`: List of log messages for each step
  - `resource_id`: None (no resource created)

### Primitives / Subskills Used
- `mc-observe-blocks`: Observation and wall detection
- `mc-advance`: ADVANCE_GUARDED skill (safe incremental movement)
- `mc-place-until-supported`: MC-PLACE-UNTIL-SUPPORTED skill (stabilization)

### Failure Modes (Legible)

- **NO_WALL_REACHABLE**: No wall encountered within bounded safe advance
- **SUPPORT_UNSTABILIZABLE**: Safe footing cannot be maintained while advancing
- **FALL_DETECTED**: Fall occurred (unexpected, catastrophic)

### Procedure

1. **INITIAL SETUP**
   - Fix current heading (no free yaw changes during main loop)
   - Rationale: diagonal approaches introduce brittle collision behavior

2. **MAIN LOOP**
   LOOP until success or failure:

   2.1 **OBSERVE**
       - Call `mc-observe-blocks`
       - If a solid block is detected directly forward (foot or eye level):
             → TERMINATE with SUCCESS

   2.2 **ADVANCE SAFELY**
       - Call `mc-advance`(direction=forward, max_steps=N)

       **INTERPRET RESULT:**

       a) **BLOCKED**
          Interpretation: A blocking surface exists at current heading.
          Action:
            - Treat as potential wall adjacency
            - Re-run OBSERVE once
            - If still no wall detected → NO_WALL_REACHABLE

       b) **FELL**
          → TERMINATE with FALL_DETECTED

       c) **SUPPORT_AMBIGUOUS**
          Interpretation: Forward progress degraded footing before wall contact.
          Action:
            - Call `mc-place-until-supported`
            - If stabilization fails → SUPPORT_UNSTABILIZABLE
            - Else → continue loop

       d) **BOUND_REACHED**
          Interpretation: Safe advance completed without encountering wall.
          Action:
            - Re-run OBSERVE
            - If no wall detected → NO_WALL_REACHABLE
            - Else → SUCCESS

   END LOOP

### Invariants

- Never retry raw movement after ambiguity without stabilization
- Never intentionally accept increased fall risk
- All retries must change either pose interpretation or world state

### Notes

- **BLOCKED is not treated as failure**; it is evidence that must be reinterpreted.
- This skill does not search or explore; it operates only within a bounded, locally reachable region.
- All stochasticity from movement is absorbed by `mc-advance` and `mc-place-until-supported`.
- The tool fixes heading at the start to avoid diagonal approaches that introduce brittle collision behavior.
- Wall detection checks for solid blocks directly forward at foot or eye level.

### Common Workflow

```json
{"type":"mc-status","out":"$status"}
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-equip","item":"dirt","slot":"hand","out":"$equip"}
{"type":"mc-approach-wall","max_steps":10,"step_duration":0.2,"placement_item":"dirt","out":"$approach"}
{"type":"mc-observe-blocks","out":"$obs_after"}
```

### Cognitive Contract

- **SITUATIONAL**: Operates within a bounded, locally reachable region
- **STABILIZATION-AWARE**: Uses stabilization routines when support becomes ambiguous
- **EVIDENCE-BASED**: Re-interprets BLOCKED outcomes through observation
- **HEADING-FIXED**: Maintains consistent orientation to avoid brittle collision behavior
- **OUTCOME-BASED**: Returns specific outcome codes for higher-level decision making

