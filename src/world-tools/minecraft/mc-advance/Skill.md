---
name: mc-advance
type: python
description: "Advance incrementally in a chosen direction while maintaining conservative safety guarantees. Movement stops as soon as forward progress is blocked, a fall occurs, or footing becomes ambiguous."
schema_hint:
  value: "ignored"
  direction: "one of {forward, back, left, right} (default: forward)"
  step_duration: "float seconds per step (default: 0.2)"
  max_steps: "integer maximum steps (default: None, unbounded)"
  out: "$variable"
examples:
  - '{"type":"mc-advance","direction":"forward","step_duration":0.2,"max_steps":5,"out":"$advance"}'
  - '{"type":"mc-advance","direction":"forward","out":"$advance"}'
---

# Minecraft Advance Tool

## Skill: ADVANCE

### Purpose
Advance the agent incrementally in a chosen direction (typically forward) while maintaining conservative safety guarantees. Movement stops as soon as forward progress is blocked, a fall occurs, or footing becomes ambiguous.

### Primitives Used
- `mc-move`: Attempted locomotion with bounded duration
- `mc-observe-blocks`: Support and clearance assessment
- `mc-status`: Optional sanity/pose check (not currently used but available)

### Input
- `value`: ignored
- `direction`: one of `{"forward", "back", "left", "right"}` (default: `"forward"`)
- `step_duration`: float seconds per step (default: `0.2`)
- `max_steps`: integer maximum steps (default: `None`, unbounded)

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: truncated text summary with outcome and step count
  - `data`: structured data dict containing:
    - `outcome`: One of `"ADVANCED"`, `"BLOCKED"`, `"FELL"`, `"SUPPORT_AMBIGUOUS"`, `"LIMIT_REACHED"`
    - `steps_completed`: Number of steps successfully completed
    - `direction`: Direction used for movement
    - `step_duration`: Duration per step (seconds)
    - `max_steps`: Maximum steps limit (if set)
    - `log`: List of log messages for each step
  - `resource_id`: None (no resource created)

### Termination Outcomes

- BLOCKED: Forward collision detected by `mc-move` (`data.status == "collision"`)
- FELL: Fall detected by `mc-move` (`data.status == "fell"`)
- SUPPORT_AMBIGUOUS: Movement would risk a fall if continued (support not provably solid)
- BOUND_REACHED: successful completion to max_steps

### Invariants

- Never intentionally step into unsupported space
- Terminate on credible risk, not on realized failure

### Notes

- This tool makes no promise of reaching a target or completing a distance.
- Success is defined only as observed incremental advancement.
- Absence of blockers does not imply safety; ambiguity triggers termination.
- Higher-level skills must interpret outcomes and decide escalation.
- For directions other than "forward", support checking is limited to current position ("here") since `mc-observe-blocks` only provides "here" and "fwd" support information.

### Common Workflow

```json
{"type":"mc-observe-blocks","out":"$obs"}
{"type":"mc-advance","direction":"forward","step_duration":0.2,"max_steps":10,"out":"$advance"}
{"type":"mc-status","out":"$status"}
```

### Cognitive Contract

- CONSERVATIVE: Terminates on any credible risk, not just realized failures
- INCREMENTAL: Moves one step at a time, checking safety after each step
- GUARDED: Uses support assessment to prevent falls before they occur
- OUTCOME-BASED: Returns specific outcome codes for higher-level decision making

