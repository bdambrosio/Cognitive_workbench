---
name: mc-place-until-supported
type: python
description: "Create reliable footing by placing blocks until observed support becomes provably solid. Absorbs placement uncertainty and never exposes 'accepted but ineffective' outcomes."
schema_hint:
  value: "item/block name"
  item: "item/block name (alternative to value)"
  placement_policy: "one of {underfoot, forward-underfoot, lateral} (default: underfoot)"
  max_attempts: "integer maximum placement attempts (default: 3)"
  verify_delay: "float seconds to wait before re-observation (default: 0.0)"
  out: "$variable"
examples:
  - '{"type":"mc-place-until-supported","value":"dirt","placement_policy":"underfoot","max_attempts":3,"out":"$place"}'
  - '{"type":"mc-place-until-supported","item":"cobblestone","placement_policy":"forward-underfoot","out":"$place"}'
---

# Minecraft Place-Until-Supported Tool

## Skill: MC-PLACE-UNTIL-SUPPORTED

### Level
Level 2 — Stabilization routine

### Purpose
Create reliable footing at or near the agent's position by placing blocks until observed support becomes provably solid, or until placement is no longer possible. This skill absorbs placement uncertainty and never exposes "accepted but ineffective" outcomes to higher layers.

### Preconditions
- Agent is stationary or safely anchored (not falling)
- At least one placeable block is available (inventory/equip handled externally)
- A reference location and face for placement can be defined

### Postconditions (on Success)
- Observed support at the agent's footing is classified as solid
- No immediate fall risk remains at the current pose

### Input
- `value`: Item/block name to place (optional, preferred if provided)
- `item`: Item/block name to place (optional, alternative to value)
- `placement_policy`: one of `{"underfoot", "forward-underfoot", "lateral"}` (default: `"underfoot"`)
- `max_attempts`: integer maximum placement attempts (default: `3`)
- `verify_delay`: float seconds to wait before re-observation (default: `0.0`)

**Auto-selection**: If `value` and `item` are not provided, the tool automatically selects the most abundant solid block from inventory. The selection considers common solid blocks that can provide support (stone, cobblestone, dirt, planks, gravel, netherrack, and many others). If no solid blocks are found in inventory, the tool returns `INVENTORY_EXHAUSTED`.

### Output
- Uniform return format dict (bound to `out` variable if specified):
  - `status`: 'success' or 'failed' (execution status)
  - `value`: truncated text summary with outcome and attempt count
  - `data`: structured data dict containing:
    - `outcome`: One of `"SUPPORTED"`, `"NO_PLACEABLE_TARGET"`, `"PLACEMENT_IMPOSSIBLE"`, `"INVENTORY_EXHAUSTED"`, `"SUPPORT_AMBIGUOUS"`
    - `attempts_made`: Number of placement attempts made
    - `max_attempts`: Maximum attempts limit
    - `placement_policy`: Policy used for placement
    - `item`: Item/block name used
    - `verify_delay`: Delay used before verification
    - `log`: List of log messages for each attempt
  - `resource_id`: None (no resource created)

### Placement Policies

- underfoo*: Place block directly underfoot at (forward:0, up:-1) with face="up"
- forward-underfoot: Place block forward and underfoot at (forward:1, up:-1) with face="up"
- lateral: Place block laterally forward at (forward:1, up:0) with face="north" (toward agent)

### Termination Outcomes

- SUPPORTED: Solid footing observed after placement
- NO_PLACEABLE_TARGET: No valid placement reference found
- PLACEMENT_IMPOSSIBLE: Placement accepted but support never achieved (max_attempts reached or placement failed)
- INVENTORY_EXHAUSTED: No blocks available in inventory
- SUPPORT_AMBIGUOUS: Observation failed or inconclusive after bounded attempts

### Invariants

- Never worsen support (no action increases fall depth)
- Terminate on bounded attempts; no infinite retries
- Never assume placement success without observation

### Failure Modes (Legible)

- NO_PLACEABLE_TARGET: No valid placement location available
- PLACEMENT_IMPOSSIBLE: Repeated placement attempts accepted but never yield support
- INVENTORY_EXHAUSTED: No placeable blocks remain
- SUPPORT_AMBIGUOUS: Observation inconclusive after bounded attempts

### Notes

- This skill hides all asynchronous placement uncertainty.
- Higher-level skills must not call `mc-place` directly when support matters.
- This skill makes no promise about optimality, only local stability.
- Placement is asynchronous; the tool waits and verifies actual support through observation.
- The `verify_delay` parameter allows time for asynchronous placement to complete before observation.

### Common Workflow

```json
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"dirt","slot":"hand","out":"$equip"}
{"type":"mc-status","out":"$status"}
{"type":"mc-place-until-supported","item":"dirt","placement_policy":"underfoot","max_attempts":3,"out":"$place"}
{"type":"mc-observe-blocks","out":"$obs"}
```

### Cognitive Contract

- STABILIZATION: Focuses on achieving local stability, not optimal placement
- VERIFICATION-BASED: Never assumes placement succeeded without observation
- BOUNDE*: Respects max_attempts limit; no infinite retries
- UNCERTAINTY-ABSORBING: Hides asynchronous placement uncertainty from higher layers
- OUTCOME-BASED: Returns specific outcome codes for higher-level decision making

