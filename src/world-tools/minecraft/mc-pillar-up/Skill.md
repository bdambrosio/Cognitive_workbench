---
name: mc-pillar-up
type: python
description: "Atomic pillar-up: jump and place block under self to ascend by 1 block. Handles timing internally."
schema_hint:
  value: "block name to place (e.g., 'cobblestone', 'dirt')"
  item: "alternative to value"
  out: "$variable"
examples:
  - '{"type":"mc-pillar-up","value":"cobblestone","out":"$pillar"}'
  - '{"type":"mc-pillar-up","item":"dirt","out":"$up"}'
---

# Minecraft Pillar Up Tool

Atomic pillar-up operation: jump and place block under self to ascend by 1 block. Handles the timing-critical jump+place sequence internally.

## Purpose

Vertical ascent by self-scaffolding. Use when you need to go straight up and there's no existing terrain to climb onto. This tool handles the precise timing required to place a block under yourself while airborne.

## Input

- `value`: Block name to place (preferred) - e.g., "cobblestone", "dirt", "stone"
- `item`: Block name to place (alternative to value)

## Output

Returns uniform_return format with:
- `value`: Text summary of result
- `data`: Structured data dict. Key fields:
  - `success`: Boolean
  - `delta_y`: Vertical displacement achieved (1 on success)
  - `from_y`: Starting Y block coordinate
  - `to_y`: Ending Y block coordinate
  - `placed_at`: `{x, y, z}` where block was placed
  - `item`: Block name used

## Behavior & Performance

1. **Grid alignment**: Ensures agent is at block center before jump
2. **Jump**: Initiates pure vertical jump (no forward movement)
3. **Peak timing**: Waits for jump peak (~350ms)
4. **Place**: Places block at original feet level while airborne
5. **Land**: Agent lands on the new block
6. **Verify**: Confirms agent is 1 block higher

Total operation time: ~800-900ms

## Guidelines

**When to use mc-pillar-up:**
- Need to ascend vertically with no existing elevated terrain
- Building a pillar/tower upward
- Escaping a pit or ravine

**When NOT to use mc-pillar-up:**
- There's already an elevated block to climb onto (use `nav-climb` instead)
- You want to bridge forward (use `mc-place` with forward anchor)
- You're already at the edge of a cliff (use `nav-climb` or `mc-place` bridge pattern)

**Comparison with alternatives:**
- `nav-climb`: Climbs onto EXISTING elevated blocks (doesn't place blocks)
- `mc-place` (bridge pattern): Places forward then walk onto (slower but works at edges)
- `mc-pillar-up`: Places directly under self (faster for pure vertical ascent)

## Failure Modes

- `missing_item`: No block name provided
- `status_failed`: Could not get agent position
- `no_headroom`: Block detected at y+2 or y+3 (insufficient clearance for jump)
- `jump_failed`: Jump command failed
- `placement_failed`: Block placement failed while airborne (inventory empty, invalid block, etc.)
- `ascent_failed`: Placement accepted but agent didn't end up higher (block may have been destroyed, agent slipped off, etc.)

## Placement Mechanics

The tool calculates placement internally:
- **Anchor**: Block at `(agent_x, agent_y - 1, agent_z)` (block below feet)
- **Face**: `"top"` (place on top of anchor)
- **Result**: Block at `(agent_x, agent_y, agent_z)` (original feet level)

Since the agent is airborne during placement, the agent's current feet level is temporarily higher than the target, so there's no overlap conflict.

## Usage Examples

**Basic pillar up:**
```json
{"type":"mc-pillar-up","value":"cobblestone","out":"$up"}
```

**Multiple pillar steps (ascend 3 blocks):**
```json
{"type":"mc-pillar-up","value":"dirt","out":"$p1"}
{"type":"mc-pillar-up","value":"dirt","out":"$p2"}
{"type":"mc-pillar-up","value":"dirt","out":"$p3"}
```

**Escape a 2-deep pit:**
```json
{"type":"mc-pillar-up","value":"cobblestone","out":"$escape1"}
{"type":"mc-pillar-up","value":"cobblestone","out":"$escape2"}
```

## Inventory Requirements

- Block must be in inventory
- Uses one block per pillar-up
- Check inventory with `mc-inventory` if unsure

## Headroom Requirements

Pillar-up requires **3 blocks clear above the floor** (or 1 block above current head):
- **y+1**: Agent's current head level (already occupied by agent)
- **y+2**: Required for landing position (agent's head after pillar-up)
- **y+3**: Required for jump peak (head reaches ~y+3 during jump)

The tool checks headroom using the local voxel grid before jumping. If blocks are detected at y+2 or y+3, it fails with `no_headroom` reason. If cells are unknown (not yet observed), it proceeds with a warning.

## Safety Notes

- If headroom check fails, observe the area first or find a different ascent path
- After pillar-up, agent is on a 1x1 pillar - be careful with movement
- If placement fails mid-jump, agent falls back to original position
