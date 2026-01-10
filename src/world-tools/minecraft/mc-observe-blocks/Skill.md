---
name: mc-observe-blocks
type: python
description: "Visible non-air blocks within radius R (default/max 7). Filtered by forward cone (yaw ±60°, pitch -60..+90) and line-of-sight. Does NOT report items."
---

# Minecraft Observe Blocks Tool

Enumerates all visible non-air blocks within observation radius. Provides structured summary with pose, directional distances, footing, clearance, geometry hints, and movement affordances.

## Purpose
Visible block observation for spatial awareness and movement planning. Returns both human-readable SUMMARY text and machine-readable structured data.

## Input
- `radius`: Optional integer observation radius (default: `7`, max: `7`)
- `blocks_radius`: Optional integer observation radius (default: `7`, max: `7`). If both provided, `blocks_radius` takes precedence.
- `value`: Ignored

## Output
Returns uniform_return format with:
- `value`: Multi-line SUMMARY text (human-readable, ≤ 1024 chars)
- `data`: Structured observation dict (machine-readable). Key fields:
  - `success`: boolean
  - `pose`: `{x, y, z, yaw, pitch}` (floats)
  - `dirs`: Per-direction info (`fwd`, `back`, `left`, `right`, `up`, `down`). Each has `dist` (float | null) and `blk` (string | null)
  - `support`: `{here: {type: "solid"|"unsafe", block, depth}, fwd: {type: "solid"|"unsafe", block, depth, forward_block}}`
  - `clear`: `{fwd: {body: bool, head: bool}, up: {body: bool, head: bool}}`
  - `blocks`: `{seen: [string], fluid: [string], hazard: [string], nearby: [dict]}` (nearby is exhaustive raw list)
    - Each entry in `blocks.nearby` includes: `name`, `position`, `dx`, `dy`, `dz`, `surface` (true|false|\"unknown\")
  - `geom`: `{pit: bool, stair: bool, slope: bool}`
  - `aff`: `{step: bool, jump: bool, descend: bool, sky: bool}`
  - `conf`: `"high"` | `"med"` | `"low"`
  - `note`: string (human-readable summary)

## Behavior & Performance
- Returns visible non-air blocks within cone/LOS (capped list)
- View cone: Only reports blocks within a 120° horizontal cone (yaw ±60°) and vertical range up 60° / down 90° (relative to pitch 0)
- Line-of-sight: Only blocks visible from bot eye position (occlusion-aware)
- Radius limits: Default 7 blocks, maximum 7 blocks
- Confidence levels: `high` (complete, fast), `med` (complete but slow/many blocks), `low` (incomplete observation)

## Guidelines
- Use `mc-observe-items` for dropped item entities (this tool only reports blocks)
- `support.here` and `support.fwd` indicate safe footing for movement planning
- `aff.step`, `aff.jump`, `aff.descend` provide movement affordances
- `conf=low` indicates incomplete observation - may need retry or larger radius
- Raw exhaustive data stored in `data.blocks.nearby` for detailed analysis

## Usage Examples

Standard observation:
```json
{"type":"mc-observe-blocks","out":"$obs"}
```

With custom radius:
```json
{"type":"mc-observe-blocks","radius":5,"out":"$obs"}
```
