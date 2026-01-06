---
name: mc-open
type: python
description: "Open block-based UI (crafting table, chest, furnace). Returns success/failure"
---

# Minecraft Open Tool

Opens block-based UI interfaces like crafting tables, chests, and furnaces. Returns success/failure status.

## Purpose

UI interaction for crafting, storage access, and furnace operations. Opens interactive interfaces from specific block types.

## Input

- Egocentric position (preferred): `forward`, `right`, `up` (floats)
- Absolute position: `x`, `y`, `z` (floats, all required if using absolute)
- Relative position: `rel_x`, `rel_y`, `rel_z` (floats, all required if using relative)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success/failure message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `ui_type`: String (e.g., `"crafting_table"`, `"chest"`, `"furnace"`)

## Behavior & Performance

- Opens UI interface from block at specified position
- Fails if block does not have openable UI
- UI remains open until closed with `mc-close`

## Guidelines

- Use `mc-observe-blocks` to identify openable blocks
- Common openable blocks: crafting_table, chest, furnace, dispenser
- Must be within reach of block
- Use `mc-close` to close opened UI

## Usage Examples

Open block forward:
```json
{"type":"mc-open","forward":1,"up":0,"right":0,"out":"$open"}
```
