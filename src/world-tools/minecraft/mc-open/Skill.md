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

- Position: `dx`, `dy`, `dz` (world-relative offsets from agent, floats, all required)
  - See coordinate system documentation in jill-minecraft.yaml for details
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

- Use `mc-observe` to identify openable blocks
- Common openable blocks: crafting_table, chest, furnace, dispenser
- Must be within reach of block
- Use `mc-close` to close opened UI

## Usage Examples

Open block directly south:
```json
{"type":"mc-open","dx":0,"dy":0,"dz":1,"out":"$open"}
```

Open block directly east:
```json
{"type":"mc-open","dx":1,"dy":0,"dz":0,"out":"$open"}
```
