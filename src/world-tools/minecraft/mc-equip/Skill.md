---
name: mc-equip
type: python
description: "Equip an item from inventory into hand or offhand. Returns success/failure"
---

# Minecraft Equip Tool

Equips an item from inventory into hand or offhand slot. Returns success/failure status.

## Purpose

Item equipping for tool use, combat, and interaction. Moves items from inventory slots to hand or offhand for use.

## Input

- `value`: Item name to equip (preferred)
- `item`: Item name to equip (alternative to value)
- `slot`: `"hand"` | `"offhand"` (default: `"hand"`)

## Output

Returns uniform_return format with:
- `value`: Text summary (e.g., "Equipped stone in hand")
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `equipped`: String (item that was equipped)
  - `slot`: String (`"hand"` | `"offhand"`)

## Behavior & Performance

- Requires item to be in inventory
- Fails if item not found in inventory
- Hand slot is default if slot not specified

## Guidelines

- Use `mc-inventory` first to check available items
- Item name must match exactly (e.g., "stone", "wooden_pickaxe")
- Hand slot is primary interaction slot
- Offhand slot is secondary slot (e.g., shield, torch)

## Usage Examples

Equip item in hand:
```json
{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"}
```

Equip tool:
```json
{"type":"mc-equip","value":"wooden_pickaxe","slot":"hand","out":"$tool"}
```
