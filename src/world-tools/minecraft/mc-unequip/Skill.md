---
name: mc-unequip
type: python
description: "Clear or swap equipped item. Returns success/failure"
---

# Minecraft Unequip Tool

Clears or swaps equipped item from hand or offhand slot. Returns success/failure status.

## Purpose

Item unequipping for inventory management and tool switching. Moves items from hand/offhand back to inventory.

## Input

- `slot`: `"hand"` | `"offhand"` (default: `"hand"`)
- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Text summary (success/failure message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean

## Behavior & Performance

- Clears specified slot
- Item returns to inventory
- Fails if slot already empty

## Guidelines

- Use before equipping different items
- Hand slot is default if slot not specified
- Empty slots cannot be unequipped

## Usage Examples

Unequip hand slot:
```json
{"type":"mc-unequip","slot":"hand","out":"$unequip"}
```
