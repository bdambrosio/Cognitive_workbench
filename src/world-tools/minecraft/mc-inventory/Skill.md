---
name: mc-inventory
type: python
description: "Observe inventory contents and equipped items - epistemic, read-only. Returns inventory slots and equipped items"
situational: true
---

# Minecraft Inventory Tool

Observes inventory contents and equipped items. Read-only epistemic tool for checking current inventory state.

## Purpose

Inventory inspection for planning and decision-making. Returns both human-readable inventory text and machine-readable structured data including all slots and equipped items.

## Input

- `value`: Ignored

## Output

Returns uniform_return format with:
- `value`: Multi-line inventory text (human-readable)
- `data`: Structured inventory dict (machine-readable). Key fields:
  - `success`: Boolean
  - `slots`: Array of `{slot: int, item: string, count: int}`
  - `equipped`: `{hand: string, offhand: string}`

## Behavior & Performance

- Read-only: Does not modify inventory
- Fast status check
- Returns complete inventory state including empty slots

## Guidelines

- Use before planning item operations (equip, drop, craft)
- Check `equipped.hand` and `equipped.offhand` for currently held items
- Empty slots have `item: null` or empty string

## Usage Examples

Check inventory:
```json
{"type":"mc-inventory","out":"$inv"}
```