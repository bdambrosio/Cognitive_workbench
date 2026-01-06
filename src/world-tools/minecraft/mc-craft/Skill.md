---
name: mc-craft
type: python
description: "Perform crafting via Mineflayer recipe system. Returns success/failure"
---

# Minecraft Craft Tool

Performs crafting using Mineflayer recipe system. Crafts items from inventory materials according to Minecraft recipes.

## Purpose

Item crafting for tool creation, material processing, and item production. Uses available inventory materials to craft items.

## Input

- `value`: Recipe/item name (preferred)
- `recipe`: Recipe/item name (alternative to value)
- `count`: Integer (optional, default: `1`)

## Output

Returns uniform_return format with:
- `value`: Text summary (success/failure message)
- `data`: Structured data dict (machine-readable). Key fields:
  - `success`: Boolean
  - `item`: String (item that was crafted)
  - `count`: Integer (number of items crafted)

## Behavior & Performance

- Uses Mineflayer recipe system
- Requires appropriate materials in inventory
- Fails if materials insufficient or recipe invalid
- May require crafting table for complex recipes

## Guidelines

- Use `mc-inventory` first to check available materials
- Item name must match Minecraft item names (e.g., "minecraft:stick", "minecraft:wooden_pickaxe")
- Some recipes require crafting table - use `mc-open` on crafting table first
- Count parameter allows crafting multiple items at once

## Usage Examples

Craft single item:
```json
{"type":"mc-craft","recipe":"minecraft:stick","count":4,"out":"$craft"}
```

Craft tool:
```json
{"type":"mc-craft","value":"minecraft:wooden_pickaxe","count":1,"out":"$tool"}
```
