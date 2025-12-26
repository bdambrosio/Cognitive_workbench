---
name: mc-pickup
type: instruction
description: "How to collect items from the world into inventory - automatic pickup when close enough."
---

# Minecraft Pickup Instruction

## How Item Collection Works

In Minecraft, items are **automatically collected** into your inventory when you move close enough to them. There is no separate "pickup" action.

## Canonical Sequence

After digging a block or dropping an item:

1. **mc-dig** - Break a block (if you wish to 'pickup' a block rather than an item already in the environment) - (drops appear in world)
2. **mc-move** - Move slightly forward/sideways towards the dropped item (small nudge, ~0.5-1 block)
3. **mc-inventory** - Verify the item appears in inventory

## Important Notes

- **No explicit pickup call needed** - Items automatically enter inventory when within ~1.5 blocks
- **equip is NOT for picking up** - `mc-equip` only holds items already in your inventory. It does not pull items from the world.
- **Movement is required** - You must move close to the dropped item for automatic collection
- **Verify with mc-inventory** - Always check inventory after moving to confirm collection

## Example Workflow

```json
{"type":"mc-dig","forward":1,"up":0,"right":0,"out":"$dig"}
{"type":"mc-move","forward":true,"duration":0.5,"out":"$nudge"}
{"type":"mc-inventory","out":"$inv"}
{"type":"mc-equip","item":"stone","slot":"hand","out":"$equip"} - if you wish to use or place the item
```

Note: `mc-equip` comes AFTER the item is already in inventory (from the automatic pickup).
