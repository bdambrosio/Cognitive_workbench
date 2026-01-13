# Blocks vs Entities – Critical Distinction

Minecraft distinguishes between BLOCKS and ENTITIES.
Many agent errors come from conflating the two.

---

## Blocks

- Occupy fixed grid positions
- Observed by `mc-observe` (via `blocks.*` fields)
- Can be dug or replaced
- Examples: dirt, stone, sand

---

## Entities

- Have continuous positions
- Move via physics
- Not part of the block grid
- Examples:
  - item entities (drops)
  - mobs
  - falling sand

---

## Item Entities

- Created when blocks drop items
- Must be collected via proximity
- NOT visible to block observation
- NOT directly manipulable

---

## Tool Implications

- `mc-observe` → blocks + entities (including item entities)
- `mc-inventory` → inventory only (slots)
- Pickup is implicit, not commanded

This separation is intentional and mirrors vanilla Minecraft.
