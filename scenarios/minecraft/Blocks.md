# Minecraft Blocks – Operational Summary (Java Edition)

This document defines block properties relevant to agent decision-making.
It is NOT an encyclopedia. Only properties that affect action outcomes
(digging, dropping, pickup, movement, observation) are included.

Version: Java Edition (1.20+ assumptions unless noted)

---

## Core Concepts

- Blocks occupy integer grid coordinates (x, y, z).
- A block may be solid, partial-height, or non-solid.
- Breaking a block may:
  - drop an item entity
  - drop nothing
  - transform into another block/item
- Dropped items are **item entities**, not blocks.

---

## Common Terrain Blocks (Starter Set)

### Dirt
- id: `dirt`
- solid: yes
- gravity: no
- breakable: yes
- guaranteed_drop: dirt
- notes: Safe default for tests

### Grass Block
- id: `grass_block`
- solid: yes
- gravity: no
- guaranteed_drop: dirt
- notes: Drops dirt, not grass_block

### Stone
- id: `stone`
- solid: yes
- gravity: no
- guaranteed_drop: cobblestone
- notes: Requires no tool for drop, just slower

### Cobblestone
- id: `cobblestone`
- solid: yes
- gravity: no
- guaranteed_drop: cobblestone

### Sand
- id: `sand`
- solid: yes
- gravity: yes
- guaranteed_drop: sand
- notes: Falls when unsupported

### Gravel
- id: `gravel`
- solid: yes
- gravity: yes
- guaranteed_drop: gravel
- notes: Falls when unsupported

---

## Partial / Special Blocks (Important Pitfalls)

### Snow (Layer)
- id: `snow`
- solid: partial
- gravity: no (Java)
- guaranteed_drop: ❌ none
- notes:
  - Snow *layers* do NOT drop items
  - Common test failure case

### Tall Grass
- id: `tall_grass`
- solid: no
- guaranteed_drop: ❌ none
- notes: Decorative, no drop

### Air
- id: `air`
- solid: no
- breakable: no
- notes: Absence of block

---

## Gravity-Affected Blocks

These blocks become entities when unsupported:

- sand
- gravel
- concrete_powder (all colors)
- anvil (all damage states)
- dragon_egg

---

## Drop Semantics (High-Level)

- Breaking a block produces:
  - zero or one **item entities** at or near the block position
- Items must be physically collected (collision-based)
- `mc-observe` reports **blocks only**
- Dropped items are NOT visible to block sampling

---

## Agent Guidance Rules

- For guaranteed pickup tests:
  - Prefer: dirt, stone, cobblestone
  - Avoid: snow, tall grass, decorative blocks
- Never assume a drop without consulting `guaranteed_drops.md`
