# Minecraft Items – Operational Summary (Java Edition)

This document defines properties of items relevant to inventory,
pickup, and reporting. It excludes crafting and enchantments.

---

## Core Concepts

- Items exist in two forms:
  1. Inventory items (inside slots)
  2. Item entities (dropped in the world)
- Only item entities can be picked up from the world.
- Inventory items are reported via `mc-inventory`.

---

## Common Items (Starter Set)

### Dirt
- id: `dirt`
- stackable: yes (64)
- obtained_from: dirt, grass_block
- placeable: yes

### Cobblestone
- id: `cobblestone`
- stackable: yes (64)
- obtained_from: stone, cobblestone
- placeable: yes

### Sand
- id: `sand`
- stackable: yes (64)
- obtained_from: sand
- placeable: yes
- gravity_when_placed: yes

### Gravel
- id: `gravel`
- stackable: yes (64)
- obtained_from: gravel
- placeable: yes
- gravity_when_placed: yes

---

## Item Entity Rules

- Dropped items are **entities**, not blocks.
- They:
  - have positions
  - obey gravity
  - can slide or bounce
- They are **not blocks** and will not appear in `mc-observe.blocks.*`.
- They **do** appear in `mc-observe.entities.*` when entity scanning is enabled.
- **Despawn**: item entities typically despawn after ~5 minutes (~6000 ticks) if not collected.

---

## Agent Guidance

- Inventory can only change via:
  - pickup of item entities
  - crafting (not in scope here)
- If inventory is empty after digging:
  - Either no item dropped
  - Or item entity was not collected
