# Item Pickup Rules (Java Edition)

Defines how item entities enter inventory.

---

## Pickup Mechanics

- Items are collected automatically when:
  - Player collides with item entity
  - Distance ≈ 1–1.5 blocks
- There is NO explicit "pick up" action in vanilla Minecraft.

---

## Failure Modes

Inventory may remain empty if:
- No item entity was created
- Item fell into an unreachable space
- Player did not intersect item hitbox
- Item despawned (long delay, not typical here)

---

## Agent Guidance

- Do NOT assume pickup after digging
- Movement near the drop location is required
- Collision does not guarantee pickup
- Inventory is the sole source of truth

---

## Design Implication

Lack of item visibility is not a bug —
it is faithful to Minecraft’s mechanics.
