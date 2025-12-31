---
name: mc-pillar-jump
description: Vertically ascends one or more levels by placing blocks beneath the agent. Best for open terrain or when walls are distant. Requires overhead clearance and placeable blocks.
type: method
resumable: yes
invalidates: [position, visibility, groundedness, inventory]
examples:
  - '{"type":"mc-pillar-jump","out":"$result"}'
---

# Minecraft Pillar Jump tool

#PURPOSE:
To gain elevation quickly in open terrain where staircasing is inefficient or walls are distant.
Primarily used for escaping deep pits, reaching high ledges, or gaining a vantage point.

#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- The agent possesses at least one stack of solid, placeable blocks (dirt, cobblestone, stone, etc.).
- The space directly above the agent (up:2) is AIR (no low ceilings).
- The agent is grounded.

#DESIGN CONSTRAINTS

- The agent must use inexpensive blocks (dirt/cobble) and preserve valuable resources.
- The agent should stop before running out of blocks completely (leave a safety reserve).
- The method must verify that a block was actually placed to effectively counteract gravity.

#RUNTIME STATE (LOCAL TO THIS SKILL)

- start_y : float (recorded at entry)
- blocks_placed : integer (initially 0)

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

---
STEP 1 — PREPARE
---

- Call mc-inventory.
- Identify a slot containing expendable solid blocks.
- Execute mc-equip with the identified block item name.
- If no blocks are available:
      TERMINATE (FAILURE - NO MATERIALS)

---
STEP 2 — ALIGN
---

- Execute mc-look with:
      pitch: 90.0 (Looking straight down)
      yaw: (Current yaw - maintain orientation)

---
STEP 3 — CHECK CLEARANCE
---

- Execute mc-observe-blocks.
- Check the block at (forward:0, up:2).
- If solid:
      TERMINATE (FAILURE - OBSTRUCTED)
      (Prevents suffocating or hitting head against a ceiling)

---
STEP 4 — JUMP AND PLACE
---

- Execute sequence:
      1. mc-move (jump: true, duration: 0.1s) to initiate upward momentum.
      2. mc-place (item: <block_name_from_step1>, forward: 0, right: 0, up: -1, face: up) immediately after jump start.
      3. mc-wait (0.5s) to allow physics to settle on the new block.

---
STEP 5 — VERIFY ASCENT
---

- Execute mc-status or mc-observe-blocks.
- Compare current Y with previous Y.
- If Y has increased by approx 1.0:
      Increment blocks_placed.
      Proceed to STEP 6.
- If Y has NOT increased:
      (Lag or placement failure)
      Retry STEP 4 up to 3 times, then TERMINATE (FAILURE - PHYSICS SYNC ERROR).

---
STEP 6 — LOOP
---

- Loop back to STEP 1 (Check inventory and continue).

#TERMINATION CONDITIONS

SUCCESS:
- Target elevation reached (if specified in goal).
- Visibility condition met (if specified).

FAILURE:
- Inventory empty (Ran out of blocks).
- Ceiling obstruction detected.
- Repeated failure to ascend (Physics/Lag issues).

#POSTCONDITIONS

On SUCCESS:
- Agent is standing on a precarious 1x1 pillar.
- Movement must be handled carefully to avoid falling (likely requires 'mc-move' with shift/sneak or a specific descent method).
