---
name: mc-staircase
description: Automates escaping a deep pit by digging and jumping to build a spiral staircase.
type: instruction
resumable: yes
---

# Minecraft Staircase tool

#PURPOSE:
Escape a vertically constrained pit by incrementally constructing an upward staircase.
This skill applies only while the agent is trapped in a deep pit and should terminate
as soon as that condition no longer holds.

#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- The block at the agent’s current position (forward:0, up:-1) is solid.
- Jumping in place without terrain modification does NOT increase Y.
- Sky is NOT visible directly above within 2 blocks.
- Terrain modification (digging) is permitted.

If these preconditions do not hold, this skill should not be used.

#DESIGN CONSTRAINTS (INTENT, NOT ENFORCED)

- The agent should avoid actions that remove its own floor.
- The block used as a step at (forward:1, up:0) should never be dug.
- Forward movement should not intentionally move the agent onto air.
- Digging an already-air block is acceptable and should be treated as success.
- Tool failures do not, by themselves, imply incorrect geometry.

Violations of these constraints may cause failure and replanning.

#RUNTIME STATE (LOCAL TO THIS SKILL)

- reposition_used_this_cycle : boolean (initially false)
  Used to limit repeated lateral repositioning.

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

---
STEP 1 — ALIGN
---

- Turn to the nearest cardinal direction:
  yaw ∈ {0°, 90°, 180°, -90°}
- Reset reposition_used_this_cycle := false

---
STEP 2 — FIND WALL
---

- Call mc-observe-blocks.
- If forward distance < 1.0:
      A wall occupies forward:1 → proceed to STEP 3
- Else:
      Rotate 90° clockwise and repeat STEP 2
- If all four cardinal directions have been checked and none show
  forward distance < 1.0:
      Proceed to STEP 9 (SAFE REPOSITION)

---
STEP 3 — VERIFY STEP EXISTS
---

- Interpret the blocking wall as the voxel at (forward:1, up:0).
- If the block at (forward:1, up:0) is solid:
      Treat it as the step block and proceed to STEP 4
- If the block is Air or missing:
      Rotate 90° clockwise and return to STEP 2

---
STEP 4 — CLEAR BODY AND HEAD SPACE
---

Ensure a 2-block-high opening exists ABOVE the step block:

- Target blocks:
    - (forward:1, up:1)  ← body space
    - (forward:1, up:2)  ← head space

For each target block:
- If solid, issue mc-dig.
- If already Air, do nothing.
- If dig fails but observation suggests clearance, proceed anyway.

---
STEP 5 — CLIMB
---

- Execute mc-move with:
      forward: true
      jump: true

---
STEP 6 — VERIFY ASCENT
---

- Observe agent position (mc-status or mc-observe-blocks).
- If Y increased by approximately one block:
      Ascent succeeded → proceed to STEP 7
- If Y did NOT increase:
      Treat as climb failure.
      Rotate 90° clockwise and return to STEP 2.

---
STEP 7 — COLLECT / SETTLE
---

- Execute mc-wait for 1.0 seconds.

---
STEP 8 — LOOP
---

- Return to STEP 1.

---
STEP 9 — SAFE REPOSITION (FALLBACK)
---

This step is used only when no valid step is found in any direction.

- If reposition_used_this_cycle is true:
      Return to STEP 1.

- Verify (by observation) that:
      - The block at (forward:0, up:-1) is solid.
      - The block at (forward:1, up:-1) is solid.
      - The block at (forward:1, up:0) is AIR.
      - The block at (forward:1, up:1) is AIR.

- If any appear unsafe:
      Return to STEP 1.

- Otherwise:
      - Execute mc-move with:
            forward: true
            jump: false
      - Execute mc-wait for 0.5 seconds.
      - Set reposition_used_this_cycle := true.
      - Return to STEP 1.

#TERMINATION CONDITIONS

SUCCESS:
- Sky is visible above AND
- Either:
    - Jumping without terrain modification increases Y, OR
    - Stepping forward without digging increases Y.

INAPPLICABLE (EXIT SHALLOW PIT / OPEN TERRAIN):
- Forward distance ≥ 1.0 in all four cardinal directions AND
- Jumping without terrain modification increases Y.

FAILURE:
- STEP 4 detects a solid block at up:2 that cannot be dug (bedrock or protected region)

On termination, control should return to the caller for selection of
a more appropriate skill.


#POSTCONDITIONS

On SUCCESS:
- Agent Y is greater than at skill entry.

On INAPPLICABLE:
- Agent position may have changed laterally.
- No assumption is made about further vertical progress.
