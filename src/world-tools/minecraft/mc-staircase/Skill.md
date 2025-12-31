---
name: mc-staircase
description: Constructs a spiral staircase to ascend from any depth (small or deep pits). Handles digging and placing automatically to reach the surface safely.
type: method
resumable: yes
invalidates: [position, orientation, visibility, groundedness, inventory]
examples:
  - '{"type":"mc-staircase","out":"$result"}'
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
- **A solid wall is within interaction range (forward distance < 1.0) in at least one cardinal direction.**

If these preconditions do not hold, this skill should not be used.

#DESIGN CONSTRAINTS (INTENT, NOT ENFORCED)

- The agent should avoid actions that remove its own floor.
- The block used as a step at (forward:1, up:0) should never be dug.
- Forward movement should not intentionally move the agent onto air.
- Digging an already-air block is acceptable and should be treated as success.
- Tool failures do not, by themselves, imply incorrect geometry.

Violations of these constraints may cause failure and replanning.

#LIMITATIONS:
- To loop within a method, use "Return to STEP X" (internal loop within this method execution), not recursive method calls
- If this method needs another method's functionality (e.g. mc-seek-boundary), it can be executed directly

#RUNTIME STATE (LOCAL TO THIS SKILL)

- reposition_used_this_cycle : boolean (initially false)
  Used to limit repeated lateral repositioning.

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

At start of each iteration, check termination conditions:
- Call mc-observe-blocks.
- If sky is visible above AND (jumping increases Y OR forward movement increases Y):
      **TERMINATE (SUCCESS)**
- If forward distance ≥ 1.0 in all four cardinal directions AND jumping increases Y:
      **TERMINATE (INAPPLICABLE)**

---
STEP 1 — FIND WALL AND ALIGN
---

- Execute mc-seek-boundary.
  This navigates to the nearest wall and ensures cardinal alignment (yaw ∈ {0°, 90°, 180°, -90°}) while maintaining wall proximity.

---
STEP 2 — VERIFY WALL PROXIMITY
---

- Call mc-observe-blocks to confirm forward distance < 1.0.
- If forward distance < 1.0:
      Wall confirmed → proceed to STEP 3
- Else:
      Wall not found → return to STEP 1

---
STEP 3 — VERIFY OR BUILD STEP
---

- Call mc-observe-blocks to check block state at (forward:1, up:0).
- If forward distance < 1.0 AND observation indicates solid block at (forward:1, up:0):
      Treat it as the step block and proceed to STEP 4
- If forward distance ≥ 1.0 OR observation indicates Air/Liquid at (forward:1, up:0):
      **CONSTRUCT STEP**: 
      1. Call mc-inventory to check for placeable blocks (dirt, cobblestone, stone, etc.).
      2. If placeable blocks available:
            a. Call mc-equip to equip a placeable block.
            b. Call mc-place with item=<block_name>, forward=1, up=0, right=0, face="north" to build the step.
      3. If placement succeeds, proceed to STEP 4.
      4. If placement fails or no blocks available:
            Call mc-status to get current yaw.
            Calculate new yaw = current_yaw + 90° (convert to radians for mc-look).
            Call mc-look with new yaw to rotate 90° clockwise, then return to STEP 2

---
STEP 4 — CLEAR BODY AND HEAD SPACE
---

Ensure clearance exists in two locations. Clear a 3-block-wide strip (left, center, right) at each Y level to handle slight misalignment:

1. Directly overhead (for jump initiation):
   - (forward:0, right:-1, up:1)  ← left body space overhead
   - (forward:0, up:1)  ← center body space overhead
   - (forward:0, right:1, up:1)  ← right body space overhead
   - (forward:0, right:-1, up:2)  ← left head space overhead
   - (forward:0, up:2)  ← center head space overhead
   - (forward:0, right:1, up:2)  ← right head space overhead

2. At destination (for forward movement):
   - (forward:1, right:-1, up:1)  ← left body space at step
   - (forward:1, up:1)  ← center body space at step
   - (forward:1, right:1, up:1)  ← right body space at step
   - (forward:1, right:-1, up:2)  ← left head space at step
   - (forward:1, up:2)  ← center head space at step
   - (forward:1, right:1, up:2)  ← right head space at step

For each target block:
- If solid, issue mc-dig.
- If already Air, do nothing.
- If dig fails but observation suggests clearance, proceed anyway.

---
STEP 4.5 — VERIFY CLEARANCE
---

- Call mc-observe-blocks to verify critical path is clear.
- Check that blocks at (forward:1, up:1) and (forward:1, up:2) are AIR.
- If both are AIR:
      Clearance confirmed → proceed to STEP 5
- If either block is still solid:
      Retry mc-dig on the remaining solid block(s) (max 2 retries per block).
      After retries, call mc-observe-blocks again.
      If still not clear:
          **Abort climb**: Call mc-status to get current yaw, calculate new yaw = current_yaw + 90° (convert to radians), call mc-look to rotate 90° clockwise, then return to STEP 2
      If now clear:
          Proceed to STEP 5

---
STEP 5 — CLIMB
---

- Call mc-status to record Y position before climb.
- Execute mc-move with:
      forward: true, jump: true, duration: 0.5

---
STEP 6 — VERIFY ASCENT
---

- Call mc-status to get current Y position.
- Compare to Y position recorded at start of STEP 5.
- If Y increased by ≥ 0.8 blocks:
      Ascent succeeded → proceed to STEP 7
- If Y did NOT increase by ≥ 0.8 blocks:
      Treat as climb failure.
      Call mc-status to get current yaw, calculate new yaw = current_yaw + 90° (convert to radians), call mc-look to rotate 90° clockwise, then return to STEP 2.

---
STEP 7 — COLLECT / SETTLE
---

- Execute mc-wait for 1.0 seconds.

---
STEP 8 — LOOP
---

- Loop back to STEP 1 (within this method execution).


#TERMINATION CONDITIONS

SUCCESS:
- Sky is visible above AND
- Either:
    - Jumping without terrain modification increases Y, OR
    - Stepping forward without digging increases Y.

INAPPLICABLE (EXIT SHALLOW PIT / OPEN TERRAIN):
- Forward distance ≥ 1.0 in all four cardinal directions AND
- Jumping without terrain modification maintains or increases Y.

FAILURE:
- STEP 4 detects a solid block at up:2 that cannot be dug (bedrock or protected region)
- STEP 2 cannot find a wall within 1.0 distance in any cardinal direction.

On termination, control should return to the caller for selection of
a more appropriate skill.


#POSTCONDITIONS

On SUCCESS:
- Agent Y is greater than at skill entry.

On INAPPLICABLE:
- Agent position may have changed laterally.
- No assumption is made about further vertical progress.
