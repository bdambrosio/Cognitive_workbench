---
name: mc-seek-boundary
description: Navigates the agent from an open area to the nearest solid wall or obstruction.
type: method
resumable: yes
invalidates: [position, orientation]
---

# Minecraft Seek Boundary tool

#PURPOSE:
To autonomously navigate the agent from open space to the nearest vertical surface.
This serves as a setup step for skills that require wall proximity (like mc-staircase).

#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- The agent is currently in an open area where forward distance > 1.0.
- The agent is grounded (not falling or swimming).
- A solid block is visible at eye-level within the observation radius.

#DESIGN CONSTRAINTS

- The agent should move towards the nearest available wall to minimize travel time.
- The agent should avoid walking into lava, deep water, or off cliffs during the approach.
- If multiple walls are equidistant, any solid surface is acceptable.

#RUNTIME STATE (LOCAL TO THIS SKILL)

- steps_taken : integer (initially 0)
  Used to prevent infinite wandering if a wall cannot be reached.

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

---
STEP 1 — SCAN
---

- Execute mc-observe-blocks.
- Check immediate proximity:
  If forward distance < 1.0:
      Wall reached → TERMINATE (SUCCESS)

- Analyze observation to find the nearest solid block at eye level (up:0 or up:1).
- If no solid blocks are visible in any direction:
      Proceed to TERMINATE (FAILURE - NO WALLS)

---
STEP 2 — ORIENT
---

- Calculate the Yaw required to face the nearest identified solid block.
- Execute mc-look with the calculated Yaw.

---
STEP 3 — APPROACH
---

- Execute mc-move with:
      forward: true
      jump: true (to handle small terrain roughness)
      duration: 1.0 seconds (short bursts to allow re-evaluation)

- Increment steps_taken by 1.

---
STEP 4 — VERIFY PROGRESS
---

- If steps_taken > 20:
      Proceed to TERMINATE (FAILURE - TIMEOUT)

- Execute mc-observe-blocks.
- If forward distance < 1.0:
      Wall reached → proceed to STEP 5 (CARDINAL ALIGN)
- Else:
      Loop back to STEP 2 (Re-orient and continue approach)

---
STEP 5 — CARDINAL ALIGN
---

- Execute mc-observe-blocks to check all four cardinal directions.
- Test each cardinal direction (yaw ∈ {0°, 90°, 180°, -90°}):
      For each direction, check if forward distance < 1.0 (wall at forward:1).
- Select a cardinal direction that has forward distance < 1.0.
      If multiple cardinal directions have walls, prefer the one closest to current yaw.
- Execute mc-look to align to the selected cardinal direction.
- Verify: Call mc-observe-blocks. Confirm forward distance < 1.0.
      If not, try the next cardinal direction with a wall.
- Once aligned to a cardinal direction with forward distance < 1.0:
      Proceed to TERMINATE (SUCCESS)

#TERMINATION CONDITIONS

SUCCESS:
- Forward distance is < 1.0 (Agent is touching or nearly touching a wall).
- Agent is aligned to a cardinal direction (yaw ∈ {0°, 90°, 180°, -90°}).

FAILURE:
- No solid blocks visible (Agent is on a floating island or in a void).
- steps_taken exceeds limit (Agent is stuck or target is unreachable).
- Path involves unavoidable safety hazards (lava/void).
- No cardinal direction has forward distance < 1.0 (wall not accessible from cardinal alignment).

#POSTCONDITIONS

On SUCCESS:
- Agent is facing a solid block within interaction range.
- Agent is aligned to a cardinal direction (yaw ∈ {0°, 90°, 180°, -90°}).