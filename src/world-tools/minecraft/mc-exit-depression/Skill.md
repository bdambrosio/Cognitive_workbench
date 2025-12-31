---
name : mc-exit-depression
description: Pathfinding utility to escape pits, holes, or uneven terrain. Scans for natural exits via horizontal movement and minimal terrain modification. Does not build structures.
type: method
resummable: yes
invalidates: [position, orientation, visibility, groundedness]
examples:
  - '{"type":"mc-exit-depression","out":"$result"}'
---

#PURPOSE:
Exit a shallow depression or uneven terrain by finding a nearby position from which vertical progress is possible without constructing a staircase.
This skill is intended to follow EXIT_DEEP_PIT_VIA_STAIRCASE when that skill terminates as inapplicable.

#PRECONDITIONS (WHEN THIS SKILL APPLIES)
- The agent is standing on solid ground.
- Jumping in place MAY increase Y, but vertical escape is not yet reliable.
- At least one horizontal direction shows forward distance ≥ 1.0.
- Terrain modification (digging) is permitted but should be minimal.
If these do not hold, this skill should terminate as INAPPLICABLE.

#DESIGN CONSTRAINTS (INTENT, NOT ENFORCED)
- Prefer horizontal movement over digging.
- Prefer stepping up natural terrain over carving steps.
- Avoid digging unless it directly enables stepping up.
- Avoid deepening the depression.
- Do not commit to a single direction for long; explore locally.
Violations may cause failure and replanning.

#LIMITATIONS:
- Method tools cannot invoke other method tools (prevents recursion and complexity)
- To loop within a method, use "Return to STEP X" (internal loop within this method execution), not recursive method calls
- If a method needs another method's functionality, return control to the outer planner which can chain methods

#RUNTIME STATE (LOCAL TO THIS SKILL)
- attempted_directions : set of cardinal directions (initially empty)

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met or exploration is exhausted:

STEP 1 — ORIENT
- Face a cardinal direction not yet in attempted_directions.
- If all four cardinal directions have been attempted:
      Terminate with status: FAILED_TO_EXIT.

---
STEP 2 — ASSESS FORWARD MOVE
- Call mc-observe-blocks.
- If forward distance ≥ 1.0:
      Candidate for movement → proceed to STEP 3.
- Else:
      Mark this direction as attempted and loop back to STEP 1 (within this method execution).

---
STEP 3 — ATTEMPT STEP-UP
- Attempt mc-move with:
      forward: true
      jump: true

---
STEP 4 — EVALUATE RESULT
- Observe agent position.
- If Y increased:
      Progress made → proceed to STEP 5.
- If movement failed (Collision/No Y increase): Proceed to STEP 6 (ATTEMPT CLEARANCE).

---
STEP 5 — CONSOLIDATE POSITION
- Execute mc-wait.
- Clear attempted_directions (local success resets exploration).
- Re-evaluate termination conditions.

---
STEP 6 — CLEAR PATH & RETRY <-- RENAMED & LOGIC FIXED
- Check Headroom:
  - Identify block at (forward:1, up:2).
  - If SOLID or UNKNOWN: Issue mc-dig (forward:1, up:2).
- Check Body:
  - Identify block at (forward:1, up:1).
  -  If SOLID: Issue mc-dig (forward:1, up:1).
- Retry:
  - If ANY digging occurred: Return to STEP 3 (Try Jump Again).
  - If NO digging was possible (blocks are Air or Indestructible): Mark direction as attempted and loop back to STEP 1 (within this method execution).
     
#TERMINATION CONDITIONS

SUCCESS:
- Agent reaches a position where:
    - Jumping in place reliably increases Y, OR
    - Forward movement without digging increases Y, OR
    - Open sky is visible above with no blocking geometry.

INAPPLICABLE:
- No horizontal movement is possible AND
- Jumping does not increase Y.

FAILED_TO_EXIT:
- All four cardinal directions explored without progress.

#POSTCONDITIONS

On SUCCESS:
- Agent is no longer constrained by the shallow depression.

On FAILED_TO_EXIT or INAPPLICABLE:
- Control should return to the caller for selection of another skill
  (e.g., EXIT_DEEP_PIT_VIA_STAIRCASE or general navigation).

