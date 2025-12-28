---
name: mc-explore
description: Systematically explores nearby terrain to discover exits, hazards, and affordances.
type: method
resumable: yes
invalidates: [position, orientation, visibility]
---

# Minecraft Explore tool

#PURPOSE:
Incrementally explore nearby terrain to gain situational awareness and discover
navigable paths, exits, hazards, or geometry that warrants a more specialized skill.
This skill prioritizes information gathering over distance traveled.

#PRECONDITIONS (WHEN THIS SKILL APPLIES)

- The agent is not currently executing a higher-priority recovery or escape skill.
- The agent can rotate in place and attempt forward movement.
- No immediate vertical entrapment is detected that would require mc-staircase.
- Terrain modification is either disallowed or unnecessary.

If these preconditions do not hold, this skill should not be used.

#DESIGN CONSTRAINTS (INTENT, NOT ENFORCED)

- Avoid stepping onto blocks with unknown or unsafe support below.
- Prefer solid ground over air when probing movement.
- Prefer unexplored directions over recently attempted ones.
- Avoid digging unless required to confirm safety.
- Exploration should remain local and reversible where possible.

Violations of these constraints may cause failure and replanning.

#LIMITATIONS:
- Method tools cannot invoke other method tools (prevents recursion and complexity)
- To loop within a method, use "Return to STEP X" (internal loop within this method execution), not recursive method calls
- If a method needs another method's functionality, return control to the outer planner which can chain methods

#RUNTIME STATE (LOCAL TO THIS SKILL)

- explored_directions : set of {N, E, S, W} (initially empty)
- last_safe_position : agent position at skill entry
- steps_since_observation : integer (initially 0)

#MAIN PROTOCOL LOOP

Repeat until a termination condition is met.

---
STEP 1 — ALIGN
---

- Snap yaw to the nearest cardinal direction:
  yaw ∈ {0°, 90°, 180°, -90°}

- If current facing direction ∈ explored_directions:
      Rotate 90° clockwise.

- If all four cardinal directions ∈ explored_directions:
      Clear explored_directions.

---
STEP 2 — OBSERVE FORWARD AFFORDANCES
---

- Call mc-observe-blocks.

Evaluate:
- Forward distance
- Block at (forward:1, up:-1)
- Clearance at (forward:1, up:0)
- Clearance at (forward:1, up:1)

Interpretation:
- If forward distance < 1.0:
      A wall blocks movement → mark direction explored and loop back to STEP 1 (within this method execution).
- If block at (forward:1, up:-1) is Air:
      Unsafe footing → mark direction explored and loop back to STEP 1 (within this method execution).
- If body or head space is blocked:
      Mark direction explored and loop back to STEP 1 (within this method execution).
- Otherwise:
      Proceed to STEP 3.

---
STEP 3 — PROBE MOVE
---

- Execute mc-move with:
      forward: true
      jump: false

- Execute mc-wait for 0.5 seconds.

---
STEP 4 — VERIFY SAFETY
---

- Observe agent position.

- If Y decreased unexpectedly OR footing appears unsafe:
      Attempt to recover if possible.
      Mark direction explored.
      Loop back to STEP 1 (within this method execution).

- Otherwise:
      steps_since_observation += 1
      Proceed to STEP 5.

---
STEP 5 — PERIODIC SCAN
---

If steps_since_observation ≥ 2:

- Call mc-observe-blocks.
- Reset steps_since_observation := 0.

Check for:
- Sky visible above.
- Sustained upward movement without digging.
- Clear downward shafts or hazardous drops.
- Open terrain suggesting navigation rather than exploration.
- Geometry matching another skill’s preconditions.

If any are detected:
      Proceed to TERMINATION (DISCOVERY).

Otherwise:
      Proceed to STEP 6.

---
STEP 6 — MARK PROGRESS
---

- Add current facing direction to explored_directions.
- Loop back to STEP 1 (within this method execution).

#TERMINATION CONDITIONS

DISCOVERY:
- Sky becomes visible.
- The agent can gain height without terrain modification.
- A clear exit, slope, or stair-like geometry is observed.
- A hazard (drop, pit, lava, void) is detected.
- Preconditions for another skill (e.g., mc-staircase) now hold.

STAGNATION:
- All four directions have been explored repeatedly AND
- The agent remains within approximately 2 blocks of last_safe_position.

FAILURE:
- The agent falls unexpectedly.
- The agent becomes immobilized.
- Observation repeatedly fails to return usable data.

On termination, control should return to the caller for
selection of a more appropriate skill.

#POSTCONDITIONS

On DISCOVERY:
- Agent position may differ from entry position.
- New terrain affordances have been observed.

On STAGNATION:
- No meaningful progress was possible with local exploration.

On FAILURE:
- Agent state may be degraded and require recovery.
