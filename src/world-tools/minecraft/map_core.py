"""
cell_id
Logical grid coordinate (agent-centered or global, your choice)

support (planner-critical):
walkable: can the agent safely stand here
support_y: Y of the block providing support
delta_y_from_agent: relative height (more useful than absolute Y)
movement_class: flat | step_up | step_down | drop | blocked | unknown
surface
support_block: the actual block that supports standing (not “top block”)
surface_block_class: coarse enum (stone, dirt, sand, foliage, water, lava, unknown)
is_partial_block: snow layer, carpet, etc.

hazards:
flags: e.g. lava, deep_water, cactus, void_risk
exposure_risk: risk of standing idle
escape_difficulty: how hard recovery would be if something goes wrong

resources:
resources_visible: affordances, not inventory (wood, stone, food_source, …)
harvest_actions: diggable, choppable, collectable
tool_required: none | wood | stone | iron | unknown

environment:
Minimal survival cues (light, spawn risk, temporal sensitivity)
observability (mandatory for sane exploration)
observation_mode: direct | inferred | propagated | unknown
confidence: explicit epistemic weight (0–1)
last_observed_at: timestamp or null

provenance:
Debugging + learning hooks (who/what updated this, and why)
"""

MAP_CELL_SCHEMA = {
  "cell_version": "1.0", "cell_id": {"x": 0, "z": 0},
  "support": {"walkable": False, "support_y": None, "delta_y_from_agent": None, "movement_class": "unknown"},
  "surface": { "support_block": None, "surface_block_class": "unknown", "is_fluid": False, "is_partial_block": False},
  "hazards": {"flags": [], "exposure_risk": "unknown", "escape_difficulty": "unknown"},
  "resources": {"resources_visible": [], "harvest_actions": [], "tool_required": "unknown"},
  "environment": {"light_level": "unknown", "spawn_risk": "unknown", "time_sensitive": False},
  "observability": {"observed_from": {"x": None, "y": None, "z": None}, "observation_mode": "unknown", "confidence": 0.0, "last_observed_at": None},
  "provenance": {"updated_by": "unknown", "update_reason": "unknown"},
}
