"""
nav_simulation.py

Pure navigation simulation helpers for Minecraft agents.

This module provides side-effect-free evaluation of single-step nav transitions.
It is intended for heavy use by path-* reasoning tools (path-reachable, path-propose),
NOT for direct planner invocation and NOT for embodied execution.

Design principles:
- Conservative: unknown == failure (unless explicitly allowed)
- Deterministic
- Semantics aligned with nav-move / nav-descend
- No executor, no HTTP, no world mutation
"""

import math
from typing import Dict, Optional, Callable, Literal

# ---- Types ----

SupportType = Literal["solid", "unsafe", "unknown", "air"]
NavActionType = Literal["nav-move", "nav-descend", "nav-climb"]

# Expected keys in cell records returned by spatial map
# This is intentionally minimal and explicit.
REQUIRED_CELL_FIELDS = {
    "clear_body",   # bool
    "clear_head",   # bool
    "support",      # SupportType
}

# ---- Helpers ----

def yaw_to_forward_delta(yaw_deg: float) -> tuple[int, int]:
    """
    Convert Minecraft yaw (degrees) to a discrete forward (dx, dz).

    Assumes yaw has already been snapped to cardinal directions
    (0, 90, 180, 270), as enforced by nav-turn / init.
    """
    yaw_rad = math.radians(yaw_deg)
    dx = int(round(-math.sin(yaw_rad)))
    dz = int(round(-math.cos(yaw_rad)))
    return dx, dz


# ---- Core simulation primitive ----

def simulate_nav_step(
    *,
    state: Dict,
    action: NavActionType,
    query_cell: Callable[[int, int, int], Optional[Dict]],
    allow_unknown: bool = False,
    max_drop: float = 1.2,
    min_drop: float = 0.2,
) -> Dict:
    """
    Simulate exactly ONE nav-* action from a hypothetical state.

    Args:
        state:
            Dict with keys {"x","y","z","yaw"} (block-centered coordinates)
        action:
            One of {"nav-move", "nav-descend", "nav-climb"}
        query_cell:
            Callable (x:int, y:int, z:int) -> cell dict or None
            The cell dict must minimally include:
              - clear_body: bool
              - clear_head: bool
              - support: "solid" | "unsafe" | "unknown" | "air"
        allow_unknown:
            If False (default), any unknown cell/support is a hard failure.
        max_drop:
            Maximum allowed downward delta for descend semantics.
        min_drop:
            Minimum delta required to count as a descend.

    Returns:
        {
          "ok": bool,
          "new_state": {x,y,z,yaw} | None,
          "reason": str | None,
          "support": SupportType | None
        }
    """

    # ---- Validate inputs ----

    if not state or not action or not query_cell:
        return {"ok": False, "reason": "missing_input", "new_state": None, "support": None}

    for k in ("x", "y", "z", "yaw"):
        if k not in state:
            return {"ok": False, "reason": f"missing_state_field:{k}", "new_state": None, "support": None}

    if action not in ("nav-move", "nav-descend", "nav-climb"):
        return {"ok": False, "reason": f"unsupported_action:{action}", "new_state": None, "support": None}

    x = int(round(state["x"]))
    y = int(round(state["y"]))
    z = int(round(state["z"]))
    yaw = state["yaw"]

    dx, dz = yaw_to_forward_delta(yaw)
    tx, tz = x + dx, z + dz

    # ---- Query forward cell at same Y ----

    fwd_cell = query_cell(tx, y, tz)
    if fwd_cell is None:
        if allow_unknown:
            return {
                "ok": True,
                "new_state": {"x": tx, "y": y, "z": tz, "yaw": yaw},
                "support": "unknown",
            }
        return {"ok": False, "reason": "unknown_forward_cell", "new_state": None, "support": None}

    # Validate expected fields
    for field in REQUIRED_CELL_FIELDS:
        if field not in fwd_cell:
            return {"ok": False, "reason": f"cell_missing_field:{field}", "new_state": None, "support": None}

    # ---- Collision checks ----

    if not fwd_cell["clear_body"] or not fwd_cell["clear_head"]:
        return {"ok": False, "reason": "collision", "new_state": None, "support": None}

    # ---- Vertical semantics ----

    target_y = y

    if action == "nav-climb":
        up_cell = query_cell(tx, y + 1, tz)
        if up_cell is None:
            return {"ok": False, "reason": "unknown_climb_cell", "new_state": None, "support": None}
        if not up_cell.get("clear_body", False):
            return {"ok": False, "reason": "blocked_climb", "new_state": None, "support": None}
        target_y = y + 1

    elif action == "nav-descend":
        down_cell = query_cell(tx, y - 1, tz)
        if down_cell is None:
            return {"ok": False, "reason": "unknown_descent_cell", "new_state": None, "support": None}

        support = down_cell.get("support", "unknown")
        if support not in ("solid", "unsafe"):
            return {"ok": False, "reason": "support_ambiguous", "new_state": None, "support": support}

        delta_y = -1.0
        if abs(delta_y) > max_drop or abs(delta_y) < min_drop:
            return {"ok": False, "reason": "invalid_descent_delta", "new_state": None, "support": support}

        target_y = y - 1

    else:  # nav-move (flat)
        down_cell = query_cell(tx, y - 1, tz)
        if down_cell is None:
            if allow_unknown:
                return {
                    "ok": True,
                    "new_state": {"x": tx, "y": y, "z": tz, "yaw": yaw},
                    "support": "unknown",
                }
            return {"ok": False, "reason": "unknown_support", "new_state": None, "support": None}

        support = down_cell.get("support", "unknown")
        if support not in ("solid", "unsafe"):
            return {"ok": False, "reason": "support_ambiguous", "new_state": None, "support": support}

    # ---- Success ----

    return {
        "ok": True,
        "new_state": {
            "x": tx,
            "y": target_y,
            "z": tz,
            "yaw": yaw,
        },
        "support": fwd_cell.get("support", "unknown"),
    }
