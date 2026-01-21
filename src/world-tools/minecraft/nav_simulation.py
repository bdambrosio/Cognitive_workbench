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
            state["y"] is the agent's feet block coordinate (where agent stands)
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

    # ---- Movement semantics ----
    # Treat state["y"] as the agent's feet Y (surface level). For climb/descend, we validate the
    # destination standing space at y±1 and the supporting block below it.

    def _validate_cell_fields(cell: Dict) -> Optional[str]:
        for field in REQUIRED_CELL_FIELDS:
            if field not in cell:
                return f"cell_missing_field:{field}"
        return None

    if action == "nav-climb":
        # Composite climb (approximation):
        # - Validate destination body space at (tx, y+1, tz)
        # - Validate destination head space at (tx, y+2, tz)
        # - Require support under destination feet (at y)
        #
        # This avoids assuming that a single query_cell() record can encode both body+head clearance
        # at the same Y, which is not true for the current SpatialMap adapter used by path-frontier.

        body_cell = query_cell(tx, y + 1, tz)
        if body_cell is None:
            if allow_unknown:
                return {"ok": True, "new_state": {"x": tx, "y": y + 1, "z": tz, "yaw": yaw}, "support": "unknown"}
            return {"ok": False, "reason": "unknown_climb_body_cell", "new_state": None, "support": None}
        missing = _validate_cell_fields(body_cell)
        if missing:
            return {"ok": False, "reason": missing, "new_state": None, "support": None}
        if not body_cell.get("clear_body", False):
            return {"ok": False, "reason": "collision", "new_state": None, "support": None}

        head_cell = query_cell(tx, y + 2, tz)
        if head_cell is None:
            if allow_unknown:
                return {"ok": True, "new_state": {"x": tx, "y": y + 1, "z": tz, "yaw": yaw}, "support": "unknown"}
            return {"ok": False, "reason": "unknown_climb_head_cell", "new_state": None, "support": None}
        missing = _validate_cell_fields(head_cell)
        if missing:
            return {"ok": False, "reason": missing, "new_state": None, "support": None}
        if not head_cell.get("clear_head", False):
            return {"ok": False, "reason": "collision", "new_state": None, "support": None}

        # Require support under the destination feet (at y)
        support_cell = query_cell(tx, y, tz)
        if support_cell is None:
            if allow_unknown:
                return {"ok": True, "new_state": {"x": tx, "y": y + 1, "z": tz, "yaw": yaw}, "support": "unknown"}
            return {"ok": False, "reason": "unknown_climb_support", "new_state": None, "support": None}
        missing = _validate_cell_fields(support_cell)
        if missing:
            return {"ok": False, "reason": missing, "new_state": None, "support": None}
        support = support_cell.get("support", "unknown")
        if support not in ("solid", "unsafe"):
            if allow_unknown and support == "unknown":
                support = "unsafe"
            else:
                return {"ok": False, "reason": "support_ambiguous", "new_state": None, "support": support}

        return {"ok": True, "new_state": {"x": tx, "y": y + 1, "z": tz, "yaw": yaw}, "support": body_cell.get("support", "unknown")}

    if action == "nav-descend":
        # Destination feet at y-1
        stand_cell = query_cell(tx, y - 1, tz)
        if stand_cell is None:
            if allow_unknown:
                return {"ok": True, "new_state": {"x": tx, "y": y - 1, "z": tz, "yaw": yaw}, "support": "unknown"}
            return {"ok": False, "reason": "unknown_descent_cell", "new_state": None, "support": None}
        missing = _validate_cell_fields(stand_cell)
        if missing:
            return {"ok": False, "reason": missing, "new_state": None, "support": None}
        if not stand_cell.get("clear_body", False) or not stand_cell.get("clear_head", False):
            return {"ok": False, "reason": "collision", "new_state": None, "support": None}

        # Validate drop magnitude (one block step by default)
        delta_y = -1.0
        if abs(delta_y) > max_drop or abs(delta_y) < min_drop:
            return {"ok": False, "reason": "invalid_descent_delta", "new_state": None, "support": None}

        # Require support under destination feet (at y-2)
        support_cell = query_cell(tx, y - 2, tz)
        if support_cell is None:
            if allow_unknown:
                return {"ok": True, "new_state": {"x": tx, "y": y - 1, "z": tz, "yaw": yaw}, "support": "unknown"}
            return {"ok": False, "reason": "unknown_support", "new_state": None, "support": None}
        missing = _validate_cell_fields(support_cell)
        if missing:
            return {"ok": False, "reason": missing, "new_state": None, "support": None}
        support = support_cell.get("support", "unknown")
        if support not in ("solid", "unsafe"):
            if allow_unknown and support == "unknown":
                support = "unsafe"
            else:
                return {"ok": False, "reason": "support_ambiguous", "new_state": None, "support": support}

        return {
            "ok": True,
            "new_state": {"x": tx, "y": y - 1, "z": tz, "yaw": yaw},
            "support": stand_cell.get("support", "unknown"),
        }

    # nav-move (flat): destination feet at same y
    stand_cell = query_cell(tx, y, tz)
    if stand_cell is None:
        if allow_unknown:
            return {"ok": True, "new_state": {"x": tx, "y": y, "z": tz, "yaw": yaw}, "support": "unknown"}
        return {"ok": False, "reason": "unknown_forward_cell", "new_state": None, "support": None}
    missing = _validate_cell_fields(stand_cell)
    if missing:
        return {"ok": False, "reason": missing, "new_state": None, "support": None}
    if not stand_cell.get("clear_body", False) or not stand_cell.get("clear_head", False):
        return {"ok": False, "reason": "collision", "new_state": None, "support": None}

    # Require support under feet (at y-1)
    support_cell = query_cell(tx, y - 1, tz)
    if support_cell is None:
        if allow_unknown:
            return {"ok": True, "new_state": {"x": tx, "y": y, "z": tz, "yaw": yaw}, "support": "unknown"}
        return {"ok": False, "reason": "unknown_support", "new_state": None, "support": None}
    missing = _validate_cell_fields(support_cell)
    if missing:
        return {"ok": False, "reason": missing, "new_state": None, "support": None}
    support = support_cell.get("support", "unknown")
    if support not in ("solid", "unsafe"):
        if allow_unknown and support == "unknown":
            support = "unsafe"
        else:
            return {"ok": False, "reason": "support_ambiguous", "new_state": None, "support": support}

    return {
        "ok": True,
        "new_state": {"x": tx, "y": y, "z": tz, "yaw": yaw},
        "support": stand_cell.get("support", "unknown"),
    }
