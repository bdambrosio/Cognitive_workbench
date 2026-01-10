"""
path_frontier.py

Enumerate locally reachable positions within a bounded number of nav actions.

This is a *reasoning* tool, not an execution tool.
It performs a conservative bounded BFS over nav semantics using simulate_nav_step.

Primary purposes:
- Validate nav_simulation correctness
- Provide a safe exploration primitive
- Serve as a substrate for later path-propose, path-adjacent-to, etc.
"""

import logging
import os
import sys
from collections import deque
from typing import Dict, Set, Tuple, List

# Ensure minecraft world-tools root is importable (src/world-tools/minecraft)
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

from infospace_executor import InfospaceExecutor
from nav_simulation import simulate_nav_step

logger = logging.getLogger(__name__)


def _round_to_cardinal(yaw_deg: float) -> float:
    """
    Round yaw to nearest cardinal direction (0, 90, 180, 270).
    
    Args:
        yaw_deg: Yaw in degrees
    
    Returns:
        Nearest cardinal yaw (0, 90, 180, or 270)
    """
    cardinals = [0, 90, 180, 270]
    normalized = yaw_deg % 360
    
    # Find nearest cardinal
    nearest = min(cardinals, key=lambda c: min(
        abs(normalized - c),
        abs(normalized - c + 360),
        abs(normalized - c - 360)
    ))
    
    return nearest


# Action set for initial frontier exploration.
# Includes movement actions and orientation changes.
DEFAULT_ACTIONS = ("nav-move", "nav-descend", "nav-climb", "nav-turn-left", "nav-turn-right")


def tool(input_value=None, **kwargs):
    """
    Enumerate reachable nearby (x,z) positions within a tight action bound.

    Args:
        max_actions: int (default: 4)
        allow_unknown: bool (default: False)

    Returns:
        Uniform tool return with:
          value = {
            "reachable": [{"x": int, "z": int, "path": List[str]}, ...],
            "count": int
          }
          where path is sequence of action names from start to position
    """

    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {
            "status": "failed",
            "reason": "executor_not_available",
            "value": None,
            "resource_id": None,
        }

    max_actions = int(kwargs.get("max_actions", 4))
    allow_unknown = bool(kwargs.get("allow_unknown", False))

    # ---- Obtain current pose ----

    status = executor.execute_action_with_log({"type": "mc-status"}, "path-frontier")
    if status.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain current status",
            reason="status_failed",
        )

    pos = status["data"]["position"]
    yaw = status["data"].get("yaw", 0.0)

    start_state = {
        "x": int(round(pos["x"])),
        "y": int(round(pos["y"])),
        "z": int(round(pos["z"])),
        "yaw": yaw,
    }

    # ---- Spatial map adapter ----
    # We keep this local and explicit to avoid hidden dependencies.

    spatial_map = executor.get_world_state("spatial_map")

    def query_cell(x: int, y: int, z: int):
        """
        Adapter over the persistent spatial map.

        Expected to return a dict with:
          clear_body: bool
          clear_head: bool
          support: str
        or None if unknown.
        """
        if not spatial_map:
            return None
        return spatial_map.get((x, y, z))

    # ---- BFS over bounded nav actions ----

    visited: Set[Tuple[int, int, int]] = set()
    reachable_paths: Dict[Tuple[int, int], List[str]] = {}  # Maps (x, z) -> path sequence

    queue = deque()
    queue.append((start_state, 0, []))  # (state, depth, path)
    visited.add((start_state["x"], start_state["y"], start_state["z"]))

    while queue:
        state, depth, path = queue.popleft()

        # Record reachable horizontal position with path (exclude origin later if desired)
        xz_key = (state["x"], state["z"])
        if xz_key not in reachable_paths:
            reachable_paths[xz_key] = path  # Store first path found (BFS ensures shortest)

        if depth >= max_actions:
            continue

        for action in DEFAULT_ACTIONS:
            # Handle nav-turn actions (orientation change only, no position change)
            if action.startswith("nav-turn-"):
                direction = action.split("-")[-1]  # "left" or "right"
                current_yaw = state["yaw"]
                
                # Calculate new yaw
                if direction == "right":
                    new_yaw = current_yaw + 90
                elif direction == "left":
                    new_yaw = current_yaw - 90
                else:
                    continue  # Skip invalid direction
                
                # Normalize to 0-360
                new_yaw = new_yaw % 360
                if new_yaw < 0:
                    new_yaw += 360
                
                # Round to nearest cardinal
                cardinal_yaw = _round_to_cardinal(new_yaw)
                
                # Create new state with updated yaw, same position
                new_state = {
                    "x": state["x"],
                    "y": state["y"],
                    "z": state["z"],
                    "yaw": cardinal_yaw,
                }
                
                # Check if this position has been visited (yaw not included in visited check)
                key = (new_state["x"], new_state["y"], new_state["z"])
                if key in visited:
                    continue
                
                visited.add(key)
                queue.append((new_state, depth + 1, path + [action]))
                continue
            
            # Handle movement actions (nav-move, nav-descend, nav-climb)
            result = simulate_nav_step(
                state=state,
                action=action,
                query_cell=query_cell,
                allow_unknown=allow_unknown,
            )

            if not result.get("ok"):
                continue

            new_state = result.get("new_state")
            if not new_state:
                continue

            key = (new_state["x"], new_state["y"], new_state["z"])
            if key in visited:
                continue

            visited.add(key)
            queue.append((new_state, depth + 1, path + [action]))

    # ---- Prepare output ----

    # Remove starting position from results (optional, but clearer)
    start_xz = (start_state["x"], start_state["z"])
    if start_xz in reachable_paths:
        del reachable_paths[start_xz]

    reachable_list: List[Dict] = [
        {"x": x, "z": z, "path": reachable_paths[(x, z)]}
        for (x, z) in sorted(reachable_paths.keys())
    ]

    return executor._create_uniform_return(
        "success",
        value={
            "reachable": reachable_list,
            "count": len(reachable_list),
        },
    )


if __name__ == "__main__":
    print("path-frontier module loaded")
