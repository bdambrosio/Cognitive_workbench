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


def _round_to_cardinal(yaw_deg: float) -> int:
    """
    Round yaw to nearest cardinal direction (0, 90, 180, 270).
    
    Args:
        yaw_deg: Yaw in degrees
    
    Returns:
        Nearest cardinal yaw (0, 90, 180, or 270)
    """
    normalized = yaw_deg % 360
    if normalized < 0:
        normalized += 360
    
    # Thresholds at midpoints: 45°, 135°, 225°, 315°
    if normalized < 45 or normalized >= 315:
        return 0
    elif normalized < 135:
        return 90
    elif normalized < 225:
        return 180
    else:  # normalized < 315
        return 270


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
    # Prefer over-approximation: when in doubt, include possible frontier elements.
    allow_unknown = bool(kwargs.get("allow_unknown", True))

    # ---- Obtain current pose ----

    status = executor.execute_action_with_log({"type": "mc-status"}, "path-frontier")
    if status.get("status") != "success":
        return executor._create_uniform_return("failed", value="Failed to obtain current status", reason="status_failed")

    pos = status["data"]["position"]
    yaw = status["data"].get("yaw", 0.0)

    start_state = {"x": int(round(pos["x"])), "y": int(round(pos["y"])), "z": int(round(pos["z"])), "yaw": yaw}

    # ---- Spatial map adapter ----
    # We keep this local and explicit to avoid hidden dependencies.
    # Converts 2D spatial map cells to 3D query format for nav_simulation.
    # Uses uniform surface definition: surface Y = support_y + 1 (where agent stands)

    spatial_map = executor.get_world_state("spatial_map")

    # If we have a mapped support_y for the current cell, snap start_state.y to surface_y (= support_y + 1).
    # This avoids brittle dependence on mc-status rounding and aligns nav_simulation with the 2D map semantics.
    try:
        if spatial_map:
            start_cell = spatial_map.get_cell(start_state["x"], start_state["z"])
            if start_cell:
                support_y = start_cell.get("support", {}).get("support_y")
                if support_y is not None:
                    start_state["y"] = int(round(float(support_y))) + 1
    except Exception:
        pass

    # ---- BFS over bounded nav actions ----

    # FIX: Visited set must include YAW.
    # If we ignore yaw, a 'turn' action (which changes yaw but not x,y,z) is immediately pruned
    # because the agent is "already at" that x,y,z.
    # State tuple: (x, y, z, cardinal_yaw)
    visited: Set[Tuple[int, int, int, int]] = set()
    
    # Store shortest path to distinct X,Z coordinates (regardless of final orientation)
    reachable_paths: Dict[Tuple[int, int], List[str]] = {}

    queue = deque()
    
    start_yaw_cardinal = _round_to_cardinal(start_state["yaw"])
    queue.append((start_state, 0, []))  # (state, depth, path)
    visited.add((start_state["x"], start_state["y"], start_state["z"], start_yaw_cardinal))

    while queue:
        state, depth, path = queue.popleft()

        # Record reachable horizontal position with path (exclude origin later if desired)
        xz_key = (state["x"], state["z"])
        if xz_key not in reachable_paths:
            reachable_paths[xz_key] = path  # Store first path found (BFS ensures shortest)

        if depth >= max_actions:
            continue

        # Create query_cell adapter with current state for query-time blocking computation
        def query_cell(x: int, y: int, z: int):
            """
            Adapter over the persistent spatial map, with local_grid safety override.
            Converts 2D cell data to 3D query format expected by nav_simulation.
            Computes "blocked" at query time relative to current state position.
            
            local_grid overrides spatial_map for immediate cells (more current, reflects recent digs/places).
            
            Uniform surface definition:
            - Surface Y = support_y + 1 (top face of support block where agent stands)
            - Support block Y = support_y (the solid block providing support)
            
            Returns dict with: clear_body: bool, clear_head: bool, support: SupportType
            or None if cell is unknown.
            """
            # Compute relative position from start_state (local_grid uses relative coords)
            dx = x - start_state["x"]
            dy = y - start_state["y"]
            dz = z - start_state["z"]
            
            # Check local_grid for safety (if within boundary=1, i.e., ±1 block)
            # local_grid is more current and reflects recent agent actions
            if abs(dx) <= 1 and abs(dy) <= 1 and abs(dz) <= 1:
                try:
                    from local_grid import check_cell_safety
                    safety = check_cell_safety(executor, dx, dy, dz)
                    if safety:
                        # Check for hazards - mark as unsafe
                        if safety.get('has_hazard'):
                            return {
                                "clear_body": False,
                                "clear_head": False,
                                "support": "unsafe"
                            }
                        
                        # Check for cliff - mark as unsafe
                        if safety.get('is_cliff'):
                            return {"clear_body": False, "clear_head": False, "support": "unsafe"}
                        
                        # Check for void - mark as air
                        if safety.get('is_void'):
                            return {"clear_body": True, "clear_head": True, "support": "air"}
                except Exception:
                    # Non-fatal - fall back to spatial_map
                    pass
            
            # Fall back to spatial_map (original logic)
            if not spatial_map:
                return {"clear_body": True, "clear_head": True, "support": "unknown"} if allow_unknown else None
            
            # Get 2D cell from spatial map
            cell = spatial_map.get_cell(x, z)
            if not cell:
                return {"clear_body": True, "clear_head": True, "support": "unknown"} if allow_unknown else None
            
            # Extract support information
            support_data = cell.get('support', {})
            support_y = support_data.get('support_y')
            walkable = support_data.get('walkable', False)
            
            # If we don't know the support Y, we can't determine 3D position
            if support_y is None:
                return {"clear_body": True, "clear_head": True, "support": "unknown"} if allow_unknown else None
            
            # Uniform surface definition: surface = support_y + 1
            surface_y = support_y + 1
            
            # Compute if this cell is blocked from current state position (query-time)
            is_blocked = spatial_map.is_blocked_from(
                state['x'], state['z'], state['y'],
                x, z, y
            )
            
            # Determine support type based on queried Y and cell data
            support_type = "unknown"
            
            if y == surface_y:
                # Querying at surface level (where agent stands)
                if walkable and not is_blocked:
                    support_type = "solid"
                elif is_blocked:
                    support_type = "air"  # Blocked = can't stand here
                else:
                    support_type = "unsafe"
            elif y == support_y:
                # Querying at support block level
                if walkable:
                    support_type = "solid"
                else:
                    support_type = "unsafe"
            elif y < support_y:
                # Querying below support block - assume solid (underground)
                support_type = "solid"
            else:
                # Querying above surface - assume air unless we know otherwise
                # For clearance checks, we'll be conservative
                support_type = "air"
            
            # Infer clearance (clear_body at Y+1, clear_head at Y+2)
            # We don't store clearance in cells, so we infer conservatively:
            # - If queried Y is surface_y, assume clearance above (agent can stand here)
            # - If queried Y is above surface_y, assume no clearance (blocks exist)
            # - If queried Y is below surface_y, assume no clearance (underground)
            
            clear_body = False
            clear_head = False
            
            if y == surface_y:
                # At surface level - assume clearance if walkable and not blocked
                if walkable and not is_blocked:
                    clear_body = True
                    clear_head = True  # Conservative: assume head clearance if body clear
            elif y > surface_y:
                # Above surface - check if this is clearance space
                # If we're querying for clearance (Y = surface_y + 1 or +2), assume clear
                # Otherwise, assume blocked
                if y == surface_y + 1:
                    clear_body = True
                if y == surface_y + 2:
                    clear_head = True
            
            return {
                "clear_body": clear_body,
                "clear_head": clear_head,
                "support": support_type
            }

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
                
                # Check if this configuration (pos + yaw) has been visited
                key = (new_state["x"], new_state["y"], new_state["z"], cardinal_yaw)
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
            
            # Ensure the new yaw is treated cardinally for the visited set key
            new_yaw_cardinal = _round_to_cardinal(new_state["yaw"])
            key = (new_state["x"], new_state["y"], new_state["z"], new_yaw_cardinal)
            
            if key in visited:
                continue

            visited.add(key)
            queue.append((new_state, depth + 1, path + [action]))

    # ---- Prepare output ----

    # Remove starting position from results (optional, but clearer)
    start_xz = (start_state["x"], start_state["z"])
    if start_xz in reachable_paths:
        del reachable_paths[start_xz]

    # Convert to relative coordinates (dx, dz) from start position
    start_x = start_state["x"]
    start_z = start_state["z"]
    reachable_list: List[Dict] = [
        {"dx": x - start_x, "dz": z - start_z, "path": reachable_paths[(x, z)]}
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