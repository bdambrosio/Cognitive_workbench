"""
Navigation climb primitive (v2).

Attempt exactly ONE adjacent climb with automatic handling of Minecraft ascent semantics:
- Try walk-up (for slabs/stairs/partial rises)
- If needed, try jump-up (for full 1-block ledges)
- Accept walkable landings (e.g., snow layers) by default

Atomic, bounded, composable.
"""

import logging
import math
import os
import requests
import sys
import time
from typing import Dict, Tuple, Optional
from infospace_executor import InfospaceExecutor

# Import nav_core helpers (ensure_grid_aligned)
_THIS_DIR = os.path.dirname(__file__)
_NAV_CORE_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _NAV_CORE_DIR not in sys.path:
    sys.path.insert(0, _NAV_CORE_DIR)
from nav_core import ensure_grid_aligned, snap_to_position

logger = logging.getLogger(__name__)

# Session-local rolling occupancy grid (agent-owned)
try:
    from local_grid import get_cell_name, is_air_name, set_center_from_pose
except Exception:
    set_center_from_pose = None
    get_cell_name = None
    is_air_name = None

DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")
MAX_NAV_HISTORY = 100


def _format_af1_text(af1: Dict) -> str:
    """Compact AF1 string (best-effort)."""
    if not isinstance(af1, dict):
        return ""
    yaw = af1.get("yaw", af1.get("yaw_cardinal"))
    v = af1.get("vertical", {}) if isinstance(af1.get("vertical"), dict) else {}
    nav = af1.get("nav", {}) if isinstance(af1.get("nav"), dict) else {}
    climb = nav.get("climb", nav.get("climb_forward_by_yaw", {}))
    placement = af1.get("placement", {}) if isinstance(af1.get("placement"), dict) else {}
    pending_targets = placement.get("pending_targets", []) if isinstance(placement.get("pending_targets"), list) else []
    anchors = af1.get("anchors", []) if isinstance(af1.get("anchors"), list) else []
    lines = [
        f"AF1 yaw={yaw}",
        f"Vertical: support_depth={v.get('support_depth','unknown')} gap_like={bool(v.get('gap_like'))} up_blk={v.get('up_blk', v.get('up_block'))} down_blk={v.get('down_blk', v.get('down_block'))}",
    ]
    if isinstance(climb, dict) and climb:
        # Support both v2 and v0.1 formats
        if any(isinstance(climb.get(k), str) for k in ("0", "90", "180", "270")):
            climb_s = " ".join([f"{k}:{climb.get(k,'?')}" for k in ("0", "90", "180", "270")])
        else:
            climb_s = " ".join([f"{k}:{(climb.get(k, {}) or {}).get('status','?')}" for k in ("0", "90", "180", "270")])
        lines.append(f"nav-climb by yaw: {climb_s}")
    if pending_targets:
        lines.append(f"placement: pending_targets={len(pending_targets)} (must mc-observe next)")
    if anchors:
        a0 = anchors[0]
        lines.append(f"anchors: n={len(anchors)} anchor0={[a0.get('dx'), a0.get('dy'), a0.get('dz')]} faces={a0.get('faces')}")
    return "\n".join(lines).strip()


def _calculate_snap_position_and_yaw(start_pos: Dict, end_pos: Dict, current_yaw: Optional[float] = None) -> Tuple[Dict, float, float]:
    """
    Calculate block center position and yaw from movement.
    
    Args:
        start_pos: Starting position dict with x, y, z
        end_pos: Ending position dict with x, y, z
        current_yaw: Current yaw in degrees (used if no horizontal movement)
    
    Returns:
        (snap_pos, yaw, pitch) where:
        - snap_pos: {"x": center_x, "y": end_y, "z": center_z}
        - yaw: degrees (0-360)
        - pitch: degrees (always 0 for horizontal movement)
    """
    # Get block coordinates (floor)
    block_x = math.floor(end_pos['x'])
    block_z = math.floor(end_pos['z'])
    
    # Block center (keep actual Y)
    center_x = block_x + 0.5
    center_z = block_z + 0.5
    
    # Calculate movement vector
    dx = end_pos['x'] - start_pos['x']
    dz = end_pos['z'] - start_pos['z']
    
    # Calculate yaw from movement direction
    if abs(dx) < 0.001 and abs(dz) < 0.001:
        # No horizontal movement - preserve current yaw
        if current_yaw is not None:
            yaw = current_yaw
        else:
            yaw = 0.0
    else:
        # Calculate yaw from movement vector (Minecraft convention)
        yaw_rad = math.atan2(-dx, -dz)
        yaw = math.degrees(yaw_rad)
        # Normalize to 0-360
        yaw = yaw % 360
        if yaw < 0:
            yaw += 360
    
    # Pitch: always 0 for horizontal movement
    pitch = 0.0
    
    return {"x": center_x, "y": end_pos['y'], "z": center_z}, yaw, pitch


def _call_move_endpoint(minecraft_url: str, forward: bool = True, duration: float = 0.6, jump: bool = False, check_collision: bool = True) -> Dict:
    """
    Call bridge /act/move endpoint directly.
    
    Returns:
        Dict with {"ok": bool, "status": str, "final_position": dict, ...} or None on error
    """
    move_params = {"forward": True, "check_collision": check_collision}
    if jump:
        move_params["jump"] = True
    if duration is not None:
        move_params["duration_ms"] = int(duration * 1000)
    
    try:
        timeout_seconds = duration + 5.0
        url = f"{minecraft_url}/act/move"
        logger.debug(f"nav-climb: POST {url} body={move_params}")
        response = requests.post(url, json=move_params, timeout=max(10.0, timeout_seconds))
        response.raise_for_status()
        data = response.json()
        return data
    except Exception as e:
        logger.warning(f"nav-climb: move request failed: {e}")
        return None


def _snap_to_position(executor: InfospaceExecutor, snap_pos: Dict, minecraft_url: str) -> bool:
    """
    Snap player to exact position via bridge endpoint (preserves current orientation).
    
    Returns:
        True if successful, False otherwise
    """
    try:
        url = f"{minecraft_url}/act/snapto"
        body = {
            "x": snap_pos['x'],
            "y": snap_pos['y'],
            "z": snap_pos['z'],
        }
        logger.debug(f"nav-climb: POST {url} body={body}")
        response = requests.post(url, json=body, timeout=5.0)
        response.raise_for_status()
        data = response.json()
        if not data.get("ok"):
            logger.warning(f"nav-climb: snapto failed: {data.get('error', 'unknown error')}")
            return False
        return True
    except Exception as e:
        logger.warning(f"nav-climb: snapto request failed: {e}")
        return False


def _to_relative_delta(from_pos: Dict[str, float], to_pos: Optional[Dict[str, float]]) -> Dict[str, float]:
    """Convert absolute positions to relative delta (dx, dy, dz)."""
    if to_pos is None:
        return {"dx": 0.0, "dy": 0.0, "dz": 0.0}
    return {
        "dx": to_pos.get("x", 0.0) - from_pos.get("x", 0.0),
        "dy": to_pos.get("y", 0.0) - from_pos.get("y", 0.0),
        "dz": to_pos.get("z", 0.0) - from_pos.get("z", 0.0),
    }


def _calculate_forward_block_abs(position: Dict, yaw: float) -> Tuple[int, int, int]:
    """
    Calculate absolute block coordinates one block forward from current position.
    
    Args:
        position: Position dict with x, y, z (floats)
        yaw: Yaw angle in degrees
    
    Returns:
        Tuple of (x, y, z) absolute block coordinates (integers)
    """
    bx = int(math.floor(position['x']))
    by = int(math.floor(position['y']))
    bz = int(math.floor(position['z']))
    
    # Forward vector from yaw: (-sin(yaw), 0, -cos(yaw))
    dx = -math.sin(math.radians(yaw))
    dz = -math.cos(math.radians(yaw))
    
    return (bx + int(round(dx)), by, bz + int(round(dz)))


def _attempt_collision_recovery(executor: InfospaceExecutor, position: Dict, yaw: float, diagnostics: Dict, minecraft_url: str, is_jump: bool = False) -> bool:
    """
    Attempt to recover from collision by digging obstructing blocks.
    
    Systematically checks all clearance diagnostics and digs at blocked positions.
    Uses the same coordinate system as mc-observe for consistency.
    
    Args:
        executor: InfospaceExecutor instance
        position: Current position dict
        yaw: Current yaw in degrees
        diagnostics: Collision diagnostics dict with clearance flags
        minecraft_url: Minecraft bridge URL
        is_jump: Whether this is a jump attempt (requires additional overhead checks)
    
    Returns:
        True if recovery was attempted, False otherwise
    """
    bx = int(math.floor(position['x']))
    by = int(math.floor(position['y']))
    bz = int(math.floor(position['z']))
    
    # Calculate forward block position (using same method as mc-observe rel_to_abs)
    dx = -math.sin(math.radians(yaw))
    dz = -math.cos(math.radians(yaw))
    fwd_x = bx + int(round(dx))
    fwd_z = bz + int(round(dz))
    
    recovery_attempted = False
    
    # Systematic recovery: check all clearance diagnostics and dig blocked positions
    # Use same coordinate calculations as mc-observe (rel_to_abs with forward/up offsets)
    def _known_air(x: int, y: int, z: int) -> bool:
        try:
            if get_cell_name is None or is_air_name is None:
                return False
            name = get_cell_name(executor, x, y, z)
            return bool(name) and is_air_name(name)
        except Exception:
            return False
    
    # 1. Forward body space (destination body): rel_to_abs(position, yaw, 1, 0, 1) = (fwd_x, by+1, fwd_z)
    if diagnostics.get("clear_fwd_body") == False:
        if _known_air(fwd_x, by + 1, fwd_z):
            logger.debug(f"nav-climb: Skip recovery dig (known air) at ({fwd_x}, {by+1}, {fwd_z})")
        else:
            dig_result = executor.execute_action_with_log(
                {"type": "mc-dig", "x": float(fwd_x), "y": float(by + 1), "z": float(fwd_z)},
                "nav-climb:recovery:dest_body"
            )
            if dig_result.get("status") == "success":
                recovery_attempted = True
                logger.info(f"nav-climb: Recovery dig at destination body ({fwd_x}, {by+1}, {fwd_z})")
                time.sleep(0.3)
    
    # 2. Forward head space (destination head): rel_to_abs(position, yaw, 1, 0, 2) = (fwd_x, by+2, fwd_z)
    if diagnostics.get("clear_fwd_head") == False:
        if _known_air(fwd_x, by + 2, fwd_z):
            logger.debug(f"nav-climb: Skip recovery dig (known air) at ({fwd_x}, {by+2}, {fwd_z})")
        else:
            dig_result = executor.execute_action_with_log(
                {"type": "mc-dig", "x": float(fwd_x), "y": float(by + 2), "z": float(fwd_z)},
                "nav-climb:recovery:dest_head"
            )
            if dig_result.get("status") == "success":
                recovery_attempted = True
                logger.info(f"nav-climb: Recovery dig at destination head ({fwd_x}, {by+2}, {fwd_z})")
                time.sleep(0.3)
    
    # 3. Source overhead body space: rel_to_abs(position, yaw, 0, 0, 1) = (bx, by+1, bz)
    # Check both up_block (block exists) and clear_up_body (clearance flag)
    if diagnostics.get("up_block") is not None or diagnostics.get("clear_up_body") == False:
        if _known_air(bx, by + 1, bz):
            logger.debug(f"nav-climb: Skip recovery dig (known air) at ({bx}, {by+1}, {bz})")
        else:
            dig_result = executor.execute_action_with_log(
                {"type": "mc-dig", "x": float(bx), "y": float(by + 1), "z": float(bz)},
                "nav-climb:recovery:source_body"
            )
            if dig_result.get("status") == "success":
                recovery_attempted = True
                logger.info(f"nav-climb: Recovery dig at source body overhead ({bx}, {by+1}, {bz})")
                time.sleep(0.3)
    
    # 4. Source overhead head space: rel_to_abs(position, yaw, 0, 0, 2) = (bx, by+2, bz)
    if diagnostics.get("clear_up_head") == False:
        if _known_air(bx, by + 2, bz):
            logger.debug(f"nav-climb: Skip recovery dig (known air) at ({bx}, {by+2}, {bz})")
        else:
            dig_result = executor.execute_action_with_log(
                {"type": "mc-dig", "x": float(bx), "y": float(by + 2), "z": float(bz)},
                "nav-climb:recovery:source_head"
            )
            if dig_result.get("status") == "success":
                recovery_attempted = True
                logger.info(f"nav-climb: Recovery dig at source head overhead ({bx}, {by+2}, {bz})")
                time.sleep(0.3)
    
    # 5. Jump-specific: destination overhead at peak height (y+3) for jump arc clearance
    # During jump, head can reach y+3, so check destination overhead at that height
    if is_jump:
        if _known_air(fwd_x, by + 3, fwd_z):
            logger.debug(f"nav-climb: Skip recovery dig (known air) at ({fwd_x}, {by+3}, {fwd_z})")
        else:
            dig_result = executor.execute_action_with_log(
                {"type": "mc-dig", "x": float(fwd_x), "y": float(by + 3), "z": float(fwd_z)},
                "nav-climb:recovery:dest_jump_arc"
            )
            if dig_result.get("status") == "success":
                recovery_attempted = True
                logger.info(f"nav-climb: Recovery dig at destination jump arc height ({fwd_x}, {by+3}, {fwd_z})")
                time.sleep(0.3)
    
    # 6. Check cover_block from nav_surface at destination (blocks jump arc at support_y+1)
    # cover_block is at surface level (support_y + 1) and can block jump arc
    nav_surface = diagnostics.get("nav_surface", [])
    if isinstance(nav_surface, list):
        for ns in nav_surface:
            if not isinstance(ns, dict):
                continue
            ns_x = ns.get("x")
            ns_z = ns.get("z")
            support_y = ns.get("support_y")
            cover_block = ns.get("cover_block")
            # Check if this nav_surface cell is at the forward destination
            if ns_x == fwd_x and ns_z == fwd_z and support_y is not None and cover_block:
                # cover_block is at support_y + 1 (surface level)
                cover_y = int(support_y) + 1
                # For climb, we need clearance up to by+2 (head space), so dig cover_block if it's in the way
                if cover_y >= by + 1 and cover_y <= by + 3:
                    if _known_air(fwd_x, cover_y, fwd_z):
                        logger.debug(f"nav-climb: Skip recovery dig (known air) at ({fwd_x}, {cover_y}, {fwd_z})")
                    else:
                        dig_result = executor.execute_action_with_log(
                            {"type": "mc-dig", "x": float(fwd_x), "y": float(cover_y), "z": float(fwd_z)},
                            "nav-climb:recovery:dest_cover_block"
                        )
                        if dig_result.get("status") == "success":
                            recovery_attempted = True
                            logger.info(f"nav-climb: Recovery dig at destination cover_block ({fwd_x}, {cover_y}, {fwd_z})")
                            time.sleep(0.3)
    
    return recovery_attempted


def _update_nav_state(executor: InfospaceExecutor, pose: Dict, support_here: str, fell: bool, was_fall: bool = False):
    """
    Update navigation state by prepending a new entry to the nav list.
    
    Args:
        executor: InfospaceExecutor instance
        pose: Dict with x, y, z, yaw
        support_here: "solid", "unsafe", "unknown", etc.
        fell: Whether the agent fell
        was_fall: Whether this was a fall event (for nav-backtrack safety check)
    """
    nav_list = executor.get_world_state("nav")
    if not isinstance(nav_list, list):
        nav_list = []
    
    nav_entry = {
        "pose": pose.copy(),
        "support_here": support_here,
        "fell": fell,
        "was_fall": was_fall,
        "timestamp": time.time()
    }
    
    # Prepend new entry
    nav_list.insert(0, nav_entry)
    
    # Limit to MAX_NAV_HISTORY
    if len(nav_list) > MAX_NAV_HISTORY:
        nav_list = nav_list[:MAX_NAV_HISTORY]
    
    executor.set_world_state("nav", nav_list)


def tool(input_value=None, **kwargs):
    """
    Attempt a single climb (+~1Y) forward.

    Args:
        step_duration: float seconds for each attempt (default: 0.6)
        allow_walkable_landing: bool (default: True)
        min_delta_y: float minimum Y gain to count as climb (default: 0.9)

    Returns:
        Uniform return with structured data describing the transition.

        Success:
            success=True, climbed=True, delta_y>=min_delta_y

        Failure:
            success=False
            failure_reason in {
                "collision",
                "fell",
                "observation_failed",
                "support_ambiguous",
                "not_elevated"
            }
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    step_duration = float(kwargs.get("step_duration", 0.6))
    allow_walkable_landing = bool(kwargs.get("allow_walkable_landing", True))
    min_delta_y = float(kwargs.get("min_delta_y", 0.9))
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL

    # --- Capture starting status ---
    status_before = executor.execute_action_with_log({"type": "mc-status"}, "nav-climb")
    if status_before.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain initial status",
            reason="status_failed"
        )

    start_pos = status_before["data"]["position"]
    start_y = start_pos.get("y")
    current_yaw = status_before["data"].get("yaw")  # Get current yaw for preservation if no movement

    # --- Ensure grid-aligned before movement (block center, cardinal yaw, pitch 0) ---
    normalized_pose, normalized_yaw = ensure_grid_aligned(executor, minecraft_url, status_data=status_before["data"])
    # Update start_pos and current_yaw to normalized values for consistency
    start_pos = normalized_pose
    current_yaw = normalized_yaw

    def _grid_precheck(position: Dict, yaw: float, *, is_jump: bool) -> Dict:
        """
        Use session-local grid to pre-check climb clearance/support.
        Returns dict with:
          - blockers: [{x,y,z,name}]
          - unknown:  [{x,y,z}]
          - support: {"x","y","z","name","known","ok"}
        """
        if get_cell_name is None or is_air_name is None:
            return {"available": False}
        try:
            bx = int(math.floor(position.get("x", 0.0)))
            by = int(math.floor(position.get("y", 0.0)))
            bz = int(math.floor(position.get("z", 0.0)))
            dx = -math.sin(math.radians(yaw))
            dz = -math.cos(math.radians(yaw))
            fwd_x = bx + int(round(dx))
            fwd_z = bz + int(round(dz))

            # For a +1 climb, target support block is at (fwd_x, by, fwd_z); agent aims to occupy y=by+1.
            checks = [
                ("dest_body", fwd_x, by + 1, fwd_z),
                ("dest_head", fwd_x, by + 2, fwd_z),
                ("src_body", bx, by + 1, bz),
                ("src_head", bx, by + 2, bz),
            ]
            if is_jump:
                checks.append(("dest_arc", fwd_x, by + 3, fwd_z))

            blockers = []
            unknown = []
            for _, x, y, z in checks:
                name = get_cell_name(executor, x, y, z)
                if name is None:
                    unknown.append({"x": x, "y": y, "z": z})
                elif not is_air_name(name):
                    blockers.append({"x": x, "y": y, "z": z, "name": name})

            sup_name = get_cell_name(executor, fwd_x, by, fwd_z)
            support_known = sup_name is not None
            support_ok = bool(sup_name) and (not is_air_name(sup_name))
            support = {"x": fwd_x, "y": by, "z": fwd_z, "name": sup_name, "known": support_known, "ok": support_ok}

            return {"available": True, "blockers": blockers, "unknown": unknown, "support": support}
        except Exception:
            return {"available": False}

    def _grid_orientation_candidates(position: Dict, *, is_jump: bool) -> Dict:
        """
        Fast scan of the 4 cardinal yaws to find orientations where the *known* grid
        suggests climb might work. Intended for: turn+observe to fill blind areas.
        """
        if get_cell_name is None or is_air_name is None:
            return {"available": False}
        candidates_known = []
        candidates_possible = []
        for yaw in (0.0, 90.0, 180.0, 270.0):
            pre = _grid_precheck(position, yaw, is_jump=is_jump)
            if not pre.get("available"):
                continue
            support = pre.get("support", {}) if isinstance(pre.get("support"), dict) else {}
            blockers = pre.get("blockers", []) if isinstance(pre.get("blockers"), list) else []
            unknown = pre.get("unknown", []) if isinstance(pre.get("unknown"), list) else []

            # "Known candidate": support is known+ok and there are no known blockers in clearance spaces.
            if support.get("known") and support.get("ok") and not blockers:
                candidates_known.append(int(yaw))
            # "Possible candidate": we don't know it's blocked; turning+observe may clarify.
            if not blockers and (unknown or not support.get("known")):
                candidates_possible.append(int(yaw))
        return {"available": True, "known": sorted(list(set(candidates_known))), "possible": sorted(list(set(candidates_possible)))}

    # Mandatory behavior: observe from all 4 cardinals, then choose and attempt climb.
    if get_cell_name is None or is_air_name is None:
        return executor._create_uniform_return(
            "failed",
            value="Climb aborted: local_grid unavailable",
            reason="local_grid_unavailable",
            extra={"delta": _to_relative_delta(start_pos, None)},
        )

    cardinals = (0.0, 90.0, 180.0, 270.0)
    for yaw in cardinals:
        # Snap yaw/pitch (no translation), then observe to populate local_grid.
        try:
            snap_to_position(minecraft_url, x=start_pos["x"], y=start_pos["y"], z=start_pos["z"], yaw=yaw, pitch=0.0)
        except Exception:
            pass
        executor.execute_action_with_log({"type": "mc-observe"}, f"nav-climb:probe:{int(yaw)}")

    chosen_yaw = None
    # Choose first yaw where the grid indicates a fully-known climb is feasible.
    for yaw in cardinals:
        pre_walk = _grid_precheck(start_pos, yaw, is_jump=False)
        pre_jump = _grid_precheck(start_pos, yaw, is_jump=True)

        def _is_climbable(pre: Dict) -> bool:
            if not pre.get("available"):
                return False
            support = pre.get("support", {}) if isinstance(pre.get("support"), dict) else {}
            blockers = pre.get("blockers", []) if isinstance(pre.get("blockers"), list) else []
            unknown = pre.get("unknown", []) if isinstance(pre.get("unknown"), list) else []
            return bool(support.get("known")) and bool(support.get("ok")) and (not blockers) and (not unknown)

        if _is_climbable(pre_walk) or _is_climbable(pre_jump):
            chosen_yaw = float(yaw)
            break

    if chosen_yaw is None:
        return executor._create_uniform_return(
            "failed",
            value="Climb aborted: no climbable cardinal orientation found (after 4-way observe)",
            reason="no_climbable_orientation",
            extra={"delta": _to_relative_delta(start_pos, None), "grid_orient": _grid_orientation_candidates(start_pos, is_jump=True)},
        )

    # Realign to chosen yaw and enforce invariants before attempting motion.
    try:
        snap_to_position(minecraft_url, x=start_pos["x"], y=start_pos["y"], z=start_pos["z"], yaw=chosen_yaw, pitch=0.0)
    except Exception:
        pass
    status_after_turn = executor.execute_action_with_log({"type": "mc-status"}, "nav-climb:chosen_yaw")
    if status_after_turn.get("status") == "success":
        start_pos, current_yaw = ensure_grid_aligned(executor, minecraft_url, status_data=status_after_turn.get("data", {}))

    def attempt(move_kwargs, label):
        jump = move_kwargs.get("jump", False)

        # Pre-check using local_grid: if it already knows blockers in required clearance spaces,
        # dig them before attempting the move (avoids guaranteed collisions).
        grid_pre = _grid_precheck(start_pos, current_yaw, is_jump=jump)
        if grid_pre.get("available") and isinstance(grid_pre.get("blockers"), list) and grid_pre["blockers"]:
            # Bounded: don't over-dig; aim for first-order clearance.
            for b in grid_pre["blockers"][:4]:
                try:
                    executor.execute_action_with_log(
                        {"type": "mc-dig", "x": float(b["x"]), "y": float(b["y"]), "z": float(b["z"])},
                        f"nav-climb:{label}:grid_pre_dig"
                    )
                    time.sleep(0.2)
                except Exception:
                    pass

        move_data = _call_move_endpoint(minecraft_url, forward=True, duration=step_duration, jump=jump, check_collision=True)
        if move_data is None or not move_data.get("ok"):
            return {"ok": False, "reason": "move_failed"}

        move_status = move_data.get("status", "success")
        if move_status == "collision":
            # Observe to expose clearance facts (like nav-move does)
            obs = executor.execute_action_with_log({"type": "mc-observe"}, f"nav-climb:{label}:collision")
            clear = obs.get("data", {}).get("clear", {}) if obs.get("status") == "success" else {}
            dirs = obs.get("data", {}).get("dirs", {}) if obs.get("status") == "success" else {}
            nav_surface = obs.get("data", {}).get("nav_surface", []) if obs.get("status") == "success" else []
            # Use current position from observation (not stale start_pos) for accurate recovery
            # The observation happens after collision, so it reflects the actual current position
            obs_data = obs.get("data", {}) if obs.get("status") == "success" else {}
            current_pos = obs_data.get("pose", start_pos) if isinstance(obs_data.get("pose"), dict) else start_pos
            current_yaw_from_obs = current_pos.get("yaw") if isinstance(current_pos, dict) and "yaw" in current_pos else current_yaw
            # Fallback to start_pos if observation doesn't have valid pose
            if not isinstance(current_pos, dict) or "x" not in current_pos:
                current_pos = start_pos
                logger.warning(f"nav-climb: Observation missing pose, using start_pos for recovery")
            diagnostics = {
                "clear_fwd_body": clear.get("fwd", {}).get("body"),
                "clear_fwd_head": clear.get("fwd", {}).get("head"),
                "clear_up_body": clear.get("up", {}).get("body"),
                "clear_up_head": clear.get("up", {}).get("head"),
                "up_block": dirs.get("up", {}).get("blk"),
                "fwd_block": dirs.get("fwd", {}).get("blk"),
                "nav_surface": nav_surface,
            }
            
            # Attempt recovery: dig obstructing blocks (use current position, not stale start_pos)
            recovery_attempted = _attempt_collision_recovery(executor, current_pos, current_yaw_from_obs, diagnostics, minecraft_url, is_jump=jump)
            
            if recovery_attempted:
                # Retry climb after recovery
                logger.info(f"nav-climb: Retrying after recovery dig")
                move_data_retry = _call_move_endpoint(minecraft_url, forward=True, duration=step_duration, jump=jump, check_collision=True)
                if move_data_retry and move_data_retry.get("ok") and move_data_retry.get("status") == "success":
                    # Recovery succeeded, continue with normal flow
                    move_status = "success"
                    move_data = move_data_retry
                else:
                    # Recovery failed, return collision
                    return {
                        "ok": False,
                        "reason": "collision",
                        "diagnostics": diagnostics,
                        "recovery_attempted": True,
                    }
            else:
                # No recovery attempted, return collision
                return {
                    "ok": False,
                    "reason": "collision",
                    "diagnostics": diagnostics,
                    "recovery_attempted": False,
                }
        if move_status == "fell":
            return {"ok": False, "reason": "fell"}

        # Observe
        obs = executor.execute_action_with_log({"type": "mc-observe"}, f"nav-climb:{label}")
        if obs.get("status") != "success":
            return {"ok": False, "reason": "observation_failed"}

        # Status after
        status_after = executor.execute_action_with_log({"type": "mc-status"}, f"nav-climb:{label}")
        if status_after.get("status") != "success":
            return {"ok": False, "reason": "observation_failed"}

        end_pos = status_after["data"]["position"]
        end_y = end_pos.get("y")
        delta_y = None
        if isinstance(start_y, (int, float)) and isinstance(end_y, (int, float)):
            delta_y = end_y - start_y

        support_here = obs.get("data", {}).get("support", {}).get("here", {})
        support_type = support_here.get("type", "unknown")

        return {
            "ok": True,
            "end_pos": end_pos,
            "delta_y": delta_y,
            "support_here": support_type,
        }

    # --- Attempt A: walk-up ---
    res = attempt({}, "walk")
    if res.get("ok"):
        dy = res.get("delta_y")
        if dy is not None and dy >= min_delta_y:
            if allow_walkable_landing or res.get("support_here") == "solid":
                end_pos = res.get("end_pos")
                # Snap to block center (preserves current yaw)
                if end_pos:
                    snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
                    _snap_to_position(executor, snap_pos, minecraft_url)
                    end_pos = snap_pos
                    try:
                        if set_center_from_pose is not None:
                            set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
                    except Exception:
                        pass
                    
                    # Update nav state (successful climb) - preserve current_yaw
                    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
                    _update_nav_state(executor, pose, res.get("support_here", "unknown"), fell=False, was_fall=False)
                
                return executor._create_uniform_return(
                    "success",
                    value=f"Climbed successfully via walk (delta_y={dy:.2f})",
                    extra={
                        "delta": _to_relative_delta(start_pos, end_pos),
                        "delta_y": dy,
                        "mode": "walk",
                        "support_here": res.get("support_here"),
                    },
                )

    # --- Attempt B: jump-up ---
    res = attempt({"jump": True}, "jump")
    if res.get("ok"):
        dy = res.get("delta_y")
        if dy is not None and dy >= min_delta_y:
            if allow_walkable_landing or res.get("support_here") == "solid":
                end_pos = res.get("end_pos")
                # Snap to block center (preserves current yaw)
                if end_pos:
                    snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
                    _snap_to_position(executor, snap_pos, minecraft_url)
                    end_pos = snap_pos
                    try:
                        if set_center_from_pose is not None:
                            set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
                    except Exception:
                        pass
                    
                    # Update nav state (successful climb) - preserve current_yaw
                    pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
                    _update_nav_state(executor, pose, res.get("support_here", "unknown"), fell=False, was_fall=False)
                
                return executor._create_uniform_return(
                    "success",
                    value=f"Climbed successfully via jump (delta_y={dy:.2f})",
                    extra={
                        "delta": _to_relative_delta(start_pos, end_pos),
                        "delta_y": dy,
                        "mode": "jump",
                        "support_here": res.get("support_here"),
                    },
                )

    # --- Failure classification ---
    reason = res.get("reason", "not_elevated")
    diagnostics = res.get("diagnostics") if isinstance(res, dict) else None
    end_pos = res.get("end_pos") if isinstance(res, dict) else None
    
    # Snap to block center even on failure if we have a position (preserves current yaw)
    if end_pos:
        snap_pos, _, _ = _calculate_snap_position_and_yaw(start_pos, end_pos, current_yaw)
        _snap_to_position(executor, snap_pos, minecraft_url)
        end_pos = snap_pos
        try:
            if set_center_from_pose is not None:
                set_center_from_pose(executor, {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"]})
        except Exception:
            pass
        
        # Update nav state (climb failure) - preserve current_yaw
        pose = {"x": end_pos["x"], "y": end_pos["y"], "z": end_pos["z"], "yaw": current_yaw}
        support_here = res.get("support_here", "unknown") if isinstance(res, dict) else "unknown"
        _update_nav_state(executor, pose, support_here, fell=False, was_fall=False)
    
    extra = {"delta": _to_relative_delta(start_pos, None)}
    if end_pos:
        extra["to"] = end_pos
    if diagnostics:
        extra["diagnostics"] = diagnostics
    # On failure, suggest orientations to try (turn + mc-observe) based on local_grid
    probe_pos = end_pos if isinstance(end_pos, dict) else start_pos
    extra["grid_orient"] = _grid_orientation_candidates(probe_pos, is_jump=True)
    
    return executor._create_uniform_return(
        "failed",
        value=f"Climb failed: {reason}",
        reason=reason,
        extra=extra,
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-climb v2 module loaded")
