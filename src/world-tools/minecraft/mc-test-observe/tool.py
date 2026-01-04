"""
Minecraft test-observe tool.
Tests movement prediction accuracy by comparing observation-based predictions with actual movement results.
"""

import logging
import math
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def can_move_forward(obs_data: Dict, direction: str = "fwd") -> Tuple[bool, Dict]:
    """
    Determine if forward movement is feasible based on observation data.
    
    Args:
        obs_data: Observation data dict from mc-observe-blocks
        direction: Direction to check ('fwd', 'left', 'right', 'back') - defaults to 'fwd'
                  Note: After rotation, 'fwd' represents the desired movement direction
        
    Returns:
        Tuple of (success: bool, diagnostics: dict) with detailed condition checks
    """
  
    support = obs_data.get('support', {})
    clear = obs_data.get('clear', {})
    
    # After rotation, we always check 'fwd' relative to current orientation
    # (the observation is taken after rotating to face the desired direction)
    fwd_support = support.get('fwd', {})
    fwd_clear = clear.get('fwd', {})
    
    # Check conditions for movement (without jumping):
    # Movement requires clearance at all three levels:
    # 1. Clearance at feet level (forward:1, up:0) - required for non-jumping movement
    # 2. Clearance at body level (forward:1, up:1)
    # 3. Clearance at head level (forward:1, up:2)
    # 
    # Note: Solid footing is NOT required - player can move forward over gaps/air
    # as long as there's clearance for the player's hitbox.
    
    # Get support info for diagnostics (not used for prediction)
    footing_block = fwd_support.get('block', 'unknown')
    footing_type = fwd_support.get('type', 'unknown')
    
    # Note: mc-observe-blocks checks clearance at up:1 (body) and up:2 (head)
    # We need to also check up:0 (feet), but the observation doesn't provide this directly
    # For now, we'll use the existing checks and note the limitation
    has_body_clearance = fwd_clear.get('body', False)
    has_head_clearance = fwd_clear.get('head', False)
    
    # TODO: mc-observe-blocks should check clearance at up:0 (feet level)
    # For now, assume feet clearance if body clearance exists (conservative)
    # This is a limitation - we can't fully verify feet clearance from current observation
    has_feet_clearance = has_body_clearance  # Approximation until observation includes up:0
    
    # Movement feasible if ALL clearance conditions met (footing not required)
    success = has_feet_clearance and has_body_clearance and has_head_clearance
    
    diagnostics = {
        "direction": direction,
        "footing_block": footing_block,
        "footing_type": footing_type,
        "has_feet_clearance": has_feet_clearance,
        "has_body_clearance": has_body_clearance,
        "has_head_clearance": has_head_clearance,
        "all_conditions_met": success,
        "note": "feet_clearance approximated from body_clearance (observation limitation); footing not required"
    }
    
    return success, diagnostics


def _get_position(status_data: Dict) -> Dict[str, float]:
    """Extract position dict from status data."""
    position = status_data.get("position", {})
    if isinstance(position, (list, tuple)) and len(position) >= 3:
        return {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])}
    elif isinstance(position, dict):
        return {
            "x": float(position.get("x", 0)),
            "y": float(position.get("y", 0)),
            "z": float(position.get("z", 0))
        }
    return {"x": 0.0, "y": 0.0, "z": 0.0}


def _position_changed(pos1: Dict[str, float], pos2: Dict[str, float], threshold: float = 0.3) -> bool:
    """Check if position changed significantly (movement occurred)."""
    dx = abs(pos2["x"] - pos1["x"])
    dy = abs(pos2["y"] - pos1["y"])
    dz = abs(pos2["z"] - pos1["z"])
    # Movement occurred if any axis changed by more than threshold
    return dx > threshold or dy > threshold or dz > threshold


def _find_nearest_y0_block(obs_data: Dict, position: Dict[str, float], max_distance: float = 10.0) -> Optional[Tuple[int, int, int]]:
    """
    Find nearest block at y=0 (or near y=0) from observation data.
    
    Args:
        obs_data: Observation data dict from mc-observe-blocks
        position: Current position dict with x, y, z
        max_distance: Maximum horizontal distance to search
        
    Returns:
        Tuple of (x, y, z) block coordinates, or None if not found
    """
    blocks_info = obs_data.get('blocks', {})
    nearby_blocks = blocks_info.get('nearby', [])
    
    if not nearby_blocks:
        return None
    
    px = position.get('x', 0)
    py = position.get('y', 0)
    pz = position.get('z', 0)
    
    nearest_block = None
    nearest_distance = float('inf')
    
    for block in nearby_blocks:
        if not isinstance(block, dict):
            continue
        
        pos = block.get('position')
        if not pos:
            continue
        
        # Extract block position
        if isinstance(pos, dict):
            bx = pos.get('x', 0)
            by = pos.get('y', 0)
            bz = pos.get('z', 0)
        elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
            bx = pos[0]
            by = pos[1]
            bz = pos[2]
        else:
            continue
        
        # Check if block is at or near y=0
        if abs(by) > 1.0:  # Only consider blocks near y=0
            continue
        
        # Calculate horizontal distance
        dx = bx - px
        dz = bz - pz
        horizontal_dist = math.sqrt(dx*dx + dz*dz)
        
        if horizontal_dist <= max_distance and horizontal_dist < nearest_distance:
            nearest_distance = horizontal_dist
            nearest_block = (int(math.floor(bx)), int(math.floor(by)), int(math.floor(bz)))
    
    return nearest_block


def _calculate_yaw_to_position(from_pos: Dict[str, float], to_pos: Tuple[int, int, int]) -> float:
    """
    Calculate yaw (in degrees) to face a target position.
    
    Args:
        from_pos: Current position dict with x, y, z
        to_pos: Target block position tuple (x, y, z)
        
    Returns:
        Yaw in degrees (0-360)
    """
    dx = to_pos[0] - from_pos['x']
    dz = to_pos[2] - from_pos['z']
    
    # Calculate yaw: atan2(-dx, -dz) gives Minecraft yaw convention
    # 0° = south (positive Z), 90° = west (negative X), 180° = north (negative Z), 270° = east (positive X)
    yaw_rad = math.atan2(-dx, -dz)
    yaw_deg = math.degrees(yaw_rad)
    
    # Normalize to 0-360
    yaw_deg = yaw_deg % 360
    if yaw_deg < 0:
        yaw_deg += 360
    
    return yaw_deg


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


def tool(input_value=None, **kwargs):
    """
    Test movement prediction by comparing observation-based predictions with actual movement.
    
    For each direction (fwd, left, right, back):
    1. Determines movement feasibility from observation
    2. Logs the prediction
    3. Attempts actual movement
    4. Verifies position change
    5. Compares result with prediction
    6. Restores original position and orientation
    
    Args:
        input_value: ignored
        
    Returns:
        Dict with test results using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    log = []
    logger.info("mc-test-observe: Testing movement prediction accuracy")
    log.append("mc-test-observe: Testing movement prediction accuracy")
    log.append("")
    
    # Get initial observation and status
    obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-test-observe")
    if obs_result.get("status") != "success":
        return executor._create_uniform_return('failed', reason="Failed to get observation")
    
    obs_data = obs_result.get("data", {})
    if not obs_data:
        return executor._create_uniform_return('failed', reason="Observation data missing")
    
    status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-test-observe")
    if status_result.get("status") != "success":
        return executor._create_uniform_return('failed', reason="Failed to get status")
    
    status_data = status_result.get("data", {})
    original_yaw = status_data.get("yaw", 0.0)
    original_pitch = status_data.get("pitch", 0.0)
    original_position = _get_position(status_data)
    
    # Directions to test: (name, yaw_offset, move_params)
    directions = [
        ("fwd", 0, {"forward": True}),
        ("left", 90, {"left": True}),
        ("right", -90, {"right": True}),
        ("back", 180, {"back": True}),
    ]
    
    results = []
    
    for dir_name, yaw_offset, move_params in directions:
        log.append(f"--- Testing {dir_name.upper()} direction ---")
        
        # Rotate if needed
        if yaw_offset != 0:
            target_yaw = (original_yaw + yaw_offset) % 360
            look_result = executor.execute_action_with_log(
                {"type": "mc-look", "yaw": target_yaw, "pitch": original_pitch},
                "mc-test-observe"
            )
            if look_result.get("status") != "success":
                logger.error(f"{dir_name}: rotation failed")
                log.append(f"  ERROR: Failed to rotate")
                results.append({"direction": dir_name, "error": "rotation failed"})
                continue
        
        # Get position before movement
        status_before = executor.execute_action_with_log({"type": "mc-status"}, "mc-test-observe")
        if status_before.get("status") != "success":
            logger.error(f"{dir_name}: status check failed")
            log.append(f"  ERROR: Status check failed")
            results.append({"direction": dir_name, "error": "status check failed"})
            continue
        
        position_before = _get_position(status_before.get("data", {}))
        
        # Get fresh observation for this orientation
        obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-test-observe")
        if obs_result.get("status") != "success":
            logger.error(f"{dir_name}: observation failed")
            log.append(f"  ERROR: Observation failed")
            results.append({"direction": dir_name, "error": "observation failed"})
            continue
        
        obs_data = obs_result.get("data", {})
        
        # Determine movement feasibility
        predicted_success, diagnostics = can_move_forward(obs_data, dir_name)
        
        # Attempt actual movement
        move_params_with_duration = {**move_params, "duration": 0.5}
        move_result = executor.execute_action_with_log(
            {"type": "mc-move", **move_params_with_duration},
            "mc-test-observe"
        )
        executor.execute_action_with_log({"type": "mc-stop"}, "mc-test-observe")
        
        # Get position after movement
        status_after = executor.execute_action_with_log({"type": "mc-status"}, "mc-test-observe")
        if status_after.get("status") != "success":
            logger.error(f"{dir_name}: status check after movement failed")
            log.append(f"  ERROR: Status check after movement failed")
            results.append({"direction": dir_name, "error": "status check failed"})
            continue
        
        position_after = _get_position(status_after.get("data", {}))
        actual_success = _position_changed(position_before, position_after)
        move_data = move_result.get("data", {})
        move_status = move_data.get("status", "unknown")
        
        position_delta = {
            "x": position_after["x"] - position_before["x"],
            "y": position_after["y"] - position_before["y"],
            "z": position_after["z"] - position_before["z"]
        }
        
        matches = predicted_success == actual_success
        
        # Essential logging: prediction vs actual result
        logger.info(f"{dir_name}: predicted={'SUCCESS' if predicted_success else 'FAIL'}, actual={'SUCCESS' if actual_success else 'FAIL'} ({move_status}), {'MATCH' if matches else 'MISMATCH'}")
        
        log.append(f"  Prediction: {'SUCCESS' if predicted_success else 'FAIL'}")
        log.append(f"  Actual: {'SUCCESS' if actual_success else 'FAIL'} ({move_status}, delta={position_delta})")
        log.append(f"  Result: {'MATCH' if matches else 'MISMATCH'}")
        
        results.append({
            "direction": dir_name,
            "predicted": predicted_success,
            "actual": actual_success,
            "move_status": move_status,
            "position_before": position_before,
            "position_after": position_after,
            "position_delta": position_delta,
            "matches": matches,
            "diagnostics": diagnostics
        })
        
        # Restore position and orientation
        if yaw_offset != 0:
            executor.execute_action_with_log(
                {"type": "mc-look", "yaw": original_yaw, "pitch": original_pitch},
                "mc-test-observe"
            )
        
        if actual_success:
            # Move back in opposite direction
            reverse_params = {}
            if dir_name == "fwd":
                reverse_params = {"back": True}
            elif dir_name == "left":
                reverse_params = {"right": True}
            elif dir_name == "right":
                reverse_params = {"left": True}
            elif dir_name == "back":
                reverse_params = {"forward": True}
            
            if reverse_params:
                reverse_params["duration"] = 0.5
                executor.execute_action_with_log(
                    {"type": "mc-move", **reverse_params},
                    "mc-test-observe"
                )
                executor.execute_action_with_log({"type": "mc-stop"}, "mc-test-observe")
        
        log.append("")
    
    # After 4-way test: Orient to face nearest y=0 block at cardinal direction
    log.append("--- Post-test orientation ---")
    status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-test-observe")
    if status_result.get("status") == "success":
        status_data = status_result.get("data", {})
        current_position = _get_position(status_data)
        
        obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-test-observe")
        if obs_result.get("status") == "success":
            obs_data = obs_result.get("data", {})
            nearest_y0_block = _find_nearest_y0_block(obs_data, current_position)
            if nearest_y0_block:
                target_yaw = _calculate_yaw_to_position(current_position, nearest_y0_block)
                cardinal_yaw = _round_to_cardinal(target_yaw)
                logger.info(f"Orienting to face nearest y=0 block at {nearest_y0_block}, cardinal yaw={cardinal_yaw}°")
                log.append(f"Orienting to cardinal yaw={cardinal_yaw}° (facing nearest y=0 block at {nearest_y0_block})")
                
                look_result = executor.execute_action_with_log(
                    {"type": "mc-look", "yaw": cardinal_yaw, "pitch": 0},
                    "mc-test-observe"
                )
                if look_result.get("status") != "success":
                    logger.error("Failed to orient to cardinal direction")
                    log.append("  ERROR: Failed to orient to cardinal direction")
                else:
                    log.append(f"  Successfully oriented to {cardinal_yaw}°")
            else:
                logger.info("No y=0 block found nearby, keeping current orientation")
                log.append("No y=0 block found nearby, keeping current orientation")
        else:
            log.append("  WARNING: Failed to get observation for orientation")
    else:
        log.append("  WARNING: Failed to get status for orientation")
    
    log.append("")
    
    # Summary
    log.append("--- Summary ---")
    matches_count = sum(1 for r in results if r.get("matches", False))
    total_count = len(results)
    logger.info(f"Summary: {matches_count}/{total_count} predictions matched")
    log.append(f"Predictions matching actual: {matches_count}/{total_count}")
    
    for r in results:
        if "error" in r:
            log.append(f"  {r['direction']}: ERROR - {r['error']}")
        else:
            match_str = "✓" if r.get("matches") else "✗"
            log.append(f"  {r['direction']}: {match_str} predicted={r['predicted']}, actual={r['actual']} (delta={r.get('position_delta', {})})")
    
    summary_text = "\n".join(log)
    
    # Build structured data
    structured_data = {
        "results": results,
        "matches_count": matches_count,
        "total_count": total_count,
        "original_position": original_position,
        "original_yaw": original_yaw,
        "original_pitch": original_pitch
    }
    
    return executor._create_uniform_return('success', value=summary_text, data=structured_data)

