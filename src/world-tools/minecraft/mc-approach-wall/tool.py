"""
Minecraft approach-wall tool.
Bring the agent into stable adjacency with a solid vertical surface.
"""

import logging
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


def _check_wall_forward(obs_data: Dict) -> bool:
    """
    Check if a solid block is detected directly forward at foot or eye level.
    
    Args:
        obs_data: Observation data from mc-observe-blocks
        
    Returns:
        True if solid block detected forward, False otherwise
    """
    dirs_info = obs_data.get("dirs", {})
    fwd_info = dirs_info.get("fwd", {})
    fwd_blk = fwd_info.get("blk")
    
    # Check if forward block exists and is not None/air
    if fwd_blk and fwd_blk != "none" and fwd_blk != "air":
        return True
    
    return False


def tool(input_value=None, **kwargs):
    """
    Approach a wall by advancing safely until a solid vertical surface is encountered.
    
    Args:
        input_value: ignored
        max_steps: integer maximum steps for each advance attempt (default: 10)
        step_duration: float seconds per step for advance (default: 0.2)
        placement_item: item/block name for stabilization (default: "dirt")
        
    Returns:
        Dict with result using uniform return format.
        Outcome values: "SUCCESS", "NO_WALL_REACHABLE", "SUPPORT_UNSTABILIZABLE", "FALL_DETECTED"
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    # Parse parameters
    max_steps = int(kwargs.get("max_steps", 10))
    step_duration = float(kwargs.get("step_duration", 0.2))
    placement_item = kwargs.get("placement_item", "dirt")
    
    # Initialize
    log_messages = []
    logger.info(f"mc-approach-wall: Starting wall approach (max_steps={max_steps}, step_duration={step_duration}s)")
    
    # Step 1: INITIAL SETUP - Fix current heading
    # Get initial status to capture heading
    status_result = executor.execute_action_with_log({"type": "mc-status"}, "mc-approach-wall")
    if status_result.get("status") != "success":
        return executor._create_uniform_return('failed', reason="Failed to get initial status")
    
    status_data = status_result.get("data", {})
    initial_yaw = status_data.get("yaw", 0.0)
    log_messages.append(f"Fixed heading: yaw={initial_yaw:.2f}°")
    
    # Main loop
    while True:
        # Step 2.1: OBSERVE
        obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-approach-wall")
        
        if obs_result.get("status") != "success":
            # Observation failed - treat as no wall reachable
            outcome = "NO_WALL_REACHABLE"
            result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (observation failed)"
            logger.warning(f"mc-approach-wall: {result_text}")
            log_messages.append("Observation failed")
            break
        
        obs_data = obs_result.get("data", {})
        
        # Check if solid block is directly forward
        if _check_wall_forward(obs_data):
            outcome = "SUCCESS"
            result_text = f"Approach-wall completed: SUCCESS (wall detected forward)"
            logger.info(f"mc-approach-wall: {result_text}")
            log_messages.append("Wall detected forward - SUCCESS")
            break
        
        # Step 2.2: ADVANCE SAFELY
        advance_result = executor.execute_action_with_log(
            {"type": "mc-advance", "direction": "forward", "max_steps": max_steps, "step_duration": step_duration},
            "mc-approach-wall"
        )
        
        if advance_result.get("status") != "success":
            # Advance request failed
            reason = advance_result.get("reason", "unknown error")
            outcome = "NO_WALL_REACHABLE"
            result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (advance failed: {reason})"
            logger.warning(f"mc-approach-wall: {result_text}")
            log_messages.append(f"Advance failed: {reason}")
            break
        
        advance_data = advance_result.get("data", {})
        advance_outcome = advance_data.get("outcome", "UNKNOWN")
        advance_steps = advance_data.get("steps_completed", 0)
        
        log_messages.append(f"Advance outcome: {advance_outcome} ({advance_steps} steps)")
        
        # INTERPRET RESULT
        
        if advance_outcome == "BLOCKED":
            # A blocking surface exists at current heading
            # Treat as potential wall adjacency - re-run OBSERVE once
            logger.info("mc-approach-wall: BLOCKED - re-observing to verify wall")
            obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-approach-wall")
            
            if obs_result.get("status") != "success":
                outcome = "NO_WALL_REACHABLE"
                result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (blocked but observation failed)"
                logger.warning(f"mc-approach-wall: {result_text}")
                log_messages.append("Blocked but observation failed")
                break
            
            obs_data = obs_result.get("data", {})
            if _check_wall_forward(obs_data):
                outcome = "SUCCESS"
                result_text = f"Approach-wall completed: SUCCESS (wall detected after collision)"
                logger.info(f"mc-approach-wall: {result_text}")
                log_messages.append("Wall detected after collision - SUCCESS")
                break
            else:
                outcome = "NO_WALL_REACHABLE"
                result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (blocked but no wall detected)"
                logger.warning(f"mc-approach-wall: {result_text}")
                log_messages.append("Blocked but no wall detected")
                break
        
        elif advance_outcome == "FELL":
            outcome = "FALL_DETECTED"
            result_text = f"Approach-wall terminated: FALL_DETECTED (fall occurred during advance)"
            logger.error(f"mc-approach-wall: {result_text}")
            log_messages.append("Fall detected - FALL_DETECTED")
            break
        
        elif advance_outcome == "SUPPORT_AMBIGUOUS":
            # Forward progress degraded footing before wall contact
            # Call MC-PLACE-UNTIL-SUPPORTED
            logger.info("mc-approach-wall: SUPPORT_AMBIGUOUS - attempting stabilization")
            place_result = executor.execute_action_with_log(
                {"type": "mc-place-until-supported", "item": placement_item, "placement_policy": "underfoot", "max_attempts": 3},
                "mc-approach-wall"
            )
            
            if place_result.get("status") != "success":
                outcome = "SUPPORT_UNSTABILIZABLE"
                result_text = f"Approach-wall terminated: SUPPORT_UNSTABILIZABLE (stabilization failed)"
                logger.warning(f"mc-approach-wall: {result_text}")
                log_messages.append("Stabilization failed - SUPPORT_UNSTABILIZABLE")
                break
            
            place_data = place_result.get("data", {})
            place_outcome = place_data.get("outcome", "UNKNOWN")
            
            log_messages.append(f"Stabilization outcome: {place_outcome}")
            
            if place_outcome != "SUPPORTED":
                outcome = "SUPPORT_UNSTABILIZABLE"
                result_text = f"Approach-wall terminated: SUPPORT_UNSTABILIZABLE (stabilization outcome: {place_outcome})"
                logger.warning(f"mc-approach-wall: {result_text}")
                break
            
            # Stabilization succeeded - continue loop
            log_messages.append("Stabilization succeeded - continuing")
            continue
        
        elif advance_outcome == "BOUND_REACHED":
            # Safe advance completed without encountering wall
            # Re-run OBSERVE
            logger.info("mc-approach-wall: BOUND_REACHED - re-observing")
            obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-approach-wall")
            
            if obs_result.get("status") != "success":
                outcome = "NO_WALL_REACHABLE"
                result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (bound reached but observation failed)"
                logger.warning(f"mc-approach-wall: {result_text}")
                log_messages.append("Bound reached but observation failed")
                break
            
            obs_data = obs_result.get("data", {})
            if _check_wall_forward(obs_data):
                outcome = "SUCCESS"
                result_text = f"Approach-wall completed: SUCCESS (wall detected after bound reached)"
                logger.info(f"mc-approach-wall: {result_text}")
                log_messages.append("Wall detected after bound reached - SUCCESS")
                break
            else:
                outcome = "NO_WALL_REACHABLE"
                result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (bound reached, no wall detected)"
                logger.warning(f"mc-approach-wall: {result_text}")
                log_messages.append("Bound reached, no wall detected")
                break
        
        else:
            # Unknown advance outcome - treat as failure
            outcome = "NO_WALL_REACHABLE"
            result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (unknown advance outcome: {advance_outcome})"
            logger.warning(f"mc-approach-wall: {result_text}")
            log_messages.append(f"Unknown advance outcome: {advance_outcome}")
            break
    
    # Build result text with log summary
    if log_messages:
        result_text += "\n" + "\n".join(log_messages)
    
    # Build structured data dict
    structured_data = {
        "outcome": outcome,
        "max_steps": max_steps,
        "step_duration": step_duration,
        "placement_item": placement_item,
        "log": log_messages
    }
    
    return executor._create_uniform_return('success', value=result_text, data=structured_data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(max_steps=10, step_duration=0.2, placement_item="dirt")
    print(result)

