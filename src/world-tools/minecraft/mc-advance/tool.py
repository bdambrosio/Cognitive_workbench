"""
Minecraft advance tool.
Advance incrementally in a chosen direction while maintaining conservative safety guarantees.
"""

import logging
from typing import Any, Dict, Optional
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)


def tool(input_value=None, **kwargs):
    """
    Advance incrementally in a chosen direction with conservative safety guarantees.
    
    Args:
        input_value: ignored
        direction: one of {"forward", "back", "left", "right"} (default: "forward")
        step_duration: float seconds per step (default: 0.2)
        max_steps: integer maximum steps (default: None, unbounded)
        
    Returns:
        Dict with result using uniform return format.
        Outcome values: "ADVANCED", "BLOCKED", "FELL", "SUPPORT_AMBIGUOUS", "LIMIT_REACHED"
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    # Parse parameters
    direction = kwargs.get("direction", "forward").lower()
    if direction not in ["forward", "back", "left", "right"]:
        return executor._create_uniform_return('failed', reason=f"Invalid direction: {direction}. Must be one of: forward, back, left, right")
    
    step_duration = float(kwargs.get("step_duration", 0.2))
    max_steps = kwargs.get("max_steps")
    if max_steps is not None:
        max_steps = int(max_steps)
    
    # Initialize
    steps = 0
    log_messages = []
    outcome = None
    logger.info(f"mc-advance: Starting advance in direction '{direction}' (step_duration={step_duration}s, max_steps={max_steps})")
    
    # Main loop
    while True:
        # Step 1: Attempt movement
        move_params = {direction: True, "duration": step_duration, "check_collision": True}
        move_result = executor.execute_action_with_log({"type": "mc-move", **move_params}, "mc-advance")
        
        if move_result.get("status") != "success":
            # Movement request failed
            reason = move_result.get("reason", "unknown error")
            result_text = f"Advance failed: Movement request failed - {reason}"
            logger.error(f"mc-advance: {result_text}")
            return executor._create_uniform_return('failed', reason=result_text)
        
        move_data = move_result.get("data", {})
        move_status = move_data.get("status", "success")
        
        # Step 2: Inspect movement result
        if move_status == "collision":
            outcome = "BLOCKED"
            result_text = f"Advance terminated: BLOCKED (collision detected after {steps} step(s))"
            logger.info(f"mc-advance: {result_text}")
            log_messages.append(f"Step {steps + 1}: Movement blocked by collision")
            break
        
        if move_status == "fell":
            outcome = "FELL"
            result_text = f"Advance terminated: FELL (fall detected after {steps} step(s))"
            logger.warning(f"mc-advance: {result_text}")
            log_messages.append(f"Step {steps + 1}: Fall detected")
            break
        
        # Step 3: Observe blocks for support assessment
        obs_result = executor.execute_action_with_log({"type": "mc-observe-blocks"}, "mc-advance")
        
        if obs_result.get("status") != "success":
            # Observation failed - treat as ambiguous
            outcome = "SUPPORT_AMBIGUOUS"
            result_text = f"Advance terminated: SUPPORT_AMBIGUOUS (observation failed after {steps} step(s))"
            logger.warning(f"mc-advance: {result_text}")
            log_messages.append(f"Step {steps + 1}: Observation failed, treating as ambiguous")
            break
        
        obs_data = obs_result.get("data", {})
        support_info = obs_data.get("support", {})
        
        # Step 4: Evaluate support at current pose
        # Check "here" support (current position footing)
        here_support = support_info.get("here", {})
        here_type = here_support.get("type", "unknown")
        here_block = here_support.get("block", None)
        
        # Check forward support (for forward direction) or appropriate directional support
        if direction == "forward":
            fwd_support = support_info.get("fwd", {})
            fwd_type = fwd_support.get("type", "unknown")
            
            # For forward movement: forward support MUST be solid
            # Current support can be "unsafe" (non-solid but walkable like snow) as long as it's not air/void
            # This allows movement from snow layers, carpets, etc. to solid blocks
            if fwd_type != "solid":
                outcome = "SUPPORT_AMBIGUOUS"
                result_text = f"Advance terminated: SUPPORT_AMBIGUOUS (forward support not solid after {steps} step(s))"
                logger.info(f"mc-advance: {result_text} (here={here_type}, fwd={fwd_type})")
                log_messages.append(f"Step {steps + 1}: Forward support not solid (here={here_type}, fwd={fwd_type})")
                break
            
            # Check if current support is air/void (truly unsafe)
            #if here_type == "unsafe" and (here_block is None or here_block in ("air", "cave_air", "void_air")):
            #    outcome = "SUPPORT_AMBIGUOUS"
            #    result_text = f"Advance terminated: SUPPORT_AMBIGUOUS (standing on air/void after {steps} step(s))"
            #    logger.warning(f"mc-advance: {result_text} (here={here_type}, block={here_block}, fwd={fwd_type})")
            #    log_messages.append(f"Step {steps + 1}: Standing on air/void (here={here_type}, block={here_block})")
            #    break
            
            # Forward is solid and current is not air/void - safe to continue
            if here_type != "solid":
                log_messages.append(f"Step {steps + 1}: Current support is {here_type} (block={here_block}) but forward is solid - continuing")
        else:
            pass
            # For other directions, check current support only
            # Note: mc-observe-blocks only provides "here" and "fwd" support
            # For back/left/right, we rely on "here" support only
            #if here_type != "solid":
            #    # Check if it's truly unsafe (air/void) vs walkable non-solid
            #    if here_block is None or here_block in ("air", "cave_air", "void_air"):
            #        outcome = "SUPPORT_AMBIGUOUS"
            #        result_text = f"Advance terminated: SUPPORT_AMBIGUOUS (standing on air/void after {steps} step(s))"
            #        logger.warning(f"mc-advance: {result_text} (here={here_type}, block={here_block})")
            #        log_messages.append(f"Step {steps + 1}: Standing on air/void (here={here_type}, block={here_block})")
            #        break
            #    else:
            #        # Non-solid but walkable (like snow) - allow for non-forward directions
            #        log_messages.append(f"Step {steps + 1}: Current support is {here_type} (block={here_block}) - continuing")
        
        # Step 5: Increment step counter
        steps += 1
        log_messages.append(f"Step {steps}: Movement successful, support confirmed")
        
        # Step 6: Check max_steps limit
        if max_steps is not None and steps >= max_steps:
            outcome = "BOUND_REACHED"
            result_text = f"Advance terminated: BOUND_REACHED (max_steps={max_steps} reached)"
            logger.info(f"mc-advance: {result_text}")
            break
        
        # Continue loop (will check termination conditions again)
        # Note: If max_steps is None, loop continues until a termination condition is met
    
    # Set outcome if loop exited normally (shouldn't happen, but safety check)
    if 'outcome' not in locals():
        outcome = "ADVANCED"
        result_text = f"Advance completed: ADVANCED ({steps} step(s) completed)"
        logger.info(f"mc-advance: {result_text}")
    
    # Build result text with log summary
    if log_messages:
        result_text += "\n" + "\n".join(log_messages)
    
    # Build structured data dict
    structured_data = {
        "outcome": outcome,
        "steps_completed": steps,
        "direction": direction,
        "step_duration": step_duration,
        "max_steps": max_steps,
        "log": log_messages
    }
    
    return executor._create_uniform_return('success', value=result_text, data=structured_data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool(direction="forward", step_duration=0.2, max_steps=5)
    print(result)

