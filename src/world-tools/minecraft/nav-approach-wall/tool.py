"""
Navigation approach-wall composite.

Bring the agent into stable adjacency with a wall, using nav motion tools (nav-advance).
"""

import logging
from typing import Dict

logger = logging.getLogger(__name__)


def _check_wall_forward(obs_data: Dict) -> bool:
    dirs_info = obs_data.get("dirs", {})
    fwd_info = dirs_info.get("fwd", {})
    fwd_blk = fwd_info.get("blk")
    if fwd_blk and fwd_blk not in ("none", "air"):
        return True
    return False


def tool(input_value=None, **kwargs):
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    target = int(kwargs.get("target", 10))
    step_duration = float(kwargs.get("step_duration", 0.2))
    placement_item = kwargs.get("placement_item", "dirt")

    log_messages = []

    # Keep heading fixed but do not force yaw changes here; motion tools operate relative to current facing.
    status_result = executor.execute_action_with_log({"type": "mc-status"}, "nav-approach-wall")
    if status_result.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to get initial status",
            reason="status_failed",
            extra={"outcome": "NO_WALL_REACHABLE"}
        )

    initial_yaw = status_result.get("data", {}).get("yaw", 0.0)
    log_messages.append(f"Fixed heading: yaw={initial_yaw:.2f}°")

    blocks_remaining = target
    outcome = "NO_WALL_REACHABLE"
    result_text = "Approach-wall terminated: NO_WALL_REACHABLE"

    while blocks_remaining > 0:
        obs_result = executor.execute_action_with_log({"type": "mc-observe"}, "nav-approach-wall")
        if obs_result.get("status") != "success":
            log_messages.append("Observation failed")
            break

        obs_data = obs_result.get("data", {})
        if _check_wall_forward(obs_data):
            outcome = "SUCCESS"
            result_text = "Approach-wall completed: SUCCESS (wall detected forward)"
            log_messages.append("Wall detected forward - SUCCESS")
            break

        # Advance some amount; nav-advance is verified and updates nav history.
        adv = executor.execute_action_with_log(
            {"type": "nav-advance", "blocks": blocks_remaining, "step_duration": step_duration},
            "nav-approach-wall",
        )
        if adv.get("status") != "success":
            adv_data = adv.get("data", {}) or {}
            stop_reason = adv_data.get("stop_reason", "UNKNOWN")
            steps_done = adv_data.get("steps_completed", 0)
            log_messages.append(f"nav-advance stop_reason={stop_reason} steps_completed={steps_done}")

            if stop_reason == "FELL":
                outcome = "FALL_DETECTED"
                result_text = "Approach-wall terminated: FALL_DETECTED (fall occurred during advance)"
                log_messages.append("Fall detected - FALL_DETECTED")
                break

            if stop_reason == "SUPPORT_AMBIGUOUS":
                # Attempt stabilization, then continue loop.
                place_result = executor.execute_action_with_log(
                    {"type": "mc-place-until-supported", "item": placement_item, "placement_policy": "underfoot", "max_attempts": 3},
                    "nav-approach-wall",
                )
                if place_result.get("status") != "success":
                    outcome = "SUPPORT_UNSTABILIZABLE"
                    result_text = "Approach-wall terminated: SUPPORT_UNSTABILIZABLE (stabilization failed)"
                    log_messages.append("Stabilization failed - SUPPORT_UNSTABILIZABLE")
                    break

                place_data = place_result.get("data", {}) or {}
                place_outcome = place_data.get("outcome", "UNKNOWN")
                log_messages.append(f"Stabilization outcome: {place_outcome}")
                if place_outcome != "SUPPORTED":
                    outcome = "SUPPORT_UNSTABILIZABLE"
                    result_text = f"Approach-wall terminated: SUPPORT_UNSTABILIZABLE (stabilization outcome: {place_outcome})"
                    break

                log_messages.append("Stabilization succeeded - continuing")
                continue

            # Blocked / move failed / etc: re-observe once to see if wall is in front now.
            obs2 = executor.execute_action_with_log({"type": "mc-observe"}, "nav-approach-wall")
            if obs2.get("status") == "success" and _check_wall_forward(obs2.get("data", {}) or {}):
                outcome = "SUCCESS"
                result_text = "Approach-wall completed: SUCCESS (wall detected after advance stop)"
                log_messages.append("Wall detected after advance stop - SUCCESS")
            else:
                outcome = "NO_WALL_REACHABLE"
                result_text = f"Approach-wall terminated: NO_WALL_REACHABLE (advance stop_reason={stop_reason})"
            break

        adv_data = adv.get("data", {}) or {}
        steps_done = int(adv_data.get("steps_completed", 0))
        blocks_remaining = max(0, blocks_remaining - steps_done)
        log_messages.append(f"Advanced {steps_done} block(s), remaining={blocks_remaining}")

    if log_messages:
        result_text += "\n" + "\n".join(log_messages)

    # Extract metadata fields for extra (excluding redundant success field)
    extra_metadata = {
        "outcome": outcome,
        "target": target,
        "step_duration": step_duration,
        "placement_item": placement_item,
        "log": log_messages,
    }
    
    status = "success" if outcome == "SUCCESS" else "failed"
    return executor._create_uniform_return(status, value=result_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("nav-approach-wall module loaded")

