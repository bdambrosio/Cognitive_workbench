"""
Navigation backtrack tool.
Attempt to return to a recently known safe navigation state using bounded local motion.
"""

import time
from math import atan2, degrees

def tool(input_value=None, **kwargs):
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available"}

    # ----------------------------
    # Parameters (hard-bounded)
    # ----------------------------
    MAX_TARGETS = int(kwargs.get("max_targets", 3))
    MAX_ORIENT_TRIES = 4  # 0°, ±90°, 180°
    POSITION_EPS = 0.51   # block-center tolerance

    # ----------------------------
    # Helper predicates
    # ----------------------------
    def same_cell(a, b):
        return (
            round(a["x"]) == round(b["x"]) and
            round(a["y"]) == round(b["y"]) and
            round(a["z"]) == round(b["z"])
        )

    def is_safe(nav_state):
        return (
            nav_state.get("support_here") in ("solid", "unsafe")
            and not nav_state.get("was_fall", False)
        )

    def classify_delta(from_pose, to_pose):
        dy = to_pose["y"] - from_pose["y"]
        if abs(dy) < 0.2:
            return "step"
        if 0.2 <= dy <= 1.2:
            return "climb"
        if -1.2 <= dy <= -0.2:
            return "descend"
        return None

    def compute_yaw(from_pose, to_pose):
        dx = to_pose["x"] - from_pose["x"]
        dz = to_pose["z"] - from_pose["z"]
        if abs(dx) < 0.01 and abs(dz) < 0.01:
            return from_pose["yaw"]
        return degrees(atan2(-dx, dz)) % 360

    def round_to_cardinal(yaw_deg: float) -> float:
        """Round yaw to nearest cardinal direction (0, 90, 180, 270)."""
        cardinals = [0.0, 90.0, 180.0, 270.0]
        normalized = float(yaw_deg) % 360.0
        return min(
            cardinals,
            key=lambda c: min(
                abs(normalized - c),
                abs(normalized - c + 360.0),
                abs(normalized - c - 360.0),
            ),
        )

    def delta_to_direction(from_yaw: float, to_yaw: float) -> str:
        """
        Convert a cardinal-to-cardinal yaw delta into nav-turn direction.
        Assumes inputs are already cardinal (0/90/180/270).
        """
        dy = (to_yaw - from_yaw) % 360.0
        if dy == 0.0:
            return "forward"
        if dy == 90.0:
            return "right"
        if dy == 180.0:
            return "back"
        if dy == 270.0:
            return "left"
        # Fallback: align without rotation
        return "forward"

    def orient_to_cardinal(desired_yaw: float) -> bool:
        """
        Use nav-turn (relative) to reach a desired cardinal yaw.
        Bounded to at most 2 nav-turn calls: align -> rotate.
        """
        # 1) Align current yaw to nearest cardinal
        r1 = executor.execute_action_with_log({"type": "nav-turn", "direction": "forward"}, "nav-backtrack")
        if r1.get("status") != "success":
            return False
        yaw1 = r1.get("data", {}).get("yaw")
        if yaw1 is None:
            return False

        yaw1 = round_to_cardinal(float(yaw1))
        desired = round_to_cardinal(float(desired_yaw))
        direction = delta_to_direction(yaw1, desired)
        if direction == "forward":
            return True

        r2 = executor.execute_action_with_log({"type": "nav-turn", "direction": direction}, "nav-backtrack")
        return r2.get("status") == "success"

    # ----------------------------
    # Load nav history
    # ----------------------------
    nav_list = executor.get_world_state("nav")
    if not isinstance(nav_list, list) or len(nav_list) < 2:
        history = []
    else:
        # Use nav[1:] as history (skip current state at nav[0])
        history = nav_list[1:]

    if not history:
        return executor._create_uniform_return(
            "failed",
            value="No navigation history available",
            reason="NO_HISTORY"
        )

    # ----------------------------
    # Get current pose
    # ----------------------------
    status = executor.execute_action_with_log({"type": "mc-status"}, "nav-backtrack")
    if status.get("status") != "success":
        return executor._create_uniform_return(
            "failed",
            value="Failed to obtain status",
            reason="STATUS_FAILED"
        )

    cur = status["result"]["position"]
    cur_pose = {
        "x": cur["x"],
        "y": cur["y"],
        "z": cur["z"],
        "yaw": status["result"]["yaw"],
    }

    # ----------------------------
    # Select recovery targets
    # ----------------------------
    safe_targets = []
    for s in history:
        if is_safe(s):
            dy = abs(s["pose"]["y"] - cur_pose["y"])
            if dy <= 1.2:
                safe_targets.append(s)
        if len(safe_targets) >= MAX_TARGETS:
            break

    if not safe_targets:
        return executor._create_uniform_return(
            "failed",
            value="No safe targets found in navigation history",
            reason="NO_SAFE_TARGET"
        )

    # ----------------------------
    # Attempt reconnection
    # ----------------------------
    for target in safe_targets:
        target_pose = target["pose"]
        move_kind = classify_delta(cur_pose, target_pose)
        if move_kind is None:
            continue

        base_yaw = compute_yaw(cur_pose, target_pose)
        yaw_offsets = [0, 90, -90, 180]

        for off in yaw_offsets[:MAX_ORIENT_TRIES]:
            desired_yaw = round_to_cardinal((base_yaw + off) % 360.0)

            # Turn (relative only)
            if not orient_to_cardinal(desired_yaw):
                continue

            # Move
            if move_kind == "step":
                move = {"type": "nav-move"}
            elif move_kind == "climb":
                move = {"type": "nav-climb"}
            elif move_kind == "descend":
                move = {"type": "nav-descend"}
            else:
                continue

            result = executor.execute_action_with_log(move, "nav-backtrack")
            if result.get("status") != "success":
                continue

            data = result.get("data", {})
            if not data.get("success", False):
                continue

            # Verify position
            status2 = executor.execute_action_with_log(
                {"type": "mc-status"},
                "nav-backtrack"
            )
            if status2.get("status") != "success":
                continue

            pos2 = status2["result"]["position"]
            now_pose = {
                "x": pos2["x"],
                "y": pos2["y"],
                "z": pos2["z"],
                "yaw": status2["result"]["yaw"],
            }

            if same_cell(now_pose, target_pose):
                # Optional: update nav history on success
                # nav_state["history"].insert(0, {
                #     "pose": now_pose,
                #     "support_here": target.get("support_here"),
                #     "was_fall": False,
                #     "timestamp": time.time(),
                # })
                # executor.set_world_state("nav", nav_state)

                return executor._create_uniform_return(
                    "success",
                    value=f"Recovered to target via {move_kind}",
                    extra={
                        "recovered_to": target_pose,
                        "method": move_kind,
                        "yaw": desired_yaw,
                    }
                )

    # ----------------------------
    # Failure
    # ----------------------------
    return executor._create_uniform_return(
        "failed",
        value="Could not reach any safe target",
        reason="UNREACHABLE"
    )
