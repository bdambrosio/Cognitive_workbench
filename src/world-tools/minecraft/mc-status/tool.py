"""
Minecraft status tool.
Fast heartbeat + sanity check for bot connection and state.
"""

import logging
import os
import time
import requests
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

# Default Minecraft bot server URL (can be overridden via environment variable or config)
DEFAULT_MINECRAFT_URL = os.getenv("MINECRAFT_URL", "http://localhost:3003")

_LAST_SAMPLE: Dict[str, Any] = {}


def _extract_y_from_status(data: Dict[str, Any]) -> Optional[float]:
    position = data.get('position')
    if isinstance(position, dict):
        y = position.get('y')
        return float(y) if isinstance(y, (int, float)) else None
    if isinstance(position, (list, tuple)) and len(position) >= 2:
        y = position[1]
        return float(y) if isinstance(y, (int, float)) else None
    return None


def _derive_vertical_state(y_prev: Optional[float], y_now: Optional[float], dt_s: Optional[float], y_epsilon: float) -> Tuple[str, str]:
    if y_now is None:
        return "UNKNOWN", "Missing current Y position"
    if y_prev is None:
        return "UNKNOWN", "First sample: no prior Y position to compare"
    if dt_s is None or dt_s <= 0:
        return "UNKNOWN", "Missing time delta"
    dy = y_now - y_prev
    if dy < -y_epsilon:
        return "FALLING", f"Y decreasing: dy={dy:.3f} over dt={dt_s:.3f}s (eps={y_epsilon:.3f})"
    if abs(dy) <= y_epsilon:
        return "STABLE", f"Y unchanged: dy={dy:.3f} over dt={dt_s:.3f}s (eps={y_epsilon:.3f})"
    return "UNKNOWN", f"Y increasing: dy={dy:.3f} over dt={dt_s:.3f}s (eps={y_epsilon:.3f})"



def tool(input_value=None, **kwargs):
    """
    Get Minecraft bot status - fast heartbeat + sanity check.
    
    Args:
        input_value: "double_sample" to perform two samples internally (~0.2s delay) for immediate vertical_state, otherwise ignored
        minecraft_url: Optional URL override for Minecraft bot server (default: http://localhost:3003)
        
    Returns:
        Dict with status information using uniform return format.
        Executor will create Note from this content.
        Includes vertical_state: STABLE | FALLING | UNKNOWN (derived from Y position changes).
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    minecraft_url = kwargs.get("world_url") or kwargs.get("minecraft_url") or DEFAULT_MINECRAFT_URL
    url = f"{minecraft_url}/status"
    # Check input_value for "double_sample" mode (value parameter from action)
    double_sample_requested = (input_value == "double_sample" or kwargs.get("double_sample", False))
    sample_delay_s = float(kwargs.get("sample_delay_s", 0.2))
    y_epsilon = float(kwargs.get("y_epsilon", 0.02))
    
    try:
        def fetch_status() -> Dict[str, Any]:
            #logger.info(f"🔍 mc-status: GET {url}")
            response = requests.get(url, timeout=10.0)
            #logger.info(f"📥 mc-status: Response status={response.status_code}, headers={dict(response.headers)}")
            response.raise_for_status()
            d = response.json()
            logger.debug(f"📥 mc-status: Response body={d}")
            return d
        
        t_now = time.time()
        data = fetch_status()
        
        if not data.get('ok'):
            return executor._create_uniform_return('failed', reason=data.get('error', 'unknown error'))

        y_prev = None
        y_now = _extract_y_from_status(data)
        dt_s = None
        mode = "single"
        
        # Auto-force double-sample if first call (no cached sample) and delay < 1 second
        has_cached_sample = (_LAST_SAMPLE.get("y") is not None and _LAST_SAMPLE.get("t") is not None)
        auto_force_double = (not has_cached_sample and sample_delay_s < 1.0)
        double_sample = double_sample_requested or auto_force_double
        
        if double_sample:
            if sample_delay_s > 0:
                time.sleep(sample_delay_s)
            t2 = time.time()
            data2 = fetch_status()
            if data2.get('ok'):
                y2 = _extract_y_from_status(data2)
                y_prev = y_now
                y_now = y2
                dt_s = t2 - t_now
                t_now = t2
                data = data2  # Use latest payload for displayed fields
                mode = "double_sample" if not auto_force_double else "auto_double_sample"
        else:
            if _LAST_SAMPLE.get("y") is not None and _LAST_SAMPLE.get("t") is not None:
                y_prev = _LAST_SAMPLE.get("y")
                dt_s = t_now - _LAST_SAMPLE.get("t")
                mode = "cached"
        
        vertical_state, vertical_explanation = _derive_vertical_state(y_prev, y_now, dt_s, y_epsilon)
        _LAST_SAMPLE.clear()
        _LAST_SAMPLE.update({"t": t_now, "y": y_now})
        
        # Format status as readable text
        status_parts = []
        status_parts.append("Minecraft Bot Status:")
        status_parts.append("Connected: True")
        
        position = data.get('position')
        if position:
            if isinstance(position, dict):
                status_parts.append(f"Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
            elif isinstance(position, (list, tuple)) and len(position) >= 3:
                status_parts.append(f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        
        yaw = data.get('yaw')
        pitch = data.get('pitch')
        if yaw is not None and pitch is not None:
            status_parts.append(f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}")
        
        health = data.get('health')
        if health is not None and isinstance(health, (int, float)):
            status_parts.append(f"Health: {health}/20")
        
        food = data.get('food')
        if food is not None and isinstance(food, (int, float)):
            status_parts.append(f"Food: {food}/20")
        
        on_ground = data.get('onGround')
        if on_ground is not None:
            status_parts.append(f"On Ground: {on_ground}")

        status_parts.append(f"Vertical State: {vertical_state}")
        status_parts.append(f"Vertical State Evidence: {vertical_explanation} (mode={mode})")
        
        dimension = data.get('dimension')
        if dimension:
            status_parts.append(f"Dimension: {dimension}")
        
        action = data.get('action', {})
        if isinstance(action, dict) and action.get('type'):
            action_type = action.get('type', 'idle')
            action_note = action.get('note', '')
            if action_note:
                status_parts.append(f"Current Action: {action_type} ({action_note})")
            else:
                status_parts.append(f"Current Action: {action_type}")
        else:
            status_parts.append("Current Action: idle")
        
        status_text = "\n".join(status_parts)
        
        # Build structured data dict
        structured_data = dict(data)
        structured_data["vertical_state"] = vertical_state
        structured_data["vertical_state_explanation"] = vertical_explanation
        structured_data["vertical_state_mode"] = mode
        structured_data["vertical_state_y_prev"] = y_prev
        structured_data["vertical_state_y_now"] = y_now
        structured_data["vertical_state_dt_s"] = dt_s
        structured_data["vertical_state_y_epsilon"] = y_epsilon
        structured_data["vertical_state_sample_delay_s"] = sample_delay_s if double_sample else None
        
        return executor._create_uniform_return('success', value=status_text, data=structured_data)
    except requests.exceptions.RequestException as e:
        logger.error(f"Minecraft status request failed: {e}")
        return executor._create_uniform_return('failed', reason=f"API request failed: {e}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

