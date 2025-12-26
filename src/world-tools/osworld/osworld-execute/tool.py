"""
OSWorld execute tool.
Executes Python code (typically pyautogui commands) in the OSWorld environment.
"""

import logging
import os
import requests
from typing import Any, Dict

logger = logging.getLogger(__name__)

# Default OSWorld server URL (can be overridden via environment variable or config)
DEFAULT_OSWORLD_URL = os.getenv("OSWORLD_URL", "http://localhost:3002")


def tool(input_value=None, **kwargs):
    """
    Execute Python code in OSWorld environment.
    
    Args:
        input_value: Python code string (preferred)
        python: Python code string (alternative to input_value)
        return_observation: bool (default: False) - include observation in response
        osworld_url: Optional URL override for OSWorld server (default: http://localhost:3002)
        
    Returns:
        Dict with execution result (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    python_code = kwargs.get("python") or input_value or ""
    
    if not isinstance(python_code, str) or not python_code.strip():
        return {"status": "failed", "reason": "python code required (string)"}
    
    osworld_url = kwargs.get("world_url") or kwargs.get("osworld_url") or DEFAULT_OSWORLD_URL
    return_observation = kwargs.get("return_observation", False)
    
    try:
        response = requests.post(
            f"{osworld_url}/execute",
            json={
                "python": python_code,
                "return_observation": return_observation
            },
            timeout=60.0
        )
        response.raise_for_status()
        data = response.json()
        
        # Build result text
        result_parts = []
        result_parts.append(f"Execution Result:")
        result_parts.append(f"Success: {data.get('success', False)}")
        result_parts.append(f"Return Code: {data.get('returncode', -1)}")
        result_parts.append(f"Duration: {data.get('duration_ms', 0)} ms")
        result_parts.append(f"Step Counter: {data.get('step_counter', 0)}")
        
        stdout = data.get("stdout", "")
        stderr = data.get("stderr", "")
        
        if stdout:
            result_parts.append(f"\nSTDOUT:\n{stdout}")
        if stderr:
            result_parts.append(f"\nSTDERR:\n{stderr}")
        
        if return_observation and data.get("observation"):
            result_parts.append("\nObservation included in metadata")
        
        result_text = "\n".join(result_parts)
        
        return {
            "text": result_text,
            "format": "text",
            "metadata": {
                "success": data.get("success"),
                "returncode": data.get("returncode"),
                "duration_ms": data.get("duration_ms"),
                "step_counter": data.get("step_counter"),
                "stdout": stdout,
                "stderr": stderr,
                "python_code": python_code,
                **({"observation": data.get("observation"), "timestamp": data.get("timestamp")} if return_observation and data.get("observation") else {})
            },
            "char_count": len(result_text)
        }
    except requests.exceptions.RequestException as e:
        logger.error(f"OSWorld execute request failed: {e}")
        return {"status": "failed", "reason": f"API request failed: {e}"}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool("pyautogui.click(100,200)")
    print(result)

