"""
Minecraft wait tool.
Synchronous wait for 1 second using time.sleep.
"""

import logging
import time
from typing import Any, Dict

logger = logging.getLogger(__name__)


def tool(input_value=None, **kwargs):
    """
    Wait synchronously for 1 second.
    
    Args:
        input_value: ignored
        out: variable name to store result (handled by executor)
        
    Returns:
        Dict with result using uniform return format.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    # Synchronous wait for 1 second
    time.sleep(1)
    
    status_text = "Wait completed: 1 second elapsed"
    
    # Extract metadata fields for extra
    extra_metadata = {
        "wait_duration_seconds": 1,
        "status": "completed"
    }
    
    return executor._create_uniform_return('success', value=status_text, extra=extra_metadata)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

