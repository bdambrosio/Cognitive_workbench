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
        Dict with wait completion information (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    # Synchronous wait for 1 second
    time.sleep(1)
    
    status_text = "Wait completed: 1 second elapsed"
    
    return {
        "text": status_text,
        "format": "text",
        "metadata": {
            "wait_duration_seconds": 1,
            "status": "completed"
        },
        "char_count": len(status_text)
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    result = tool()
    print(result)

