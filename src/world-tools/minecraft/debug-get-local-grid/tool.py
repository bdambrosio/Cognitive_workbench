"""
Debug tool: Get raw local_grid from world_state.

Hidden from planner catalog - for testing/debugging only.
"""

import sys
import os
from typing import Any, Dict

# Ensure minecraft world-tools root is importable
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

from local_grid import get_or_init, WORLD_STATE_KEY


def tool(input_value=None, **kwargs):
    """
    Get raw local_grid from world_state.
    
    Returns:
        Uniform return format with raw local_grid data.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    grid = get_or_init(executor)
    
    # Count cells
    cells = grid.get("cells", {})
    cell_count = len(cells) if isinstance(cells, dict) else 0
    
    center = grid.get("center", {})
    radius = grid.get("radius", 0)
    
    summary = f"Local grid: {cell_count} cells, radius={radius}, center={center}"
    
    return executor._create_uniform_return(
        "success",
        value=summary,
        extra={"local_grid": grid}
    )
