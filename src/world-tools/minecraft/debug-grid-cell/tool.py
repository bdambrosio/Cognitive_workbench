"""
Debug tool: Query specific cell in observed voxel grid.

Hidden from planner catalog - for testing/debugging only.
"""

import sys
import os
from typing import Any, Dict, Optional

# Ensure minecraft world-tools root is importable
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

from local_grid import get_observed_voxel_grid


def tool(input_value=None, **kwargs):
    """
    Query specific cell in observed voxel grid.
    
    Args:
        dx: int - Agent-relative X coordinate
        dy: int - Agent-relative Y coordinate
        dz: int - Agent-relative Z coordinate
        radius: int - Optional grid radius (default: 2)
    
    Returns:
        Uniform return format with cell data.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    dx = kwargs.get("dx")
    dy = kwargs.get("dy")
    dz = kwargs.get("dz")
    radius = int(kwargs.get("radius", 2))
    
    if dx is None or dy is None or dz is None:
        return executor._create_uniform_return(
            "failed",
            value="dx, dy, dz required",
            reason="missing_coordinates"
        )
    
    dx = int(dx)
    dy = int(dy)
    dz = int(dz)
    
    # Get voxel grid
    voxel_grid = get_observed_voxel_grid(executor, radius=radius)
    if not voxel_grid:
        return executor._create_uniform_return(
            "failed",
            value="No voxel grid available",
            reason="grid_unavailable"
        )
    
    # Find cell
    cells = voxel_grid.get("cells", [])
    cell = None
    for c in cells:
        if isinstance(c, dict) and c.get("dx") == dx and c.get("dy") == dy and c.get("dz") == dz:
            cell = c
            break
    
    if not cell:
        return executor._create_uniform_return(
            "failed",
            value=f"Cell [{dx},{dy},{dz}] not found in grid (radius={radius})",
            reason="cell_not_found"
        )
    
    block_id = cell.get("block_id", "air")
    solid = cell.get("solid", False)
    support = cell.get("support", False)
    
    summary = f"Cell [{dx},{dy},{dz}]: {block_id}, solid={solid}, support={support}"
    
    return executor._create_uniform_return(
        "success",
        value=summary,
        extra={"cell": cell}
    )
