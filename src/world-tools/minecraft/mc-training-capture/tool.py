"""
Minecraft training data capture tool.

Enable/disable automatic capture of training samples (partial observation grid + ground truth).
"""

import logging
import os
import sys
from typing import Any, Dict

# Ensure minecraft world-tools root is importable (src/world-tools/minecraft)
_THIS_DIR = os.path.dirname(__file__)
_MINECRAFT_TOOLS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _MINECRAFT_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, _MINECRAFT_TOOLS_ROOT)

logger = logging.getLogger(__name__)

try:
    from local_grid import enable_training_capture, disable_training_capture
except Exception as e:
    logger.warning(f"Failed to import training capture functions: {e}")
    enable_training_capture = None
    disable_training_capture = None


def tool(input_value=None, **kwargs):
    """
    Enable or disable training data capture.
    
    Args:
        enable: bool - If True, enable capture; if False, disable (default: True)
        sample_interval: int - Capture every N position changes (default: 5)
        include_ground_truth: bool - If True, fetch ground truth from bridge (default: True, slower but required for useful data)
        minecraft_url: str - Bridge URL for ground truth (optional)
    
    Returns:
        Uniform return format dict.
    """
    executor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}
    
    if enable_training_capture is None or disable_training_capture is None:
        return executor._create_uniform_return(
            "failed",
            value="Training capture functions not available",
            reason="local_grid_unavailable"
        )
    
    enable = kwargs.get("enable", True)
    if isinstance(enable, str):
        enable = enable.lower() in ("true", "1", "yes")
    
    if enable:
        sample_interval = int(kwargs.get("sample_interval", 5))
        include_ground_truth = kwargs.get("include_ground_truth", True)
        if isinstance(include_ground_truth, str):
            include_ground_truth = include_ground_truth.lower() in ("true", "1", "yes")
        minecraft_url = kwargs.get("minecraft_url") or kwargs.get("world_url")
        
        enable_training_capture(
            executor,
            sample_interval=sample_interval,
            include_ground_truth=include_ground_truth,
            minecraft_url=minecraft_url,
        )
        
        gt_status = "with ground truth" if include_ground_truth else "without ground truth"
        return executor._create_uniform_return(
            "success",
            value=f"Training capture enabled (interval={sample_interval}, {gt_status})",
            extra={
                "enabled": True,
                "sample_interval": sample_interval,
                "include_ground_truth": include_ground_truth,
            }
        )
    else:
        disable_training_capture(executor)
        return executor._create_uniform_return(
            "success",
            value="Training capture disabled",
            extra={"enabled": False}
        )
