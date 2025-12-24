"""
ScienceWorld reset tool.
Starts a ScienceWorld scenario and returns the initial observation and session_id.
"""

import logging
import uuid
from importlib.util import find_spec
from typing import Any, Dict, Optional

# No longer using global session store - env stored in executive_node.scienceworld_env

logger = logging.getLogger(__name__)

def _require_scienceworld():
    if find_spec("scienceworld") is None:
        return None
    from scienceworld import ScienceWorldEnv  # type: ignore
    return ScienceWorldEnv


def tool(value, **kwargs):
    """
    Start a ScienceWorld episode.
    
    Reads scenario, difficulty, and seed from scienceworld_config (passed via kwargs).
    The 'value' parameter is ignored (scenario comes from config).

    Args:
        value: ignored (scenario comes from config)
        scenario: string (from scienceworld_config, required)
        difficulty: int (from scienceworld_config, default 0)
        seed: int (from scienceworld_config, default 42)
        executive_node: required (stores scienceworld_env)

    Returns:
        Dict with initial observation content (text, format, metadata, char_count).
        Executor will create Note from this content.
    """
    # Read scenario, difficulty, seed from config (passed via kwargs from infospace_executor)
    scenario = kwargs.get("scenario")
    if not isinstance(scenario, str) or not scenario.strip():
        return {"status": "failed", "reason": "scenario name required in scienceworld_config"}

    ScienceWorldEnv = _require_scienceworld()
    if ScienceWorldEnv is None:
        return {"status": "failed", "reason": "scienceworld package not found; pip install scienceworld"}

    variation_idx = kwargs.get("difficulty", 0)  # Map difficulty to variationIdx
    simplification_str = kwargs.get("simplification", '')  # Optional simplification string (e.g., "easy")
    seed = kwargs.get("seed", 42)  # Note: seed not supported in load(), stored for metadata only

    # Get executive_node to store env
    executive_node = kwargs.get("executive_node")
    if not executive_node:
        return {"status": "failed", "reason": "executive_node required in kwargs"}

    # ScienceWorld load() signature: load(taskName, variationIdx, simplificationStr, generateGoldPath)
    # There is no seed parameter - seed is stored for metadata/reproducibility tracking only
    if executive_node.scienceworld_env is None:
        env = ScienceWorldEnv()
        env.load(scenario, variation_idx, simplification_str)
    else:
        env = executive_node.scienceworld_env
    #env.load(scenario, variation_idx, simplification_str)
    obs, info = env.reset()

    # Store env in executive_node (replaces global session store)
    executive_node.scienceworld_env = env
    
    # Generate session_id for metadata (not used for lookup anymore)
    session_id = str(uuid.uuid4())

    # Build content dict (executor will create Note from this)
    content = {
        "text": obs if isinstance(obs, str) else str(obs),
        "format": "text",
        "metadata": {
            "session_id": session_id,
            "scenario": scenario,
            "variation_idx": variation_idx,
            "simplification": simplification_str,
            "seed": seed,  # Stored for metadata, not used by ScienceWorld API
            "reward": 0,
            "done": False,
            "info": info if isinstance(info, dict) else {}
        },
        "char_count": len(str(obs)) if obs is not None else 0
    }

    # Return content dict (executor will create Note)
    return content


if __name__ == "__main__":
    # Simple self-test (requires scienceworld to be installed and assets available)
    rm = None
    result = tool("waterplant", agent_name="TestAgent", resource_manager=rm)
    print(result)

