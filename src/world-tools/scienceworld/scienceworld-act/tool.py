"""
ScienceWorld act tool.
Takes an action in an existing ScienceWorld session and returns the new observation.
"""

import logging
import re
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from importlib.util import find_spec
# No longer using global session store - env stored in executive_node.scienceworld_env

logger = logging.getLogger(__name__)

def _require_scienceworld():
    if find_spec("scienceworld") is None:
        return None
    from scienceworld import ScienceWorldEnv  # type: ignore
    return ScienceWorldEnv


def tool(value=None, **kwargs):
    """
    Take an action in ScienceWorld.

    Args:
        action: text command (required)
        executive_node: required (contains scienceworld_env)

    Returns:
        Dict with observation content (text, format, metadata, char_count, filtered).
        Executor will create Note from this content.
    """
    action = kwargs.get("action") or value or ""
    
    if not isinstance(action, str) or not action.strip():
        return {"status": "failed", "reason": "action required (string)"}

    executive_node = kwargs.get("executive_node")
    if not executive_node:
        return {"status": "failed", "reason": "executive_node required in kwargs"}

    ScienceWorldEnv = _require_scienceworld()
    if ScienceWorldEnv is None:
        return {"status": "failed", "reason": "scienceworld package not found; pip install scienceworld"}

    # Get env from executive_node (no session_id needed)
    env = getattr(executive_node, 'scienceworld_env', None)
    if env is None:
        # Check if initialization might have failed
        scienceworld_config = getattr(executive_node, 'character_config', {}).get('scienceworld_config')
        if scienceworld_config:
            logger.warning("ScienceWorld config present but environment not initialized. Initialization may have failed or environment was cleared.")
        else:
            logger.debug("No ScienceWorld config found - environment initialization was skipped.")
        return {"status": "failed", "reason": "No active ScienceWorld session. Call scienceworld-reset first."}
    
    # Verify environment is still valid (Java server might have shut down)
    try:
        # Try to access a property to verify the env is still connected
        _ = env.getTaskDescription if hasattr(env, 'getTaskDescription') else None
    except Exception as e:
        logger.warning(f"ScienceWorld environment appears invalid (Java server may have shut down): {e}")
        # Clear the invalid environment
        executive_node.scienceworld_env = None
        return {"status": "failed", "reason": "ScienceWorld environment is no longer valid. Call scienceworld-reset to reinitialize."}
    
    # Check if action is a self-inspection command (look at agent/self/etc)
    action_string = action.strip().lower()
    # Regex matches: look/l/examine/x [at] agent/self/me/myself
    is_self_inspection = re.match(r'^(look|l|examine|x)(\s+at)?\s+(agent|self|me|myself)$', action_string)
    
    obs, reward, done, info = env.step(action)

    # Check for error indicators in observation
    obs_str = str(obs) if obs else ""
    
    # If self-inspection command, append inventory
    if is_self_inspection:
        try:
            inv_obs, _, _, _ = env.step("inventory")
            inv_obs_str = str(inv_obs) if inv_obs else ""
            if inv_obs_str:
                obs_str += f"\n[Auto-appended Inventory]: {inv_obs_str}"
        except Exception as e:
            logger.debug(f"Inventory call failed during self-inspection: {e}")
            # Continue with original observation
    obs_lower = obs_str.lower()
    
    # Common ScienceWorld error patterns (specific to action failures)
    error_patterns = [
        "no known action matches",
        "error:",
        "invalid action",
        "unknown action",
        "action not recognized",
        "cannot perform that action"
    ]
    
    # Check if observation starts with or contains error message
    is_error = any(pattern in obs_lower for pattern in error_patterns)
    
    # Also check info dict for error flags
    if isinstance(info, dict):
        if info.get('error', False) or info.get('valid', False) == False:
            is_error = True

    # Generate session_id for metadata (from env object id)
    session_id = str(id(env))  # Use object id as session identifier for metadata

    # Get filtered actions using affordance filtering
    filtered_actions = []
    try:
        # Import from local scripts directory
        script_dir = Path(__file__).parent / "scripts"
        if script_dir.exists() and str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))
        
        # Import affordance filtering modules (local scripts)
        from affordances import Heuristics, AffordanceFilter, extract_locations_from_observation  # type: ignore
        from scienceworld_interface import extract_actions_and_state  # type: ignore
        
        heur = Heuristics(locations={"kitchen", "bedroom", "bathroom", "living room", "hallway", "garage", "workshop", "greenhouse", "outside"})
        filt = AffordanceFilter(heur)
        
        raw_actions, ws = extract_actions_and_state(env)
        new_locs = extract_locations_from_observation(obs_str)
        heur.locations |= new_locs
        filtered_actions = f"""Available options for scienceworld-act 'action' parameter: {filt.filter_actions(raw_actions, ws)}
 - When choosing a ScienceWorld action, copy the action string verbatim from the most recent affordance list or select by index.
 - Never paraphrase, truncate, or synthesize action text."""
    except Exception as e:
        logger.debug(f"Affordance filtering failed (non-critical): {e}")
        # Continue without filtered actions if filtering fails

    # Build content dict (executor will create Note from this)
    content = {
        "affordances": filtered_actions,
        "text": obs_str,
        "format": "text",
        "metadata": {
            "session_id": session_id,  # For metadata/reference only
            "action": action,
            "reward": reward,
            "done": bool(done),
            "info": info if isinstance(info, dict) else {}
        },
        "char_count": len(obs_str) if obs_str else 0
    }

    # Return failure if error detected
    if is_error:
        error_reason = obs_str[:200] if obs_str else "Invalid action"
        return {"status": "failed", "reason": error_reason}

    # Return content dict (executor will create Note)
    return content


if __name__ == "__main__":
    # Simple self-test (requires a running session)
    print("scienceworld-act requires a session_id from scienceworld-reset.")

