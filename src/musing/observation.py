from typing import Any, Dict, Optional


def build_observation(
    goal: str,
    preplan: Optional[str] = None,
    step_index: Optional[int] = None,
    error_count: Optional[int] = None,
    tool_name: Optional[str] = None,
    error_type: Optional[str] = None,
    segment_index: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Construct a lightweight observation payload for musing policy input.
    Keeps raw goal/preplan text and planner/error metadata; embeddings are left
    to the policy implementation.
    """
    return {
        "goal": goal,
        "preplan": preplan,
        "step_index": step_index,
        "error_count": error_count,
        "tool_name": tool_name,
        "error_type": error_type,
        "segment_index": segment_index,
    }
