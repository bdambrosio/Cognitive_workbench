import json
from typing import List, Dict, Any


def format_views_compact(views: List[Dict[str, Any]]) -> str:
    """Return one JSON object per line for each view direction without indentation.

    This preserves all information while minimizing tokens for LLM prompts.
    """
    if not views:
        return ""
    lines = []
    for view in views:
        try:
            lines.append(json.dumps(view, separators=(",", ":")))
        except Exception:
            # Fallback to default dumps if compact separators fail on unexpected data
            lines.append(json.dumps(view))
    return "\n".join(lines)


