import json
from typing import List, Dict, Any


def format_views_compact(views: List[Dict[str, Any]]) -> str:
    """Return one JSON object per line for each view direction without indentation.

    This preserves all information while minimizing tokens for LLM prompts.
    Outputs clean, unescaped quotes for better readability.
    Handles both Python dicts and JSON strings.
    """
    if not views:
        return ""
    lines = []
    for view in views:
        try:
            # Handle case where view might be a JSON string instead of a dict
            if isinstance(view, str):
                # Try to parse JSON string back to dict
                import json
                view = json.loads(view)
            
            # Format with clean quotes
            lines.append(_format_dict_clean(view))
        except Exception:
            # Fallback to clean formatting if custom formatting fails
            try:
                lines.append(_format_dict_clean(view))
            except:
                # Last resort: return as string
                lines.append(str(view))
    return "\n".join(lines)


def _format_dict_clean(data):
    """Format a dict with clean, unescaped quotes for better readability."""
    if not isinstance(data, dict):
        return str(data)
    
    items = []
    for key, value in data.items():
        if isinstance(value, str):
            items.append(f"'{key}': '{value}'")
        elif isinstance(value, list):
            if value and isinstance(value[0], dict):
                # Handle list of dicts (like resources)
                formatted_items = []
                for item in value:
                    if isinstance(item, dict):
                        formatted_items.append(_format_dict_clean(item))
                    else:
                        formatted_items.append(_format_value_clean(item))
                items.append(f"'{key}': [{', '.join(formatted_items)}]")
            else:
                # Handle simple lists
                formatted_values = [_format_value_clean(v) for v in value]
                items.append(f"'{key}': [{', '.join(formatted_values)}]")
        else:
            items.append(f"'{key}': {_format_value_clean(value)}")
    
    return '{' + ', '.join(items) + '}'


def _format_value_clean(value):
    """Format a value with clean, unescaped quotes for better readability."""
    if isinstance(value, str):
        return f"'{value}'"
    elif isinstance(value, dict):
        return _format_dict_clean(value)
    elif isinstance(value, list):
        formatted_items = [_format_value_clean(item) for item in value]
        return '[' + ', '.join(formatted_items) + ']'
    else:
        # For numbers, booleans, None, etc., use str() for clean output
        return str(value)


