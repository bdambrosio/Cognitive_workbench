import json
from typing import List, Dict, Any

from templates import PHYSIOLOGICAL_STATES


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
            
            # Filter infrastructure paths by distance <= 16 (do not mutate original input)
            try:
                if isinstance(view, dict) and isinstance(view.get('paths'), list):
                    filtered_paths = []
                    for p in view.get('paths'):
                        if isinstance(p, dict):
                            try:
                                if float(p.get('distance', 1e9)) <= 16:
                                    filtered_paths.append(p)
                            except Exception:
                                # If distance missing or unparsable, exclude
                                pass
                    # Shallow copy to avoid side effects
                    view = dict(view)
                    view['paths'] = filtered_paths
            except Exception:
                pass

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

def format_map_types(map_types: Dict[str, List[str]]) -> str:
    """Format map types with clean, unescaped quotes for better readability."""
    type_str = ''
    map_types_str = f'\n#Available map types:\n'
    if map_types.get('terrain_types'):
        type_str += '\n'.join(map_types['terrain_types'])+'\n'
    if map_types.get('infrastructure_types'):
        type_str += '\n'.join(map_types['infrastructure_types'])+'\n'
    if map_types.get('property_types'):
        type_str += '\n'.join(map_types['property_types'])+'\n'
    if map_types.get('resource_types'):
        type_str += '\n'.join(map_types['resource_types'])+'\n'
    if map_types.get('character_names'):
        type_str += '\n'.join(map_types['character_names'])+'\n'
    type_str += '\n'.join([need['name'] for need in PHYSIOLOGICAL_STATES])
    map_types_str += '\n'

    return type_str.replace('\n\n','\n')

def format_map_places(map_types: Dict[str, List[str]]) -> str:
    """Format map types with clean, unescaped quotes for better readability."""
    type_str = ''
    map_types_str = f'\n#Available map places:\n'
    if map_types.get('infrastructure_types'):
        type_str += '\n'.join(map_types['infrastructure_types'])+'\n'
    if map_types.get('property_types'):
        type_str += '\n'.join(map_types['property_types'])+'\n'
    map_types_str += '\n'

    return type_str.replace('\n\n','\n')

def format_map_tools(map_types: Dict[str, List[str]]) -> str:
    """Format map types with clean, unescaped quotes for better readability."""
    type_str = ''
    map_types_str = f'\n#Available map resources and tools:\n'
    if map_types.get('property_types'):
        type_str += '\n'.join(map_types['property_types'])+'\n'
 
    if map_types.get('resource_types'):
        type_str += '\n'.join(map_types['resource_types'])+'\n'
    map_types_str += '\n'
    return type_str.replace('\n\n','\n')

def _format_dict_clean(data):
    """Format dict without braces or quotes, more natural for LLM prompts."""
    try:
        if not isinstance(data, dict):
            return str(data)
        
        items = []
        for key, value in data.items():
            if isinstance(value, list):
                if value and isinstance(value[0], dict):
                    # Handle list of dicts (resources/paths with 'name' and other fields)
                    formatted_items = []
                    for item in value:
                        if isinstance(item, dict):
                            # Extract 'name' field, format others as "key: value"
                            name = item.get('name', '')
                            other_fields = []
                            for k, v in item.items():
                                if k != 'name':
                                    other_fields.append(f"{k}: {v}")
                            if name and other_fields:
                                formatted_items.append(f"{name} at {', '.join(other_fields)}")
                            elif name:
                                formatted_items.append(name)
                            else:
                                # Fallback: format all fields
                                formatted_items.append(', '.join(f"{k}: {v}" for k, v in item.items()))
                        else:
                            formatted_items.append(str(item))
                    items.append(f"{key}: [{', '.join(formatted_items)}]")
                else:
                    # Simple list
                    items.append(f"{key}: [{', '.join(str(v) for v in value)}]")
            else:
                # Simple value
                items.append(f"{key}: {value}")
        
        return ', '.join(items)
    except Exception:
        # Fallback to string representation
        return str(data)


def _format_value_clean(value):
    """Format value without quotes for natural LLM prompts."""
    try:
        if isinstance(value, dict):
            return _format_dict_clean(value)
        elif isinstance(value, list):
            return '[' + ', '.join(str(v) for v in value) + ']'
        else:
            return str(value)
    except Exception:
        return str(value)

def format_middle_nouns(middle_ontology):
    middle_nouns = middle_ontology.get('nouns', {}).get('nodes', [])
    middle_nouns_str = '\n'.join([f"{n['id']}" for n in middle_nouns])
    return middle_nouns_str

def format_middle_verbs(middle_ontology):
    middle_verbs = middle_ontology.get('verbs', {}).get('nodes', [])
    middle_verbs_str = '\n'.join([f"{v['id']}" for v in middle_verbs])
    return middle_verbs_str
