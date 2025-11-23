"""
Action Post-Processing - Compare action results against expectations using LLM.

Pure module (no Zenoh dependencies) for expectation comparison.
"""

import logging
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger(__name__)


def resolve_result_content(result_value: Any, resource_manager=None, max_chars: int = 2000) -> str:
    """
    Resolve result value to text content, handling Note/Collection IDs.
    
    Args:
        result_value: Result value (may be Note_X, Collection_X, or direct content)
        resource_manager: Optional resource manager for fetching content
        max_chars: Maximum characters to return
        
    Returns:
        Resolved text content (truncated to max_chars)
    """
    if result_value is None:
        return ""
    
    # If it's a Note ID, fetch content
    if isinstance(result_value, str) and result_value.startswith('Note_'):
        if resource_manager:
            resource = resource_manager.get_resource(result_value)
            if resource:
                content = resource.get('properties', {}).get('content')
                if content is not None:
                    text = str(content)
                    return text[:max_chars] if len(text) > max_chars else text
        return result_value[:max_chars] if len(result_value) > max_chars else result_value
    
    # If it's a Collection ID, flatten it
    if isinstance(result_value, str) and result_value.startswith('Collection_'):
        if resource_manager:
            resource = resource_manager.get_resource(result_value)
            if resource:
                content = resource.get('properties', {}).get('content')
                if isinstance(content, list):
                    # Flatten Collection: resolve each Note ID to content
                    note_contents = []
                    for item in content:
                        if isinstance(item, str) and item.startswith('Note_'):
                            note_resource = resource_manager.get_resource(item)
                            if note_resource:
                                note_content = note_resource.get('properties', {}).get('content')
                                if note_content is not None:
                                    note_contents.append(str(note_content))
                        elif isinstance(item, str) and item.startswith('Collection_'):
                            # Recursively flatten nested Collections
                            nested_text = resolve_result_content(item, resource_manager, max_chars)
                            if nested_text:
                                note_contents.append(nested_text)
                        else:
                            note_contents.append(str(item))
                    flattened = '\n\n'.join(note_contents)
                    return flattened[:max_chars] if len(flattened) > max_chars else flattened
        return result_value[:max_chars] if len(result_value) > max_chars else result_value
    
    # Direct content - convert to string
    text = str(result_value)
    return text[:max_chars] if len(text) > max_chars else text


def process_action_expectation(
    action: Dict,
    result: Dict,
    llm_generate: Callable,
    resource_manager: Optional[Any] = None,
    debug: bool = False
) -> Optional[Dict]:
    """
    Compare action result against expectation using LLM.
    
    Args:
        action: Action dict (may contain 'expect' field)
        result: Result dict with 'status' and 'value'
        llm_generate: Callback function for LLM generation (from infospace_executor)
        resource_manager: Optional resource manager for resolving Note/Collection IDs
        debug: Enable debug logging
        
    Returns:
        Dict with comparison result, or None if no expectation or processing skipped
        Format: {
            'matched': bool,
            'response': str,
            'action_type': str,
            'expect': str,
            'result_summary': str
        }
    """
    # Extract expectation from action
    expect = action.get('expect', '')
    if not expect:
        return None
    
    # Only process successful actions
    if result.get('status') != 'success':
        return None
    
    # Get result value
    result_value = result.get('value', '')
    if not result_value:
        return None
    
    action_type = action.get('type', 'unknown')
    
    # Resolve result content (handles Note/Collection IDs)
    resolved_text = resolve_result_content(result_value, resource_manager, max_chars=2000)
    
    if not resolved_text:
        return None
    
    if debug:
        logger.info(f'Processing expectation: type={action_type}, '
                   f'expect={expect[:50]}..., result={resolved_text[:50]}...')
    
    # Compare expectation against actual result using LLM
    prompt_text = """Does the actual result satisfy the expectation for the action?
#Action:
{{$action_text}}

#Expectation: 
{{$expect_text}}

#Actual Result:
{{$update_text}}

Answer 'yes' or 'no', followed by a brief explanation (max 20 words). Include no other text, reasoning, or formatting
"""
    
    try:
        # Call llm_generate (handles SGLang or other runtime)
        response = llm_generate(
            messages=[prompt_text],
            bindings={
                "action_text": action,
                "expect_text": expect,
                "update_text": resolved_text
            },
            max_tokens=30,
            temperature=0.3,
            is_json=False
        )
        
        # Handle response format (SGLang returns dict, llm_client returns object)
        if isinstance(response, dict):
            response_text = response.get('text', '').strip()
            success = response.get('success', False)
        elif hasattr(response, 'text'):
            response_text = response.text.strip()
            success = response.success if hasattr(response, 'success') else True
        else:
            logger.warning(f'Unexpected response format from llm_generate: {type(response)}')
            return None
        
        if not success or not response_text:
            logger.warning(f'LLM expectation comparison failed for {action_type}')
            return None
        
        # Parse yes/no response
        matched = response_text.strip().lower().startswith('yes')
        
        comparison_result = {
            'matched': matched,
            'response': response_text,
            'action_type': action_type,
            'expect': expect,
            'result_summary': resolved_text[:100] + '...' if len(resolved_text) > 100 else resolved_text
        }
        
        if matched:
            logger.info(f'Expectation match for {action_type}: {response_text}')
        else:
            logger.warning(f'Outcome failed to meet expectation {action_type}: {response_text}')
        
        return comparison_result
        
    except Exception as e:
        logger.error(f'Error processing action expectation: {e}')
        return None

