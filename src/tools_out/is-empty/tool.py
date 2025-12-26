"""
is-empty tool - check if text is null, empty, or whitespace only.
"""

def tool(input_value, runtime=None, **kwargs):
    """
    Check if input_value is null, empty, or only whitespace.
    
    Args:
        input_value: Text content to check
        **kwargs: Optional parameters (none used)
    
    Returns:
        bool: True if empty/whitespace, False if has content
    """
    if input_value is None:
        return True
    
    if not isinstance(input_value, str):
        input_value = str(input_value)
    
    return len(input_value.strip()) == 0

