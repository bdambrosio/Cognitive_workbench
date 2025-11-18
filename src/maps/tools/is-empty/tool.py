"""
is-empty tool - check if text is null, empty, or whitespace only.
"""

def tool(value, **kwargs):
    """
    Check if value is null, empty, or only whitespace.
    
    Args:
        value: Text content to check
        **kwargs: Optional parameters (none used)
    
    Returns:
        bool: True if empty/whitespace, False if has content
    """
    if value is None:
        return True
    
    if not isinstance(value, str):
        value = str(value)
    
    return len(value.strip()) == 0

