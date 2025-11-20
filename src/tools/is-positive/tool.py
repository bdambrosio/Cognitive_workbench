"""
Is-positive tool - example Python condition tool.
"""

def tool(value, runtime=None, **kwargs):
    """
    Check if value is a positive number.
    
    Args:
        value: Number or string to check
        **kwargs: Optional parameters (none used)
    
    Returns:
        bool: True if value > 0, False otherwise
    """
    # Try to convert to number
    try:
        num = float(value)
        return num > 0
    except (ValueError, TypeError):
        # Not a valid number
        return False

