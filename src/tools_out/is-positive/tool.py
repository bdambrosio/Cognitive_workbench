"""
Is-positive tool - example Python condition tool.
"""

def tool(input_value, runtime=None, **kwargs):
    """
    Check if input_value is a positive number.
    
    Args:
        input_value: Number or string to check
        **kwargs: Optional parameters (none used)
    
    Returns:
        bool: True if input_value > 0, False otherwise
    """
    # Try to convert to number
    try:
        num = float(input_value)
        return num > 0
    except (ValueError, TypeError):
        # Not a valid number
        return False

