"""
Word count tool - example Python tool.
"""

def tool(value, runtime=None, **kwargs):
    """
    Count words in input text.
    
    Args:
        value: Input text (string)
        **kwargs: Optional parameters (none used)
    
    Returns:
        String with word count
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'Input must be a string'
        }
    
    # Count words
    words = value.split()
    count = len(words)
    
    return f"Word count: {count}"

