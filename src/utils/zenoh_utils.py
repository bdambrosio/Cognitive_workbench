#!/usr/bin/env python3
"""
Zenoh utility functions for common operations across nodes.
"""


def decode_zenoh_error_payload(reply_str: str) -> str:
    """Decode hex payload from Zenoh error messages.
    
    Args:
        reply_str: String representation of Zenoh reply object
        
    Returns:
        Formatted string with decoded error message, or empty string if decoding fails
    """
    try:
        # Look for hex values in the format [54, 69, 6d, 65, 6f, 75, 74]
        if 'slices: [[' in reply_str:
            start = reply_str.find('slices: [[') + 10
            end = reply_str.find(']]', start)
            if start > 8 and end > start:
                hex_str = reply_str[start:end]
                # Extract hex values and convert to ASCII
                hex_values = [int(x.strip(), 16) for x in hex_str.split(',')]
                decoded = ''.join([chr(x) for x in hex_values])
                return f" (decoded: '{decoded}')"
    except:
        pass
    return "" 