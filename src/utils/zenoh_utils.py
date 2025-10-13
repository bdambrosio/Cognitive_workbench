#!/usr/bin/env python3
"""
Zenoh utility functions for common operations across nodes.
"""

from datetime import datetime, timedelta
from typing import Any, Dict, Iterable, Optional
import json
import logging
import random

logger = logging.getLogger('zenoh_utils')


def datetime_handler(obj):
    if isinstance(obj, datetime):
        return obj.isoformat()  # More standard format
    elif isinstance(obj, timedelta):
        return obj.total_seconds()
    else:
        return str(obj) 


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


def zenoh_get_with_retry(session: Any,
                         key_expr: str,
                         payload: Optional[bytes] = None,
                         base_timeout: float = 1.0,
                         retries: int = 3,
                         backoff_factor: float = 2.0,
                         jitter: float = 0.1,
                         block_mode: bool = False,
                         block_timeout: float = 300.0) -> Optional[Dict[str, Any]]:
    """Perform a Zenoh get with exponential backoff and graceful fallback.

    Returns parsed JSON dict on first ok reply, or None on timeout/exhaustion.
    In block_mode, makes a single long attempt (intended for debugger breakpoints).
    """
    try:
        if block_mode:
            logger.warning(f"Zenoh get in BLOCK mode, timeout={block_timeout:.1f}s, key={key_expr[:60]}...")
            for reply in session.get(key_expr, payload=payload, timeout=block_timeout):
                if reply.ok:
                    try:
                        return json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                    except Exception as e:
                        logger.error(f"Failed to parse Zenoh reply JSON (block mode): {e}")
                        return None
            return None

        attempt = 0
        timeout = base_timeout
        while attempt < retries:
            attempt += 1
            try:
                if attempt > 1:  # Only log retries, not first attempt
                    logger.debug(f"Zenoh get retry {attempt}/{retries}, timeout={timeout:.2f}s, key={key_expr[:60]}...")
                replies: Iterable = session.get(key_expr, payload=payload, timeout=timeout)
                got_any = False
                for reply in replies:
                    got_any = True
                    if reply.ok:
                        try:
                            data = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
                            return data
                        except Exception as e:
                            logger.error(f"Failed to parse Zenoh reply JSON: {e}")
                            return None
                if not got_any and attempt == retries:  # Only log on final failure
                    logger.warning(f"No Zenoh replies after {retries} attempts for key={key_expr[:60]}...")
            except Exception as e:
                if attempt == retries:  # Only log error on final attempt
                    logger.error(f"Zenoh get error after {retries} attempts for key={key_expr[:60]}...: {e}")

            sleep_jitter = random.uniform(-jitter, jitter)
            timeout = max(0.1, timeout * backoff_factor + sleep_jitter)

        return None
    except Exception as e:
        logger.error(f"zenoh_get_with_retry fatal error: {e}")
        return None