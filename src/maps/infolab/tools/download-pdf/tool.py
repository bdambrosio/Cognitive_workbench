#!/usr/bin/env python3
"""
Download PDF from URL.
"""
import base64
import logging
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

logger = logging.getLogger(__name__)


def tool(url: str, **kwargs) -> str:
    """
    Download PDF from URL and return base64 encoded content.
    
    Args:
        url: URL to download PDF from
        
    Returns:
        Base64 encoded PDF content
    """
    if not url or not isinstance(url, str):
        return "Error: url parameter required"
    
    if not url.startswith(('http://', 'https://')):
        return f"Error: Invalid URL scheme: {url}"
    
    # Download PDF
    headers = {'User-Agent': 'Mozilla/5.0 (compatible; CognitiveWorkbench/1.0)'}
    request = Request(url, headers=headers)
    
    response = urlopen(request, timeout=30)
    content = response.read()
    
    # Verify it's a PDF
    if not content.startswith(b'%PDF'):
        return f"Error: Downloaded content is not a PDF"
    
    # Encode as base64
    encoded = base64.b64encode(content).decode('utf-8')
    
    logger.info(f"Downloaded PDF from {url}, size: {len(content)} bytes")
    return encoded

