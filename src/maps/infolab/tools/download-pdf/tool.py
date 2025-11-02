#!/usr/bin/env python3
"""
Download PDF from URL.
"""
import base64
import json
import re
import logging
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError
from urllib.parse import urljoin, urlparse

logger = logging.getLogger(__name__)


def tool(url: str, **kwargs) -> str:
    """
    Download PDF from URL and return base64 encoded content.
    
    Args:
        url: URL to download PDF from, or JSON string containing URL field
        
    Returns:
        Base64 encoded PDF content
    """
    if not url or not isinstance(url, str):
        return "Error: url parameter required"
    
    # Try to extract URL from JSON if input looks like JSON
    extracted_url = url.strip()
    if extracted_url.startswith('{') or extracted_url.startswith('['):
        try:
            parsed = json.loads(extracted_url)
            if isinstance(parsed, dict):
                # Look for 'url' field at any level
                if 'url' in parsed:
                    extracted_url = parsed['url']
                else:
                    # Try recursive search for url field
                    def find_url(obj):
                        if isinstance(obj, dict):
                            if 'url' in obj:
                                return obj['url']
                            for v in obj.values():
                                result = find_url(v)
                                if result:
                                    return result
                        elif isinstance(obj, list):
                            for item in obj:
                                result = find_url(item)
                                if result:
                                    return result
                        return None
                    found = find_url(parsed)
                    if found:
                        extracted_url = found
        except (json.JSONDecodeError, TypeError):
            pass
    
    # Fallback: regex to find URL in string
    if not extracted_url.startswith(('http://', 'https://')):
        url_match = re.search(r'https?://[^\s<>"\'{}|\\^`\[\]]+', extracted_url)
        if url_match:
            extracted_url = url_match.group(0)
    
    # Final validation
    if not extracted_url.startswith(('http://', 'https://')):
        return f"Error: Invalid URL scheme: {extracted_url[:100]}"
    
    # Normalize known PDF URL patterns
    extracted_url = _normalize_pdf_url(extracted_url)
    
    # Download and check content
    headers = {'User-Agent': 'Mozilla/5.0 (compatible; CognitiveWorkbench/1.0)'}
    
    try:
        request = Request(extracted_url, headers=headers)
        response = urlopen(request, timeout=30)
        content = response.read()
        
        # Check if it's a PDF
        if content.startswith(b'%PDF'):
            # It's a PDF - encode and return
            encoded = base64.b64encode(content).decode('utf-8')
            logger.info(f"Downloaded PDF from {extracted_url}, size: {len(content)} bytes")
            return encoded
        
        # Not a PDF - check if it's HTML
        if content.startswith(b'<') or b'<html' in content[:1024].lower():
            # It's HTML - try to extract PDF link
            pdf_url = _extract_pdf_from_html(content, extracted_url)
            if pdf_url:
                logger.info(f"Found PDF link in HTML: {pdf_url}, downloading...")
                # Retry with extracted PDF URL
                request = Request(pdf_url, headers=headers)
                response = urlopen(request, timeout=30)
                content = response.read()
                
                if content.startswith(b'%PDF'):
                    encoded = base64.b64encode(content).decode('utf-8')
                    logger.info(f"Downloaded PDF from extracted URL {pdf_url}, size: {len(content)} bytes")
                    return encoded
                else:
                    return f"Error: Extracted URL {pdf_url} is not a PDF"
        
        # Not PDF and not HTML
        return f"Error: Downloaded content is not a PDF"
        
    except HTTPError as e:
        return f"Error: HTTP {e.code} {e.reason}"
    except URLError as e:
        return f"Error: URL error: {e.reason}"
    except Exception as e:
        return f"Error: Download failed: {str(e)}"


def _normalize_pdf_url(url: str) -> str:
    """
    Normalize known PDF URL patterns to direct PDF URLs.
    
    Examples:
    - arxiv.org/abs/1706.03762 → arxiv.org/pdf/1706.03762
    """
    # ArXiv: convert /abs/ to /pdf/
    if 'arxiv.org/abs/' in url:
        url = url.replace('/abs/', '/pdf/')
        # Ensure it ends with .pdf for ArXiv
        if not url.endswith('.pdf'):
            url = url + '.pdf'
    
    return url


def _extract_pdf_from_html(html_content: bytes, base_url: str) -> str:
    """
    Extract PDF URL from HTML content.
    
    Looks for:
    - Links containing '.pdf'
    - Links containing '/pdf/'
    - download links with pdf in href
    
    Args:
        html_content: HTML content as bytes
        base_url: Original URL for resolving relative links
        
    Returns:
        PDF URL if found, None otherwise
    """
    try:
        html_str = html_content.decode('utf-8', errors='ignore')
        
        # Pattern 1: href="...pdf" or href="...pdf/"
        pdf_patterns = [
            r'href=["\']([^"\']*\.pdf[^"\']*)["\']',  # .pdf in href
            r'href=["\']([^"\']*\/pdf\/[^"\']*)["\']',  # /pdf/ in href
        ]
        
        for pattern in pdf_patterns:
            matches = re.finditer(pattern, html_str, re.IGNORECASE)
            for match in matches:
                pdf_url = match.group(1)
                # Resolve relative URLs
                pdf_url = urljoin(base_url, pdf_url)
                # Validate it looks like a PDF URL
                if '.pdf' in pdf_url.lower() or '/pdf/' in pdf_url.lower():
                    return pdf_url
        
        # Pattern 2: Look for download links
        download_pattern = r'href=["\']([^"\']+)["\'][^>]*download[^>]*>'
        matches = re.finditer(download_pattern, html_str, re.IGNORECASE)
        for match in matches:
            download_url = match.group(1)
            download_url = urljoin(base_url, download_url)
            if '.pdf' in download_url.lower() or '/pdf/' in download_url.lower():
                return download_url
        
        return None
    except Exception as e:
        logger.warning(f"Error extracting PDF from HTML: {e}")
        return None

