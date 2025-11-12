#!/usr/bin/env python3
"""
Fetch text from URL or base64 PDF. Auto-detects format (PDF/HTML/MD/TXT) and extracts all text.
"""
import base64
import json
import re
import logging
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError
from urllib.parse import urljoin, urlparse
from html.parser import HTMLParser
import pymupdf
import warnings

try:
    from unstructured.partition.html import partition_html
    HAS_UNSTRUCTURED = True
except ImportError:
    HAS_UNSTRUCTURED = False

try:
    from playwright.sync_api import sync_playwright
    HAS_PLAYWRIGHT = True
except ImportError:
    HAS_PLAYWRIGHT = False

logger = logging.getLogger(__name__)
warnings.filterwarnings('ignore', category=UserWarning, module='unstructured')


def tool(url_or_content: str, **kwargs) -> str:
    """
    Fetch text from URL or base64 PDF. Auto-detects format and extracts all text.
    Collection-aware: If input is Collection ID, fetches first item's URL.
    
    Args:
        url_or_content: URL string, base64-encoded PDF content, or Collection ID
        
    Returns:
        JSON string with text, format, metadata, page_count (if PDF), char_count
    """
    if not url_or_content or not isinstance(url_or_content, str):
        return json.dumps({"error": "url_or_content parameter required"})
    
    # Check if input is Collection ID - extract first item
    if url_or_content.startswith('Collection_'):
        try:
            from infospace_resource_manager import InfospaceResourceManager
            resource_mgr = kwargs.get('resource_manager')
            if not resource_mgr:
                # Try to get from world_map
                world_map = kwargs.get('world_map')
                if world_map and hasattr(world_map, 'resource_registry'):
                    collection = world_map.resource_registry.get(url_or_content)
                    if collection and collection.get('type') == 'collection':
                        note_ids = collection.get('content', [])
                        if len(note_ids) == 0:
                            return json.dumps({"error": "Collection is empty"})
                        if len(note_ids) > 1:
                            logger.warning(f"fetch-text: Collection {url_or_content} has {len(note_ids)} items, using first")
                        first_note_id = note_ids[0]
                        # Get Note content
                        note = world_map.resource_registry.get(first_note_id)
                        if note:
                            url_or_content = note.get('content', '')
                        else:
                            return json.dumps({"error": f"Note {first_note_id} not found"})
                    else:
                        return json.dumps({"error": f"Collection {url_or_content} not found"})
                else:
                    return json.dumps({"error": "Cannot resolve Collection without world_map"})
        except Exception as e:
            logger.error(f"Failed to resolve Collection: {e}")
            return json.dumps({"error": f"Failed to resolve Collection: {e}"})
    
    # Check if input is base64 PDF (backward compatibility)
    if _is_base64_pdf(url_or_content):
        return _process_base64_pdf(url_or_content)
    
    # Otherwise treat as URL
    url = _extract_url(url_or_content)
    if not url:
        return json.dumps({"error": f"Invalid URL or content: {url_or_content[:100]}"})
    
    # Download and detect format
    content, content_type, final_url = _download_from_url(url)
    if not content:
        return json.dumps({"error": f"Failed to download from {url}"})
    
    # Detect format
    file_format = _detect_format(content, content_type, final_url)
    
    # Extract text based on format
    if file_format == "pdf":
        return _extract_pdf_text(content, final_url)
    elif file_format == "html":
        return _extract_html_text(content, final_url)
    elif file_format == "markdown":
        return _extract_text_plain(content, final_url, "markdown")
    else:
        return _extract_text_plain(content, final_url, "text")


def _is_base64_pdf(content: str) -> bool:
    """Check if content is base64-encoded PDF."""
    if len(content) < 100:
        return False
    try:
        decoded = base64.b64decode(content[:200])
        return decoded.startswith(b'%PDF')
    except:
        return False


def _extract_url(url_str: str) -> str:
    """Extract URL from string, handling JSON wrappers and embedded URLs."""
    extracted_url = url_str.strip()
    
    # Try JSON parsing
    if extracted_url.startswith('{') or extracted_url.startswith('['):
        try:
            parsed = json.loads(extracted_url)
            if isinstance(parsed, dict):
                if 'url' in parsed:
                    return parsed['url']
                # Recursive search
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
                    return found
        except (json.JSONDecodeError, TypeError):
            pass
    
    # Regex fallback
    if not extracted_url.startswith(('http://', 'https://')):
        url_match = re.search(r'https?://[^\s<>"\'{}|\\^`\[\]]+', extracted_url)
        if url_match:
            extracted_url = url_match.group(0)
    
    # Validate
    if not extracted_url.startswith(('http://', 'https://')):
        return None
    
    return _normalize_url(extracted_url)


def _normalize_url(url: str) -> str:
    """Normalize known URL patterns."""
    # ArXiv: convert /abs/ to /pdf/
    if 'arxiv.org/abs/' in url:
        url = url.replace('/abs/', '/pdf/')
        if not url.endswith('.pdf'):
            url = url + '.pdf'
    # GitHub: convert /blob/ to /raw/ to get raw content instead of HTML
    if 'github.com' in url and '/blob/' in url:
        url = url.replace('/blob/', '/raw/')
    return url


def _download_from_url(url: str) -> tuple:
    """Download content from URL. Returns (content_bytes, content_type, final_url)."""
    headers = {'User-Agent': 'Mozilla/5.0 (compatible; CognitiveWorkbench/1.0)'}
    
    try:
        request = Request(url, headers=headers)
        response = urlopen(request, timeout=30)
        content = response.read()
        content_type = response.headers.get('Content-Type', '').lower()
        final_url = response.url or url
        
        return content, content_type, final_url
    except HTTPError as e:
        logger.error(f"HTTP {e.code} {e.reason} for {url}")
        return None, None, url
    except URLError as e:
        logger.error(f"URL error: {e.reason} for {url}")
        return None, None, url
    except Exception as e:
        logger.error(f"Download failed: {str(e)} for {url}")
        return None, None, url


def _detect_format(content: bytes, content_type: str, url: str) -> str:
    """Detect file format from content, Content-Type header, and URL."""
    # Check magic bytes first (most reliable)
    if content.startswith(b'%PDF'):
        return "pdf"
    
    # Check Content-Type header (more reliable than URL extension)
    if 'pdf' in content_type:
        return "pdf"
    if 'markdown' in content_type:
        return "markdown"
    if 'html' in content_type:
        return "html"
    
    # Check content for HTML markers (override URL extension if HTML detected)
    content_start = content[:1024].decode('utf-8', errors='ignore').strip()
    if content_start.startswith('<!DOCTYPE html>') or content_start.startswith('<!doctype html>'):
        return "html"
    if content.startswith(b'<') or b'<html' in content[:1024].lower():
        return "html"
    
    # Check URL extension (least reliable - only if content didn't indicate HTML)
    url_lower = url.lower()
    if url_lower.endswith('.md') or url_lower.endswith('.markdown'):
        return "markdown"
    if url_lower.endswith('.html') or url_lower.endswith('.htm'):
        return "html"
    if url_lower.endswith('.txt'):
        return "text"
    
    # Check text/plain with markdown detection
    if 'text/plain' in content_type:
        try:
            text_sample = content[:2048].decode('utf-8', errors='ignore')
            if _looks_like_markdown(text_sample):
                return "markdown"
        except:
            pass
        return "text"
    
    # Check content for markdown patterns
    try:
        text_sample = content[:2048].decode('utf-8', errors='ignore')
        if _looks_like_markdown(text_sample):
            return "markdown"
    except:
        pass
    
    # Default to text
    return "text"


def _looks_like_markdown(text: str) -> bool:
    """Heuristic check if text looks like markdown."""
    if not text or len(text) < 20:
        return False
    
    # Check for markdown patterns
    markdown_indicators = [
        r'^#+\s',  # Headers
        r'^\*\s',  # Unordered lists
        r'^\d+\.\s',  # Ordered lists
        r'\[.*?\]\(.*?\)',  # Links
        r'```',  # Code fences
        r'\*\*.*?\*\*',  # Bold
        r'_.*?_',  # Italic
    ]
    
    lines = text.split('\n')[:50]  # Check first 50 lines
    matches = 0
    for line in lines:
        for pattern in markdown_indicators:
            if re.search(pattern, line):
                matches += 1
                break
    
    # If 10%+ of lines match markdown patterns, likely markdown
    return matches >= max(2, len(lines) * 0.1)


def _process_base64_pdf(content: str) -> str:
    """Process base64 PDF content (backward compatibility)."""
    try:
        pdf_bytes = base64.b64decode(content)
        return _extract_pdf_text(pdf_bytes, "")
    except Exception as e:
        return json.dumps({"error": f"Failed to decode PDF: {str(e)}"})


def _extract_pdf_text(content: bytes, url: str) -> str:
    """Extract text from PDF bytes."""
    try:
        doc = pymupdf.open(stream=content, filetype="pdf")
        pages_text = []
        for page_num in range(len(doc)):
            page = doc[page_num]
            text = page.get_text()
            pages_text.append(text)
        
        full_text = "\n\n".join(pages_text)
        metadata = doc.metadata or {}
        page_count = len(doc)
        doc.close()
        
        result = {
            "text": full_text,
            "format": "pdf",
            "metadata": {
                "source_url": url,
                "pdf_metadata": {
                    "title": metadata.get("title", ""),
                    "author": metadata.get("author", ""),
                    "subject": metadata.get("subject", ""),
                    "creator": metadata.get("creator", "")
                }
            },
            "page_count": page_count,
            "char_count": len(full_text)
        }
        
        logger.info(f"Extracted {len(full_text)} chars from {page_count} page PDF")
        return json.dumps(result, indent=2)
    except Exception as e:
        logger.error(f"PDF extraction failed: {str(e)}")
        return json.dumps({"error": f"PDF extraction failed: {str(e)}"})


def _extract_html_text(content: bytes, url: str) -> str:
    """Extract text from HTML content."""
    html_str = content.decode('utf-8', errors='ignore')
    extraction_method = "static_html"
    
    # Try normal extraction first
    if HAS_UNSTRUCTURED:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            elements = partition_html(text=html_str)
        text_parts = [str(e).strip() for e in elements if str(e).strip()]
        full_text = "\n".join(text_parts)
    else:
        class TextExtractor(HTMLParser):
            def __init__(self):
                super().__init__()
                self.text = []
            def handle_data(self, data):
                if data.strip():
                    self.text.append(data.strip())
        parser = TextExtractor()
        parser.feed(html_str)
        full_text = "\n".join(parser.text)
    
    # If we got very little text, try playwright fallback (for SPAs)
    if len(full_text) < 100 and HAS_PLAYWRIGHT:
        logger.info(f"Low text content ({len(full_text)} chars) from {url}, trying playwright fallback")
        playwright_text = _try_playwright_extraction(url)
        if playwright_text and len(playwright_text) > len(full_text):
            full_text = playwright_text
            extraction_method = "playwright"
            logger.info(f"Playwright extraction succeeded: {len(full_text)} chars")
    
    # Extract basic HTML metadata
    html_metadata = {}
    title_match = re.search(r'<title[^>]*>(.*?)</title>', html_str, re.IGNORECASE | re.DOTALL)
    if title_match:
        html_metadata["title"] = re.sub(r'<[^>]+>', '', title_match.group(1)).strip()
    
    result = {
        "text": full_text,
        "format": "html",
        "metadata": {
            "source_url": url,
            "html_metadata": html_metadata,
            "extraction_method": extraction_method
        },
        "char_count": len(full_text)
    }
    
    logger.info(f"Extracted {len(full_text)} chars from HTML using {extraction_method}")
    return json.dumps(result, indent=2)


def _try_playwright_extraction(url: str) -> str:
    """Try extracting text using playwright (for SPAs)."""
    if not HAS_PLAYWRIGHT:
        return ""
    
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            page = browser.new_page()
            page.goto(url, wait_until='networkidle', timeout=30000)
            # Use inner_text to get rendered text content, not HTML markup
            full_text = page.inner_text('body')
            browser.close()
            return full_text
    except Exception as e:
        logger.warning(f"Playwright extraction failed for {url}: {str(e)}")
        return ""


def _extract_text_plain(content: bytes, url: str, format_type: str) -> str:
    """Extract text from plain text or markdown content."""
    try:
        text = content.decode('utf-8', errors='ignore')
        
        result = {
            "text": text,
            "format": format_type,
            "metadata": {
                "source_url": url
            },
            "char_count": len(text)
        }
        
        logger.info(f"Extracted {len(text)} chars from {format_type}")
        return json.dumps(result, indent=2)
    except Exception as e:
        logger.error(f"Text extraction failed: {str(e)}")
        return json.dumps({"error": f"Text extraction failed: {str(e)}"})

