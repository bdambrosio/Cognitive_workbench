#!/usr/bin/env python3
"""
Extract text from PDF using pymupdf.
"""
import base64
import json
import logging
import pymupdf

logger = logging.getLogger(__name__)


def tool(pdf_content: str, **kwargs) -> str:
    """
    Extract text from base64 encoded PDF.
    
    Args:
        pdf_content: Base64 encoded PDF binary data
        
    Returns:
        JSON string with extracted text and metadata
    """
    if not pdf_content or not isinstance(pdf_content, str):
        return "Error: pdf_content parameter required"
    
    # Decode base64
    pdf_bytes = base64.b64decode(pdf_content)
    
    # Open PDF with pymupdf
    doc = pymupdf.open(stream=pdf_bytes, filetype="pdf")
    
    # Extract text from all pages
    pages_text = []
    for page_num in range(len(doc)):
        page = doc[page_num]
        text = page.get_text()
        pages_text.append(text)
    
    full_text = "\n\n".join(pages_text)
    
    # Get basic metadata
    metadata = doc.metadata or {}
    page_count = len(doc)
    
    result = {
        "text": full_text,
        "page_count": page_count,
        "pdf_metadata": {
            "title": metadata.get("title", ""),
            "author": metadata.get("author", ""),
            "subject": metadata.get("subject", ""),
            "creator": metadata.get("creator", "")
        }
    }
    doc.close()
    
    logger.info(f"Extracted {len(full_text)} chars from {page_count} page PDF")
    return json.dumps(result, indent=2)

