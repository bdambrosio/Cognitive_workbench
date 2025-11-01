#!/usr/bin/env python3
"""
Summarization tool with automatic chunking for long documents.
"""
import logging
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)


def tool(value, **kwargs):
    """
    Summarize content with automatic chunking for long documents.
    
    Args:
        value: Text content to summarize
        **kwargs: Optional parameters
            - focus: Topic to guide summarization (optional)
    
    Returns:
        Summary as string
    """
    if not value:
        return "No content to summarize"
    
    focus = kwargs.get('focus', '')
    focus_guidance = f"\nFocus on: {focus}" if focus else ""
    
    # Check if segmentation needed
    chunks = _segment_text(value, max_chunk_size=16000)
    
    if len(chunks) == 1:
        # Single chunk - direct summarization
        prompt = f"""Summarize the following content concisely.{focus_guidance}

Content:
{value}

Provide a brief summary (3-5 sentences) highlighting key points."""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=500,
            temperature=0.3,
            is_json=False
        )
        
        if not response.success:
            logger.error(f"summarize failed: {response.error}")
            return f"Error: {response.error}"
        
        return response.text
    
    # Multiple chunks - hierarchical summarization
    logger.info(f"summarize: long document ({len(chunks)} chunks), using hierarchical approach")
    
    # Step 1: Summarize each chunk
    chunk_summaries = []
    for i, (chunk_text, _) in enumerate(chunks):
        prompt = f"""Summarize this section concisely.{focus_guidance}

Section:
{chunk_text}

Provide key points (2-4 sentences)."""
        
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=300,
            temperature=0.3,
            is_json=False
        )
        
        if not response.success:
            logger.error(f"summarize chunk {i+1}/{len(chunks)} failed: {response.error}")
            return f"Error: {response.error}"
        
        chunk_summaries.append(response.text)
    
    # Step 2: Synthesize chunk summaries into final summary
    combined = "\n\n".join([f"Section {i+1}: {s}" for i, s in enumerate(chunk_summaries)])
    
    synthesis_prompt = f"""Synthesize these section summaries into a coherent overall summary.{focus_guidance}

Section Summaries:
{combined}

Provide a unified summary (4-6 sentences) that captures the key themes and findings."""
    
    response = llm_client.generate(
        messages=[synthesis_prompt],
        max_tokens=500,
        temperature=0.3,
        is_json=False
    )
    
    if not response.success:
        logger.error(f"summarize synthesis failed: {response.error}")
        return f"Error: {response.error}"
    
    return response.text


def _segment_text(text, max_chunk_size=16000):
    """
    Segment text into chunks at sentence boundaries.
    Returns list of (chunk_text, delimiter) tuples.
    """
    if len(text) <= max_chunk_size:
        return [(text, None)]
    
    chunks = []
    pos = 0
    
    while pos < len(text):
        end_pos = min(pos + max_chunk_size, len(text))
        
        if end_pos >= len(text):
            chunks.append((text[pos:], None))
            break
        
        # Find sentence boundary
        search_start = max(pos, end_pos - 500)
        best_split = -1
        best_delimiter = None
        
        for i in range(end_pos, search_start, -1):
            if i < len(text) - 1 and text[i] == '.' and text[i+1] in (' ', '\n'):
                best_split = i + 1
                best_delimiter = text[i+1]
                break
        
        # Fall back to word boundary
        if best_split == -1:
            for i in range(end_pos, search_start, -1):
                if text[i] in (' ', '\n', '\t'):
                    best_split = i
                    best_delimiter = text[i]
                    break
        
        # Hard split if needed
        if best_split == -1:
            best_split = end_pos
            best_delimiter = ''
        
        chunk_text = text[pos:best_split]
        chunks.append((chunk_text, best_delimiter))
        pos = best_split + (1 if best_delimiter else 0)
    
    return chunks

