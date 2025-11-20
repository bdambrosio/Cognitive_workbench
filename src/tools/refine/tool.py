#!/usr/bin/env python3
"""
Universal LLM-based Note transformation tool.
"""
import logging
import logging
from llm_client import ZenohLLMClient
llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)


def tool(value, runtime=None, **kwargs):
    """
    Transform Note content using natural language instruction.
    
    Args:
        value: Note content to transform
        **kwargs: Additional parameters
            - instruction: Natural language instruction (REQUIRED)
            - llm_client: LLM client instance (optional, creates default if not provided)
    
    Returns:
        Transformed content as string
    """
    instruction = kwargs.get('instruction')
    if not instruction:
        return "Error: instruction parameter required"
    
    if not value:
        return "Error: value parameter required"
    
    # Segment long content into chunks
    chunks = _segment_text(value, max_chunk_size=16000)
    logger.info(f"refine: {instruction[:50]}... ({len(chunks)} chunks)")
    
    results = []
    for i, (chunk_text, delimiter) in enumerate(chunks):
        # Build prompt for this chunk
        prompt = f"""Transform the following content according to this instruction:

Instruction: {instruction}

Content:
{chunk_text}

Return only the transformed result, no explanation.
Do not include any introductory, reasoning, code fences, or explanatory text in your response. Only provide the transformed result, followed by the </end> tag.
End your response with:
</end>
"""
        
        # Calculate max_tokens based on chunk size
        max_tokens = len(chunk_text) // 2
        
        # Use unified llm_generate callback if available, else fall back to llm_client
        llm_generate = kwargs.get('llm_generate')
        if llm_generate:
            response = llm_generate(
                messages=[prompt],
                max_tokens=max_tokens,
                temperature=0.4,
                is_json=False,
                stops=['</end>']
            )
        else:
            response = llm_client.generate(
                messages=[prompt],
                max_tokens=max_tokens,
                temperature=0.4,
                is_json=False,
                stops=['</end>']
            )
        
        # Send heartbeat after LLM call to reset timeout
        heartbeat = kwargs.get('heartbeat')
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"refine chunk {i+1}/{len(chunks)} failed: {response.error}")
            return f"Error: {response.error}"
        
        results.append(response.text)
        # Append delimiter if it's not the last chunk
        if delimiter and i < len(chunks) - 1:
            results.append(delimiter)
    
    result = ''.join(results)
    logger.info(f"refine complete: output_len={len(result)}")
    
    return result


def _segment_text(text, max_chunk_size=16000):
    """
    Segment text into chunks at sentence or word boundaries.
    
    Returns list of (chunk_text, delimiter) tuples where delimiter
    is the separator found at the split point (to preserve on concatenation).
    """
    if len(text) <= max_chunk_size:
        return [(text, None)]
    
    chunks = []
    pos = 0
    
    while pos < len(text):
        # Determine chunk end position
        end_pos = min(pos + max_chunk_size, len(text))
        
        if end_pos >= len(text):
            # Last chunk
            chunks.append((text[pos:], None))
            break
        
        # Try to find sentence boundary near end_pos
        # Look backward up to 500 chars for ". " or ".\n"
        search_start = max(pos, end_pos - 500)
        best_split = -1
        best_delimiter = None
        
        # Search for sentence boundaries
        for i in range(end_pos, search_start, -1):
            if i < len(text) - 1:
                if text[i] == '.' and text[i+1] in (' ', '\n'):
                    best_split = i + 1  # Include the period
                    best_delimiter = text[i+1]  # Space or newline
                    break
        
        # Fall back to word boundary if no sentence boundary found
        if best_split == -1:
            for i in range(end_pos, search_start, -1):
                if text[i] in (' ', '\n', '\t'):
                    best_split = i
                    best_delimiter = text[i]
                    break
        
        # Fall back to hard split if no boundary found
        if best_split == -1:
            best_split = end_pos
            best_delimiter = ''
        
        # Extract chunk (up to but not including delimiter)
        chunk_text = text[pos:best_split]
        chunks.append((chunk_text, best_delimiter))
        
        # Move position past the delimiter
        pos = best_split + (1 if best_delimiter else 0)
    
    return chunks


