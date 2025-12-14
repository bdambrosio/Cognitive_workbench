"""
Text chunking utilities for LLM processing.

Provides boundary-aware text segmentation to split long documents
into manageable chunks while preserving sentence/word boundaries.
"""

from typing import List, Tuple, Optional


def segment_text_boundary_aware(text: str, max_chunk_size: int = 16000) -> List[Tuple[str, Optional[str]]]:
    """
    Segment text into chunks at sentence or word boundaries.
    
    Attempts to split at natural boundaries to preserve semantic meaning:
    1. Sentence boundaries (". " or ".\\n")
    2. Word boundaries (space, newline, tab)
    3. Hard split (if no boundary found)
    
    Args:
        text: Text to segment
        max_chunk_size: Maximum characters per chunk (default: 16000)
    
    Returns:
        List of (chunk_text, delimiter) tuples where delimiter is the
        separator found at the split point (to preserve on concatenation).
        Delimiter is None for the last chunk.
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
        # Look backward up to 500 chars for ". " or ".\\n"
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

