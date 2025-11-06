#!/usr/bin/env python3
"""
Summarization tool with focus-aware compression and adaptive styling.
"""
import logging
import json
import zenoh
from zenoh import QueryTarget, ConsolidationMode
from llm_client import ZenohLLMClient

llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
logger = logging.getLogger(__name__)

# Leaky filter threshold (4/10 = 40% relevance)
LEAKY_THRESHOLD = 4

# Open zenoh session for fetching Collection/Note content
config = zenoh.Config()
zenoh_session = zenoh.open(config)


def _get_content(resource_id: str) -> any:
    """Fetch content from map_node for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    for reply in zenoh_session.get(
        f"cognitive/map/resource/{resource_id}",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                if 'resource' in response:
                    resource_data = response.get('resource')
                    if resource_data:
                        return resource_data.get('properties', {}).get('content')
                else:
                    return response.get('content')
        break
    return None


def _flatten_list(items: list, separator: str = '\n\n') -> str:
    """Flatten a list (Collection content) into concatenated Note content."""
    note_contents = []
    for item in items:
        if isinstance(item, str) and item.startswith('Note_'):
            note_content = _get_content(item)
            if note_content is not None:
                note_contents.append(str(note_content))
        elif isinstance(item, str) and item.startswith('Collection_'):
            # Recursively flatten nested Collections
            flattened = _flatten_collection(item, separator)
            if flattened:
                note_contents.append(flattened)
        else:
            note_contents.append(str(item))
    
    return separator.join(note_contents)


def _flatten_collection(collection_id: str, separator: str = '\n\n') -> str:
    """Flatten a Collection into concatenated Note content."""
    content = _get_content(collection_id)
    if not isinstance(content, list):
        return str(content) if content else ""
    
    return _flatten_list(content, separator)


def _estimate_tokens(text):
    """Rough token estimation: ~4 chars per token"""
    return len(text) // 4


def _segment_for_filtering(text, max_size=2048):
    """
    Segment text into small chunks for filtering, preferring paragraph boundaries.
    Falls back to sentence then word boundaries for oversized paragraphs.
    Returns list of chunk strings.
    """
    if len(text) <= max_size:
        return [text]
    
    # Split by paragraphs first
    paragraphs = text.split('\n\n')
    
    chunks = []
    current_chunk = []
    current_size = 0
    
    for para in paragraphs:
        para_size = len(para)
        
        # If paragraph exceeds max_size, split it further
        if para_size > max_size:
            # Flush current chunk if it has content
            if current_chunk:
                chunks.append('\n\n'.join(current_chunk))
                current_chunk = []
                current_size = 0
            
            # Split oversized paragraph by sentences
            sentences = para.split('. ')
            sentence_chunk = []
            sentence_size = 0
            
            for sentence in sentences:
                sentence_len = len(sentence)
                
                # If sentence itself exceeds max_size, split by words
                if sentence_len > max_size:
                    words = sentence.split(' ')
                    word_chunk = []
                    word_size = 0
                    
                    for word in words:
                        word_len = len(word) + 1  # +1 for space
                        
                        if word_size + word_len > max_size and word_chunk:
                            chunks.append(' '.join(word_chunk))
                            word_chunk = [word]
                            word_size = word_len
                        else:
                            word_chunk.append(word)
                            word_size += word_len
                    
                    if word_chunk:
                        chunks.append(' '.join(word_chunk))
                    
                elif sentence_size + sentence_len > max_size and sentence_chunk:
                    chunks.append('. '.join(sentence_chunk))
                    sentence_chunk = [sentence]
                    sentence_size = sentence_len
                else:
                    sentence_chunk.append(sentence)
                    sentence_size += sentence_len + 2  # +2 for '. '
            
            if sentence_chunk:
                chunks.append('. '.join(sentence_chunk))
        
        # Normal paragraph handling
        elif current_size + para_size > max_size and current_chunk:
            chunks.append('\n\n'.join(current_chunk))
            current_chunk = [para]
            current_size = para_size
        else:
            current_chunk.append(para)
            current_size += para_size + 2  # +2 for '\n\n'
    
    # Add final chunk
    if current_chunk:
        chunks.append('\n\n'.join(current_chunk))
    
    return chunks


def _score_relevance(chunk_text, focus, heartbeat=None):
    """
    Score chunk relevance to focus topic (0-10).
    Returns score, defaults to 5 if scoring fails.
    """
    filter_prompt = f"""Rate the relevance of this section to the topic: "{focus}"

Score 0-10 where:
- 0-3: Not relevant
- 4-6: Somewhat relevant or provides useful context
- 7-10: Highly relevant

If a section is directly relevant to "{focus}", or provides context, discusses related concepts, or contains cross-references, score it >= 4.

Section:
{chunk_text}

Respond with ONLY a number 0-10, followed by the </end> tag. End your response with:
</end>
"""
    
    response = llm_client.generate(
        messages=[filter_prompt],
        max_tokens=5,
        temperature=0.1,
        is_json=False,
        stops=['</end>']
    )
    
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.warning(f"Filter failed, including chunk by default: {response.error}")
        return 5  # Default: include
    
    # Parse score
    score_text = response.text.strip()
    score = 5  # default if parsing fails
    for char in score_text:
        if char.isdigit():
            score = int(char)
            break
    
    return score


def _apply_leaky_filter(text, focus, heartbeat=None):
    """
    Apply leaky relevance filter to text using fine-grained chunks.
    
    Args:
        text: Full text string (flattened Collection)
        focus: Topic to filter by
        heartbeat: Optional heartbeat callback
    
    Returns:
        Filtered text string (concatenated filtered chunks)
    """
    # Segment into small filter chunks (2048 chars, paragraph boundaries)
    filter_chunks = _segment_for_filtering(text, max_size=2048)
    
    logger.debug(f"Filtering {len(filter_chunks)} small chunks at 2048-char granularity")
    
    # Filter each small chunk
    filtered_parts = []
    included_count = 0
    
    for i, chunk in enumerate(filter_chunks):
        score = _score_relevance(chunk, focus, heartbeat)
        
        if score >= LEAKY_THRESHOLD:
            filtered_parts.append(chunk)
            included_count += 1
            logger.debug(f"Chunk {i+1}/{len(filter_chunks)} included (score={score})")
        else:
            logger.debug(f"Chunk {i+1}/{len(filter_chunks)} excluded (score={score})")
    
    logger.info(f"Filter kept {included_count}/{len(filter_chunks)} small chunks ({int(included_count/len(filter_chunks)*100)}%)")
    
    # Reassemble into larger text
    return "\n\n".join(filtered_parts)


def _compute_target_length(effective_tokens, style, compression_ratio):
    """Compute target summary length based on style and compression ratio."""
    if style == 'executive':
        # Fixed cap for executive summaries
        return min(500, effective_tokens // 3)
    elif style == 'comprehensive':
        # Low compression for detailed summaries
        return effective_tokens // 2.0
    else:  # technical (default)
        # Use specified compression ratio
        target = effective_tokens / compression_ratio
        # Floor: never go below 300 tokens for very small inputs
        return max(target, 300)


def tool(value, **kwargs):
    """
    Summarize content with focus-aware compression and adaptive styling.
    
    Args:
        value: Text content to summarize, or list (Collection) which will be flattened
        **kwargs: Optional parameters
            - focus: Topic to guide summarization (optional)
            - style: Output style - 'technical' (default), 'executive', or 'comprehensive'
            - compression_ratio: Compression factor (default 3.0), applied to focused content
    
    Returns:
        Summary as string
    """
    if not value:
        return "No content to summarize"
    
    # Extract parameters
    focus = kwargs.get('focus', '')
    style = kwargs.get('style', 'technical')
    compression_ratio = kwargs.get('compression_ratio', 3.0)
    heartbeat = kwargs.get('heartbeat')
    
    # Validate style
    if style not in ['technical', 'executive', 'comprehensive']:
        logger.warning(f"Unknown style '{style}', using 'technical'")
        style = 'technical'
    
    # Flatten Collection if input is a list
    if isinstance(value, list):
        logger.info(f"summarize: flattening Collection with {len(value)} items")
        value = _flatten_list(value)
    
    # Measure input
    input_tokens = _estimate_tokens(value)
    
    # Apply leaky focus filter if focus provided (filter before segmentation for fine granularity)
    effective_tokens = input_tokens
    inclusion_pct = 100
    
    if focus:
        logger.info(f"summarize: applying leaky focus filter for '{focus}'")
        filtered_text = _apply_leaky_filter(value, focus, heartbeat)
        
        if not filtered_text or not filtered_text.strip():
            logger.warning(f"Focus '{focus}' yielded no content, using all content")
            filtered_text = value
        else:
            # Recompute effective tokens from filtered text
            effective_tokens = _estimate_tokens(filtered_text)
            inclusion_pct = int((effective_tokens / input_tokens) * 100) if input_tokens > 0 else 100
        value = filtered_text
    
    # Check if segmentation needed (after filtering)
    chunks = _segment_text(value, max_chunk_size=16000)
    chunk_count = len(chunks)
    
    # Compute target length
    target_tokens = int(_compute_target_length(effective_tokens, style, compression_ratio))
    
    # Style-specific instructions
    style_instructions = {
        'executive': 'Provide a high-level executive summary focused on key findings and implications.',
        'technical': 'Provide a technical summary preserving key details, methodology, and caveats.',
        'comprehensive': 'Provide a comprehensive summary preserving nuance, technical details, and supporting evidence.'
    }
    style_instruction = style_instructions.get(style, style_instructions['technical'])
    
    focus_guidance = f"\nFocus on: {focus}" if focus else ""
    
    # Log summarization parameters
    logger.info(f"summarize: input={input_tokens}t, focus={'yes' if focus else 'no'}, "
               f"filtered={effective_tokens}t ({inclusion_pct}%), target={target_tokens}t, "
               f"style={style}, ratio={compression_ratio:.1f}")
    
    if chunk_count == 1:
        # Single chunk - direct summarization
        chunk_text = chunks[0][0]
        
        prompt = f"""{style_instruction}{focus_guidance}

Content:
{chunk_text}

Target length: approximately {target_tokens} tokens. Provide a summary highlighting key points.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x final output limit (2000 tokens)
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=3000,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize failed: {response.error}")
            return f"Error: {response.error}"
        
        output_tokens = _estimate_tokens(response.text)
        logger.info(f"summarize: output={output_tokens}t")
        
        return response.text
    
    # Multiple chunks - hierarchical summarization
    logger.info(f"summarize: hierarchical summarization ({chunk_count} chunks)")
    
    # Distribute target across chunks
    tokens_per_chunk = target_tokens // chunk_count
    tokens_per_chunk = max(tokens_per_chunk, 100)  # Floor per chunk
    
    # Step 1: Summarize each chunk
    chunk_summaries = []
    for i, (chunk_text, _) in enumerate(chunks):
        prompt = f"""{style_instruction}{focus_guidance}

Section:
{chunk_text}

Target length: approximately {tokens_per_chunk} tokens. Provide key points.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x target per chunk
        max_chunk_tokens = int(tokens_per_chunk * 1.5)
        response = llm_client.generate(
            messages=[prompt],
            max_tokens=max_chunk_tokens,
            temperature=0.3,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize chunk {i+1}/{chunk_count} failed: {response.error}")
            return f"Error: {response.error}"
        
        chunk_summaries.append(response.text)
    
    # Step 2: Synthesize chunk summaries into final summary
    combined = "\n\n".join([f"Section {i+1}: {s}" for i, s in enumerate(chunk_summaries)])
    
    synthesis_prompt = f"""{style_instruction}{focus_guidance}

Synthesize these section summaries into a coherent overall summary.

Section Summaries:
{combined}

Target length: approximately {target_tokens} tokens. Provide a unified summary that captures the key themes and findings.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
"""
    
    # max_tokens = 1.5x final output limit (2000 tokens)
    response = llm_client.generate(
        messages=[synthesis_prompt],
        max_tokens=3000,
        temperature=0.3,
        is_json=False,
        stops=['</end>']
    )
    
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"summarize synthesis failed: {response.error}")
        return f"Error: {response.error}"
    
    output_tokens = _estimate_tokens(response.text)
    logger.info(f"summarize: output={output_tokens}t")
    
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

