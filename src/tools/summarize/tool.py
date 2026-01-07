#!/usr/bin/env python3
"""
Summarization tool with focus-aware compression and adaptive styling.
"""
import logging
import json
import math
import zenoh
import time
import uuid
import traceback
from zenoh import QueryTarget, ConsolidationMode
from utils.text_chunking import segment_text_boundary_aware
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)


def _fail(executor: InfospaceExecutor, reason: str, value: str | None = None, extra: dict | None = None):
    return executor._create_uniform_return(
        "failed",
        value=value or reason,
        reason=reason,
        extra=extra,
    )


def _success(executor: InfospaceExecutor, result: str, extra: dict | None = None):
    return executor._create_uniform_return("success", value=result, extra=extra)


# Resource manager will be passed via kwargs


def _get_content(resource_id: str, resource_manager) -> any:
    """Fetch content for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if not resource:
        return None
    
    return resource.get('properties', {}).get('content')


def _flatten_list(items: list, resource_manager, separator: str = '\n\n') -> str:
    """Flatten a list (Collection content) into concatenated Note content."""
    note_contents = []
    for item in items:
        if isinstance(item, str) and item.startswith('Note_'):
            note_content = _get_content(item, resource_manager)
            if note_content is not None:
                note_contents.append(str(note_content))
        elif isinstance(item, str) and item.startswith('Collection_'):
            # Recursively flatten nested Collections
            flattened = _flatten_collection(item, resource_manager, separator)
            if flattened:
                note_contents.append(flattened)
        else:
            note_contents.append(str(item))
    
    return separator.join(note_contents)


def _flatten_collection(collection_id: str, resource_manager, separator: str = '\n\n') -> str:
    """Flatten a Collection into concatenated Note content."""
    content = _get_content(collection_id, resource_manager)
    if not isinstance(content, list):
        return str(content) if content else ""
    
    return _flatten_list(content, resource_manager, separator)


def _estimate_tokens(text):
    """Rough token estimation: ~4 chars per token"""
    return len(text) // 4


# _wait_for_response removed - no longer needed with direct method calls


def _semantic_filter_direct(text_content: str, focus: str, target_tokens: int, resource_manager) -> str:
    """
    Directly embed and search text chunks without creating temp Collections.
    Uses resource_manager's embedder.
    
    Args:
        text_content: Full text to filter
        focus: Query string for semantic filtering
        target_tokens: Approx number of tokens needed in result
        resource_manager: InfospaceResourceManager instance (for embedder access)
        
    Returns:
        Filtered text containing most relevant chunks
    """
    try:
        import numpy as np
        
        # Use resource_manager's embedder (ensure initialized)
        if not resource_manager:
            logger.warning("summarize: resource_manager not available, using all content")
            return text_content
        resource_manager._init_embedder()
        embedder = resource_manager.embedder
        
        # Chunk the text (paragraph-first, then sentences)
        chunks = []
        paragraphs = [p.strip() for p in text_content.split('\n\n') if p.strip()]
        
        for para in paragraphs:
            if len(para) < 1024:  # Small enough, use as-is
                chunks.append(para)
            else:
                # Split long paragraphs by sentences
                sentences = para.split('. ')
                current_chunk = []
                current_len = 0
                for sent in sentences:
                    sent_len = len(sent)
                    if current_len + sent_len > 1024 and current_chunk:
                        chunks.append('. '.join(current_chunk) + '.')
                        current_chunk = [sent]
                        current_len = sent_len
                    else:
                        current_chunk.append(sent)
                        current_len += sent_len
                if current_chunk:
                    chunks.append('. '.join(current_chunk) + ('.' if not chunks[-1].endswith('.') else ''))
        
        if not chunks:
            return text_content
        
        # Generate embeddings for all chunks and query
        logger.debug(f"summarize: embedding {len(chunks)} chunks for semantic filtering")
        chunk_embeddings = embedder.encode(chunks, convert_to_tensor=False, show_progress_bar=False)
        query_embedding = embedder.encode([focus], convert_to_tensor=False, show_progress_bar=False)[0]
        
        # Compute similarity scores (cosine similarity via normalized dot product)
        chunk_embeddings_norm = chunk_embeddings / np.linalg.norm(chunk_embeddings, axis=1, keepdims=True)
        query_embedding_norm = query_embedding / np.linalg.norm(query_embedding)
        similarities = np.dot(chunk_embeddings_norm, query_embedding_norm)
        
        # Filter by minimum similarity threshold (exclude low-relevance chunks)
        min_similarity = 0.3
        valid_indices = np.where(similarities >= min_similarity)[0]
        
        if len(valid_indices) == 0:
            # If no chunks meet threshold, use top 10% anyway (fallback)
            logger.debug(f"summarize: no chunks above similarity threshold {min_similarity}, using top 10%")
            valid_indices = np.argsort(similarities)[::-1][:max(1, len(chunks) // 10)]
        
        # Calculate average chunk size dynamically
        avg_chunk_tokens = sum(len(chunks[i]) // 4 for i in valid_indices) // len(valid_indices) if valid_indices.size > 0 else 256
        target_chunks = max(1, int(target_tokens / avg_chunk_tokens))
        limit = min(len(valid_indices), target_chunks * 3)  # 3x overshoot
        
        # Get top chunks from valid indices only
        valid_similarities = similarities[valid_indices]
        top_valid_idx = np.argsort(valid_similarities)[::-1][:limit]
        top_indices = valid_indices[top_valid_idx]
        
        # Collect chunks up to max tokens
        selected_chunks = []
        cumulative_tokens = 0
        max_tokens = target_tokens * 3
        
        for idx in top_indices:
            chunk = chunks[idx]
            chunk_tokens = len(chunk) // 4  # rough estimate
            if cumulative_tokens + chunk_tokens > max_tokens:
                break
            selected_chunks.append((idx, chunk))  # Keep index for ordering
            cumulative_tokens += chunk_tokens
        
        # Handle empty filtered results (fallback to original text)
        if not selected_chunks:
            logger.warning(f"summarize: semantic filter returned no chunks, using all content")
            return text_content
        
        # Sort by original order to maintain coherence
        selected_chunks.sort(key=lambda x: x[0])
        filtered_text = '\n\n'.join([chunk for _, chunk in selected_chunks])
        
        inclusion_pct = int(100 * len(filtered_text) / len(text_content)) if text_content else 0
        logger.debug(f"summarize: semantic filter kept {len(selected_chunks)}/{len(chunks)} chunks ({inclusion_pct}%)")
        
        return filtered_text
        
    except Exception as e:
        logger.warning(f"Semantic filtering failed: {e}, using all content")
        traceback.print_exc()
        return text_content


def _create_temp_collection(text_content: str, resource_manager, map_name: str = 'infolab') -> str:
    """Create temporary Collection with text content for indexing."""
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    # Create a Note with the text content
    success, note_id, error_msg, location = resource_manager.create_note(
        'summarize_tool', text_content, 'text', 'summarize_temp', str(text_content)[:100], '', {}
    )
    
    if not success:
        logger.error(f"Failed to create temporary Note: {error_msg}")
        return None
    
    # Create Collection containing the Note
    success, collection_id, error_msg, location = resource_manager.create_collection(
        'summarize_tool', [note_id], 'list', 'summarize_temp', '1 item', f'summarize_temp_{uuid.uuid4().hex[:8]}', {}
    )
    
    if not success:
        logger.error(f"Failed to create temporary Collection: {error_msg}")
        return None
    
    return collection_id


def _index_collection(collection_id: str, resource_manager, agent_name: str = 'summarize_tool') -> bool:
    """Index a Collection via resource_manager."""
    if not resource_manager:
        logger.error("resource_manager required for indexing")
        return False
    
    success, indexed_count, error_msg = resource_manager.index_collection(
        agent_name,
        collection_id,
        'semantic',
        {'content': 'embed'}
    )
    
    if not success:
        logger.error(f"Index failed: {error_msg}")
        return False
    
    return True


def _search_collection(collection_id: str, query: str, limit: int, resource_manager, agent_name: str = 'summarize_tool') -> list:
    """Search indexed Collection via resource_manager and return chunk results."""
    if not resource_manager:
        logger.error("resource_manager required for searching")
        return []
    
    success, results, error_msg = resource_manager.search_collection(
        agent_name,
        collection_id,
        query,
        'semantic',
        limit,
        0.0,
        'chunks'
    )
    
    if not success:
        logger.error(f"Search failed: {error_msg}")
        return []
    
    return results


def _compute_target_length(effective_tokens, style, compression_ratio):
    """Compute target summary length based on style and compression ratio."""
    if style == 'executive':
        # Fixed cap for executive summaries
        return min(1500, effective_tokens // 3)
    elif style == 'comprehensive':
        # Low compression for detailed summaries
        return min(int(math.sqrt(effective_tokens)*15+1500), effective_tokens // 2.0)
    else:  # technical (default)
        # Use specified compression ratio
        target = min(int(math.sqrt(effective_tokens)*15+1500), effective_tokens / compression_ratio)
        # Floor: never go below 300 tokens for very small inputs
        return max(target, 300)


def tool(input_value, runtime=None, **kwargs):
    """
    Summarize content with focus-aware compression and adaptive styling.
    
    Args:
        input_value: Text content to summarize, or list (Collection) which will be flattened
        **kwargs: Optional parameters
            - focus: Topic to guide summarization (optional)
            - style: Output style - 'technical' (default), 'executive', or 'comprehensive'
            - compression_ratio: Compression factor (default 3.0), applied to focused content
    
    Returns:
        Summary as string
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    if not input_value:
        return _fail(executor, "No content to summarize")
    
    # Extract parameters
    focus = kwargs.get('focus', '')
    style = kwargs.get('style', 'technical')
    compression_ratio = kwargs.get('compression_ratio', 3.0)
    heartbeat = kwargs.get('heartbeat')
    
    # Validate style
    if style not in ['technical', 'executive', 'comprehensive']:
        logger.warning(f"Unknown style '{style}', using 'technical'")
        style = 'technical'
    
    resource_manager = kwargs.get('resource_manager')
    
    # Flatten Collection if input is a list
    if isinstance(input_value, list):
        logger.debug(f"summarize: flattening Collection with {len(input_value)} items")
        input_value = _flatten_list(input_value, resource_manager)
    
    # Convert dict/object to JSON string if needed
    if isinstance(input_value, dict):
        import json
        logger.debug(f"summarize: converting dict to JSON string for processing")
        input_value = json.dumps(input_value, indent=2)
    elif not isinstance(input_value, str):
        # Convert any other type to string
        input_value = str(input_value)
    
    # Measure input
    input_chars = len(input_value)
    input_tokens = _estimate_tokens(input_value)
    
    # Apply focus filtering via index+search if focus provided
    effective_tokens = input_tokens
    filtered_chars = input_chars
    inclusion_pct = 100
    
    if focus:
        logger.debug(f"summarize: applying semantic search filter for '{focus}'")
        
        # Use direct embedding approach (no temp Collections/Zenoh overhead)
        target_tokens = int(_compute_target_length(effective_tokens, style, compression_ratio))
        filtered_text = _semantic_filter_direct(input_value, focus, target_tokens, resource_manager)
        
        filtered_chars = len(filtered_text)
        effective_tokens = _estimate_tokens(filtered_text)
        inclusion_pct = int((effective_tokens / input_tokens) * 100) if input_tokens > 0 else 100
        input_value = filtered_text
    
    # Check if segmentation needed (after filtering)
    chunks = segment_text_boundary_aware(input_value, max_chunk_size=16000)
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
    
    # Log summarization metrics
    logger.info(f"summarize: input={input_chars} chars ({input_tokens}t), "
               f"after_filter={filtered_chars} chars ({effective_tokens}t), "
               f"target={target_tokens}t")
    
    final_text: str | None = None

    if chunk_count == 1:
        # Single chunk - direct summarization
        chunk_text = chunks[0][0]
        
        prompt = f"""{style_instruction}
        
{focus_guidance}

Content:
{chunk_text}

. Your response should be approximately {target_tokens} tokens in length. Do not exceed this length.
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the response, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x final output limit (2000 tokens)
        # Use unified llm_generate callback (required)
        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            return _fail(executor, "llm_generate callback is required")
        response = llm_generate(
            messages=[prompt],
            max_tokens=3000,
            temperature=0.1,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize failed: {response.error}")
            return _fail(
                executor,
                "llm_generate_failed",
                value=f"Error: {response.error}",
                extra={"llm_error": response.error},
            )
        
        final_text = response.text
    
    # Multiple chunks - hierarchical summarization
    logger.info(f"summarize: hierarchical summarization ({chunk_count} chunks)")
    
    # Distribute target across chunks
    tokens_per_chunk = target_tokens // chunk_count
    tokens_per_chunk = max(tokens_per_chunk, 100)  # Floor per chunk
    
    # Step 1: Summarize each chunk
    chunk_summaries = []
    for i, (chunk_text, _) in enumerate(chunks):
        prompt = f"""{style_instruction}
        
{focus_guidance}

Section:
{chunk_text}

Target length: approximately {tokens_per_chunk} words in length
Do not include any introductory, reasoning, or explanatory text in your response. Only provide your response, followed by the </end> tag.
End your response with:
</end>
"""
        
        # max_tokens = 1.5x target per chunk
        max_chunk_tokens = int(tokens_per_chunk * 1.5)
        # Use unified llm_generate callback (required)
        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            return _fail(executor, "llm_generate callback is required")
        response = llm_generate(
            messages=[prompt],
            max_tokens=max_chunk_tokens,
            temperature=0.1,
            is_json=False,
            stops=['</end>']
        )
        
        if heartbeat:
            heartbeat()
        
        if not response.success:
            logger.error(f"summarize chunk {i+1}/{chunk_count} failed: {response.error}")
            return _fail(
                executor,
                "llm_generate_failed",
                value=f"Error: {response.error}",
                extra={"llm_error": response.error, "chunk_index": i + 1},
            )
        
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
    # Use unified llm_generate callback (required)
    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        return _fail(executor, "llm_generate callback is required")
    response = llm_generate(
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
        return _fail(
            executor,
            "llm_generate_failed",
            value=f"Error: {response.error}",
            extra={"llm_error": response.error, "stage": "synthesis"},
        )
    
    final_text = response.text

    if final_text is None:
        return _fail(executor, "summarization_failed")
    
    output_chars = len(final_text)
    output_tokens = _estimate_tokens(final_text)
    logger.info(f"summarize: output={output_chars} chars ({output_tokens}t)")
    
    return _success(
        executor,
        final_text,
        {
            "style": style,
            "focus": focus,
            "chunk_count": chunk_count,
            "input_chars": input_chars,
            "output_chars": output_chars,
            "inclusion_pct": inclusion_pct,
        },
    )

