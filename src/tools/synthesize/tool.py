#!/usr/bin/env python3
"""
Cross-document synthesis tool.
Integrates content across multiple documents to produce new understanding:
narrative synthesis, comparison, and reporting from Collections.

Replaces: summarize (on Collections), relate, generate-note with context.
"""
import logging
import json
import math
import traceback
from typing import Any, Dict, Optional
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


# ---------------------------------------------------------------------------
# Content resolution helpers (from summarize + relate)
# ---------------------------------------------------------------------------

def _get_content(resource_id: str, resource_manager) -> Any:
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


def _resolve_to_text(element: Any, resource_manager) -> str:
    """Resolve any input (Collection ID, Note ID, list, string) to text."""
    if isinstance(element, str) and element.startswith('Collection_'):
        content = _flatten_collection(element, resource_manager)
        return str(content) if content else ""
    elif isinstance(element, str) and element.startswith('Note_'):
        content = _get_content(element, resource_manager)
        return str(content) if content is not None else ""
    elif isinstance(element, list):
        return _flatten_list(element, resource_manager)
    elif isinstance(element, dict):
        return json.dumps(element, indent=2)
    else:
        return str(element) if element else ""


def _estimate_tokens(text: str) -> int:
    """Rough token estimation: ~4 chars per token."""
    return len(text) // 4


# ---------------------------------------------------------------------------
# Semantic focus filtering (from summarize)
# ---------------------------------------------------------------------------

def _semantic_filter_direct(text_content: str, focus: str, target_tokens: int, resource_manager) -> str:
    """
    Directly embed and search text chunks for focus-relevance filtering.
    Uses resource_manager's embedder.
    """
    try:
        import numpy as np

        if not resource_manager:
            logger.warning("synthesize: resource_manager not available, using all content")
            return text_content
        resource_manager._init_embedder()
        embedder = resource_manager.embedder

        # Chunk the text (paragraph-first, then sentences)
        chunks = []
        paragraphs = [p.strip() for p in text_content.split('\n\n') if p.strip()]

        for para in paragraphs:
            if len(para) < 1024:
                chunks.append(para)
            else:
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
                    chunks.append('. '.join(current_chunk) + '.')

        if not chunks:
            return text_content

        # Generate embeddings
        logger.debug(f"synthesize: embedding {len(chunks)} chunks for semantic filtering")
        chunk_embeddings = embedder.encode(chunks, convert_to_tensor=False, show_progress_bar=False)
        query_embedding = embedder.encode([focus], convert_to_tensor=False, show_progress_bar=False)[0]

        # Cosine similarity
        chunk_embeddings_norm = chunk_embeddings / np.linalg.norm(chunk_embeddings, axis=1, keepdims=True)
        query_embedding_norm = query_embedding / np.linalg.norm(query_embedding)
        similarities = np.dot(chunk_embeddings_norm, query_embedding_norm)

        # Filter by minimum similarity threshold
        min_similarity = 0.3
        valid_indices = np.where(similarities >= min_similarity)[0]

        if len(valid_indices) == 0:
            logger.debug(f"synthesize: no chunks above similarity threshold {min_similarity}, using top 10%")
            valid_indices = np.argsort(similarities)[::-1][:max(1, len(chunks) // 10)]

        avg_chunk_tokens = sum(len(chunks[i]) // 4 for i in valid_indices) // len(valid_indices) if valid_indices.size > 0 else 256
        target_chunks = max(1, int(target_tokens / avg_chunk_tokens))
        limit = min(len(valid_indices), target_chunks * 3)

        valid_similarities = similarities[valid_indices]
        top_valid_idx = np.argsort(valid_similarities)[::-1][:limit]
        top_indices = valid_indices[top_valid_idx]

        selected_chunks = []
        cumulative_tokens = 0
        max_tokens = target_tokens * 3

        for idx in top_indices:
            chunk = chunks[idx]
            chunk_tokens = len(chunk) // 4
            if cumulative_tokens + chunk_tokens > max_tokens:
                break
            selected_chunks.append((idx, chunk))
            cumulative_tokens += chunk_tokens

        if not selected_chunks:
            logger.warning(f"synthesize: semantic filter returned no chunks, using all content")
            return text_content

        selected_chunks.sort(key=lambda x: x[0])
        filtered_text = '\n\n'.join([chunk for _, chunk in selected_chunks])

        inclusion_pct = int(100 * len(filtered_text) / len(text_content)) if text_content else 0
        logger.debug(f"synthesize: semantic filter kept {len(selected_chunks)}/{len(chunks)} chunks ({inclusion_pct}%)")

        return filtered_text

    except Exception as e:
        logger.warning(f"Semantic filtering failed: {e}, using all content")
        traceback.print_exc()
        return text_content


# ---------------------------------------------------------------------------
# Target length computation (from summarize)
# ---------------------------------------------------------------------------

def _compute_target_length(effective_tokens: int, fmt: str, compression_ratio: float) -> int:
    """Compute target summary length based on format and compression ratio."""
    if fmt == 'executive':
        return min(1500, effective_tokens // 3)
    elif fmt == 'comprehensive':
        return min(int(math.sqrt(effective_tokens) * 15 + 1500), effective_tokens // 2)
    else:  # narrative, technical (default)
        target = min(int(math.sqrt(effective_tokens) * 15 + 1500), effective_tokens / compression_ratio)
        return max(int(target), 300)


# ---------------------------------------------------------------------------
# Comparison path (from relate)
# ---------------------------------------------------------------------------

def _summarize_for_comparison(content: str, focus: str, heartbeat, kwargs) -> str:
    """Summarize content for comparison if it exceeds context limits."""
    if len(content) <= 16000:
        focus_guidance = f"\nFocus on: {focus}" if focus else ""
        prompt = f"""Summarize the following content concisely for comparison purposes.{focus_guidance}
Provide a detailed summary (8-12 sentences) preserving key themes, facts, and perspectives.

Content:
{content}
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
Summary:"""

        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            raise ValueError("llm_generate callback is required")
        response = llm_generate(messages=[prompt], max_tokens=1000, temperature=0.3, is_json=False, stops=['</end>'])

        if heartbeat:
            heartbeat()

        if not response.success:
            logger.warning(f"summarize for comparison failed: {response.error}, truncating to 16000 chars")
            return content[:16000]
        return response.text if response.text else content[:16000]

    # Content > 16k: chunk and summarize each chunk
    logger.info(f"synthesize: content ({len(content)} chars) exceeds 16k, chunking before comparison summarization")
    chunk_size = 12000
    chunks = [content[i:i + chunk_size] for i in range(0, len(content), chunk_size)]
    logger.info(f"synthesize: split into {len(chunks)} chunks")

    chunk_summaries = []
    focus_guidance = f"\nFocus on: {focus}" if focus else ""

    for i, chunk in enumerate(chunks):
        prompt = f"""Summarize the following content chunk concisely for comparison purposes.{focus_guidance}
Provide a detailed summary (6-10 sentences) preserving key themes, facts, and perspectives.
This is chunk {i+1} of {len(chunks)}.

Content:
{chunk}

Do not include any introductory, reasoning, or explanatory text in your response. Only provide the summary, followed by the </end> tag.
End your response with:
</end>
Summary:"""

        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            raise ValueError("llm_generate callback is required")
        response = llm_generate(messages=[prompt], max_tokens=1000, temperature=0.3, is_json=False, stops=['</end>'])

        if heartbeat:
            heartbeat()

        if response.success and response.text:
            chunk_summaries.append(response.text)
        else:
            logger.warning(f"synthesize: chunk {i+1} failed, using truncated chunk")
            chunk_summaries.append(chunk[:chunk_size // 2])

    combined_summary = '\n\n'.join(chunk_summaries)
    if len(combined_summary) > 16000:
        logger.info(f"synthesize: combined summary ({len(combined_summary)} chars) still exceeds 16k, recursively summarizing")
        return _summarize_for_comparison(combined_summary, focus, heartbeat, kwargs)

    return combined_summary


def _preprocess_for_comparison(element: Any, resource_manager, focus: str, heartbeat, kwargs) -> str:
    """Preprocess a single element for comparison: resolve, flatten, summarize if needed."""
    content = _resolve_to_text(element, resource_manager)
    if not content:
        return ""

    if len(content) > 16000:
        logger.info(f"synthesize: element exceeds 16k ({len(content)} chars), summarizing for comparison")
        content = _summarize_for_comparison(content, focus, heartbeat, kwargs)
        content = str(content) if content else ""

    if len(content) > 16000:
        logger.warning(f"synthesize: element still exceeds 16k after summarization ({len(content)} chars), truncating")
        content = content[:16000]

    return content


def _do_comparison(target_text: str, other_text: str, focus: str, instruction: str, executor, kwargs):
    """Execute the comparison path (replaces relate tool)."""
    formatted_input = f"## Item A\n{target_text}\n\n## Item B\n{other_text}"
    instruction_guidance = f"\n{instruction}" if instruction else ""
    focus_guidance = f"\nFocus on: {focus}" if focus else ""

    prompt = f"""Compare the following content to identify similarities, differences, and relationships.{instruction_guidance}{focus_guidance}

Content:
{formatted_input}

Provide a comparison analysis in JSON format with:
- similarity_score: float (0.0 to 1.0)
- shared_themes: array of strings
- unique_to_first: array of strings (unique to Item A)
- unique_to_second: array of strings (unique to Item B)
- contradictions: array of objects with "aspect", "first", "second" fields
- relationship: string describing relationship type
- summary: string summary of comparison
Do not include any introductory, reasoning, or explanatory text in your response. Only provide the JSON.

"""

    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        return _fail(executor, "llm_generate callback is required")
    response = llm_generate(messages=[prompt], max_tokens=1000, temperature=0.5, is_json=True)

    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()

    if not response.success:
        logger.error(f"synthesize comparison failed: {response.error}")
        return _fail(executor, "llm_generate_failed", value=json.dumps({"error": response.error}), extra={"llm_error": response.error})

    return _success(executor, response.text, {
        "format": "comparison",
        "focus": focus,
        "instruction": instruction,
        "comparison_lengths": {"item_a": len(target_text), "item_b": len(other_text)},
    })


# ---------------------------------------------------------------------------
# Narrative synthesis path (from summarize)
# ---------------------------------------------------------------------------

def _do_narrative_synthesis(input_text: str, fmt: str, focus: str, instruction: str, compression_ratio: float, executor, kwargs):
    """Execute the narrative synthesis path (replaces summarize on Collection)."""
    heartbeat = kwargs.get('heartbeat')
    resource_manager = kwargs.get('resource_manager')

    input_chars = len(input_text)
    input_tokens = _estimate_tokens(input_text)

    # Apply focus filtering
    effective_tokens = input_tokens
    filtered_chars = input_chars
    inclusion_pct = 100

    if focus:
        logger.debug(f"synthesize: applying semantic search filter for '{focus}'")
        target_tokens = int(_compute_target_length(effective_tokens, fmt, compression_ratio))
        filtered_text = _semantic_filter_direct(input_text, focus, target_tokens, resource_manager)
        filtered_chars = len(filtered_text)
        effective_tokens = _estimate_tokens(filtered_text)
        inclusion_pct = int((effective_tokens / input_tokens) * 100) if input_tokens > 0 else 100
        input_text = filtered_text

    # Segment into chunks
    chunks = segment_text_boundary_aware(input_text, max_chunk_size=16000)
    chunk_count = len(chunks)

    # Compute target length (output sizing guidance overrides formula when present)
    guided_tokens = kwargs.get('target_tokens')
    if guided_tokens and isinstance(guided_tokens, (int, float)) and int(guided_tokens) > 0:
        target_tokens = int(guided_tokens)
    else:
        target_tokens = int(_compute_target_length(effective_tokens, fmt, compression_ratio))

    # Style-specific instructions
    style_instructions = {
        'executive': 'Provide a high-level executive summary focused on key findings and implications.',
        'technical': 'Provide a technical summary preserving key details, methodology, and caveats.',
        'comprehensive': 'Provide a comprehensive summary preserving nuance, technical details, and supporting evidence.',
        'narrative': 'Provide a narrative synthesis that identifies themes, patterns, and integrative conclusions.',
    }
    style_instruction = instruction if instruction else style_instructions.get(fmt, style_instructions['narrative'])
    focus_guidance = f"\nFocus on: {focus}" if focus else ""

    logger.info(f"synthesize: input={input_chars} chars ({input_tokens}t), after_filter={filtered_chars} chars ({effective_tokens}t), target={target_tokens}t")

    final_text: str | None = None

    if chunk_count == 1:
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

        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            return _fail(executor, "llm_generate callback is required")
        response = llm_generate(messages=[prompt], max_tokens=3000, temperature=0.1, is_json=False, stops=['</end>'])

        if heartbeat:
            heartbeat()

        if not response.success:
            logger.error(f"synthesize failed: {response.error}")
            return _fail(executor, "llm_generate_failed", value=f"Error: {response.error}", extra={"llm_error": response.error})

        final_text = response.text

    # Multiple chunks - hierarchical summarization
    if final_text is None:
        logger.info(f"synthesize: hierarchical summarization ({chunk_count} chunks)")

        tokens_per_chunk = max(target_tokens // chunk_count, 100)

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

            max_chunk_tokens = int(tokens_per_chunk * 1.5)
            llm_generate = kwargs.get('llm_generate')
            if not llm_generate:
                return _fail(executor, "llm_generate callback is required")
            response = llm_generate(messages=[prompt], max_tokens=max_chunk_tokens, temperature=0.1, is_json=False, stops=['</end>'])

            if heartbeat:
                heartbeat()

            if not response.success:
                logger.error(f"synthesize chunk {i+1}/{chunk_count} failed: {response.error}")
                return _fail(executor, "llm_generate_failed", value=f"Error: {response.error}", extra={"llm_error": response.error, "chunk_index": i + 1})

            chunk_summaries.append(response.text)

        # Step 2: Synthesize chunk summaries into final
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

        llm_generate = kwargs.get('llm_generate')
        if not llm_generate:
            return _fail(executor, "llm_generate callback is required")
        response = llm_generate(messages=[synthesis_prompt], max_tokens=3000, temperature=0.3, is_json=False, stops=['</end>'])

        if heartbeat:
            heartbeat()

        if not response.success:
            logger.error(f"synthesize synthesis failed: {response.error}")
            return _fail(executor, "llm_generate_failed", value=f"Error: {response.error}", extra={"llm_error": response.error, "stage": "synthesis"})

        final_text = response.text

    if final_text is None:
        return _fail(executor, "synthesize_failed")

    output_chars = len(final_text)
    output_tokens = _estimate_tokens(final_text)
    logger.info(f"synthesize: output={output_chars} chars ({output_tokens}t)")

    return _success(executor, final_text, {
        "format": fmt,
        "focus": focus,
        "chunk_count": chunk_count,
        "input_chars": input_chars,
        "output_chars": output_chars,
        "inclusion_pct": inclusion_pct,
    })


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def tool(input_value, runtime=None, **kwargs):
    """
    Synthesize content across documents.
    
    Args:
        input_value: Collection content (list) or Note content to synthesize
        **kwargs: Tool parameters
            - other: Second Note/Collection for comparison (optional)
            - focus: Topic to guide synthesis (optional)
            - format: "narrative" (default), "comparison", "executive", "technical", "comprehensive"
            - compression_ratio: Float (default 3.0), controls output length
            - instruction: Free-form override instruction (optional)
    
    Returns:
        Synthesized content as string (prose or JSON for comparison)
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    if not input_value:
        return _fail(executor, "target parameter required")

    # Extract parameters
    other = kwargs.get('other')
    focus = kwargs.get('focus', '')
    fmt = kwargs.get('format', 'narrative')
    compression_ratio = kwargs.get('compression_ratio', 3.0)
    instruction = kwargs.get('instruction', '')
    heartbeat = kwargs.get('heartbeat')
    resource_manager = kwargs.get('resource_manager')

    # Validate format
    valid_formats = ['narrative', 'comparison', 'executive', 'technical', 'comprehensive']
    if fmt not in valid_formats:
        logger.warning(f"Unknown format '{fmt}', using 'narrative'")
        fmt = 'narrative'

    # Comparison path
    if fmt == 'comparison':
        if not other:
            return _fail(executor, "comparison format requires 'other' parameter")

        comparison_focus = focus if focus else (instruction if instruction else "key themes, facts, and perspectives for comparison")
        target_text = _preprocess_for_comparison(input_value, resource_manager, comparison_focus, heartbeat, kwargs)
        other_text = _preprocess_for_comparison(other, resource_manager, comparison_focus, heartbeat, kwargs)

        if not target_text:
            return _fail(executor, "target is empty")
        if not other_text:
            return _fail(executor, "other is empty")

        return _do_comparison(target_text, other_text, focus, instruction, executor, kwargs)

    # Narrative synthesis path
    # Resolve input to text
    if isinstance(input_value, list):
        logger.debug(f"synthesize: flattening Collection with {len(input_value)} items")
        input_text = _flatten_list(input_value, resource_manager)
    elif isinstance(input_value, dict):
        input_text = json.dumps(input_value, indent=2)
    elif not isinstance(input_value, str):
        input_text = str(input_value)
    else:
        input_text = input_value

    # If other is provided for non-comparison format, concatenate both inputs
    if other:
        other_text = _resolve_to_text(other, resource_manager)
        if other_text:
            input_text = input_text + "\n\n---\n\n" + other_text

    if not input_text:
        return _fail(executor, "target is empty")

    return _do_narrative_synthesis(input_text, fmt, focus, instruction, compression_ratio, executor, kwargs)
