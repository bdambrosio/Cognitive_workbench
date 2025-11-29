#!/usr/bin/env python3
"""
LLM-based content generation tool.
Generates new text or code from scratch using natural language prompts.
"""
import logging
import json

logger = logging.getLogger(__name__)


def _get_content(resource_id: str, resource_manager) -> any:
    """Fetch content from resource_manager for a resource ID."""
    if resource_id == "Note_null":
        return None
    
    resource = resource_manager.get_resource(resource_id)
    if resource:
        return resource.get('properties', {}).get('content')
    return None


def _resolve_context(context_arg: str, resource_manager) -> str:
    """
    Resolve context argument to text content.
    
    Supports:
    - Collection ID: Fetches all Notes in Collection and concatenates their content
    - Note ID: Fetches Note content
    - Plain text: Returns as-is
    
    Returns:
        Context text string
    """
    if not context_arg:
        return ''
    
    context_str = str(context_arg)
    
    # Check if it's a Collection ID
    if context_str.startswith('Collection_'):
        try:
            collection_content = _get_content(context_str, resource_manager)
            if collection_content is None:
                logger.warning(f"Collection {context_str} not found")
                return ''
            
            # Get list of Note IDs
            if not isinstance(collection_content, list):
                logger.warning(f"Collection {context_str} content is not a list")
                return ''
            
            # Fetch each Note's content and concatenate
            context_parts = []
            for note_id in collection_content:
                if isinstance(note_id, str) and note_id.startswith('Note_'):
                    note_content = _get_content(note_id, resource_manager)
                    if note_content is not None:
                        # Handle structured Notes (extract text field if present)
                        if isinstance(note_content, dict) and 'text' in note_content:
                            context_parts.append(str(note_content['text']))
                        else:
                            context_parts.append(str(note_content))
            
            return '\n\n'.join(context_parts)
        except Exception as e:
            logger.error(f"Failed to resolve Collection context {context_str}: {e}")
            return ''
    
    # Check if it's a Note ID
    if context_str.startswith('Note_'):
        try:
            note_content = _get_content(context_str, resource_manager)
            if note_content is None:
                logger.warning(f"Note {context_str} not found")
                return ''
            
            # Handle structured Notes (extract text field if present)
            if isinstance(note_content, dict) and 'text' in note_content:
                return str(note_content['text'])
            return str(note_content)
        except Exception as e:
            logger.error(f"Failed to resolve Note context {context_str}: {e}")
            return ''
    
    # Plain text - return as-is
    return context_str


def tool(value, runtime=None, **kwargs):
    """
    Generate new content using natural language prompt.
    
    Args:
        value: Unused (for compatibility with executor interface)
        **kwargs: Tool parameters
            - prompt: Generation instruction (REQUIRED)
            - style: "code" or "text" (optional, default: "text")
            - context: Optional context - Collection ID, Note ID, or plain text string
    
    Returns:
        Generated content as string
    """
    prompt = kwargs.get('prompt', '')
    if not prompt:
        return "Error: prompt parameter required"
    
    style = kwargs.get('style', 'text').lower()
    context_arg = kwargs.get('context', '')
    resource_manager = kwargs.get('resource_manager')
    
    # Resolve context (handles Collection IDs, Note IDs, or plain text)
    context = _resolve_context(context_arg, resource_manager)
    
    # Build generation prompt based on style
    if style == 'code':
        generation_prompt = f"""Generate code according to the following instruction.

Instruction: {prompt}

{f"Context/Requirements:\n{context}\n" if context else ""}

Return only the code, no explanation, no code fences, no markdown formatting.
Do not include any introductory text, reasoning, or commentary.
Only provide the code itself, followed by the </end> tag.
End your response with:
</end>
"""
        max_tokens = 4000
        temperature = 0.2
    else:
        generation_prompt = f"""Generate text content according to the following instruction.

Instruction: {prompt}

{f"Context:\n{context}\n" if context else ""}

Return only the generated content, no explanation, no code fences, no markdown formatting.
Do not include any introductory text, reasoning, or commentary.
Only provide the generated content itself, followed by the </end> tag.
End your response with:
</end>
"""
        max_tokens = 2000
        temperature = 0.7
    

    logger.info(f"generate-note: {prompt[:50]}... (style={style})")
    
    # Use unified llm_generate callback (required)
    llm_generate = kwargs.get('llm_generate')
    if not llm_generate:
        raise ValueError("llm_generate callback is required")
    response = llm_generate(
        messages=[generation_prompt],
        max_tokens=max_tokens,
        temperature=temperature,
        is_json=False,
        stops=['</end>']
    )
    
    # Send heartbeat after LLM call to reset timeout
    heartbeat = kwargs.get('heartbeat')
    if heartbeat:
        heartbeat()
    
    if not response.success:
        logger.error(f"generate-note failed: {response.error}")
        return f"Error: {response.error}"
    
    result = response.text.strip()
    logger.info(f"generate-note complete: output_len={len(result)}")
    
    return result

