"""
LLM-assisted web search + synthesize.

Primary:  OpenAI GPT-5.4-mini + web_search via Responses API (cheaper)
Fallback: Claude Sonnet + web_search tool

Env vars:
  OPENAI_API_KEY   — OpenAI API key (primary)
  CLAUDE_API_KEY   — Anthropic API key (fallback)

The output contract: a single Note containing structured JSON with:
  - synthesis: readable synthesized answer (what the agent acts on)
  - sources: [{url, domain, title, excerpt}, ...] (for citation/follow-up)
  - query: the original query
"""

import json
import logging
import os
import time
import traceback
from datetime import date
from typing import List, Dict, Any, Optional
from urllib.parse import urlparse

import requests
import warnings

# infospace_executor was the planner-side runtime; the chat ReAct loop
# bypasses it and calls llm_search() directly. Keep the import optional so
# this module loads in chat-only builds. The InfospaceExecutor type is used
# only as an annotation on `_fail`/`_success`/`tool()` (the planner-facing
# entry point); when the executor is gone, those signatures degrade to
# `Any`, but they are not callable from chat-only mode anyway.
try:
    from infospace_executor import InfospaceExecutor
except ImportError:
    InfospaceExecutor = Any  # type: ignore[assignment,misc]

# ------------------------------
# Logging
# ------------------------------
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
warnings.filterwarnings('ignore', category=UserWarning)

# ------------------------------
# Constants
# ------------------------------
OPENAI_API_URL = "https://api.openai.com/v1/responses"
OPENAI_MODEL = "gpt-5.4-mini"

CLAUDE_API_URL = "https://api.anthropic.com/v1/messages"
CLAUDE_MODEL = "claude-sonnet-4-5-20250929"

# ------------------------------
# Uniform return helpers (unchanged interface)
# ------------------------------

def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None,
          extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason, extra=extra,
    )


def _success(executor: InfospaceExecutor, value: str,
             resource_id: Optional[str], extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=resource_id, extra=extra,
    )


# ------------------------------
# Note helper
# ------------------------------

def _create_note(text_content: str, agent_name: str, resource_manager,
                 source_skill: str = 'search-web', tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note. Content is text-only; metadata in tool_metadata (internal)."""
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    success, note_id, error_msg, _location = resource_manager.create_note(
        agent_name, text_content, 'text', source_skill, (text_content or '')[:100], '',
        {'tool_metadata': tool_metadata or {}}
    )
    if success:
        return note_id
    logger.error(f"Failed to create Note: {error_msg}")
    return None

# ------------------------------
# Domain extraction
# ------------------------------

def _extract_domain(url: str) -> str:
    try:
        return urlparse(url).netloc.lower()
    except Exception:
        return ""

# ------------------------------
# OpenAI Responses API with web_search tool
# ------------------------------

def _call_openai_web_search(query: str, timeout: float = 90.0,
                            heartbeat=None) -> Optional[Dict[str, Any]]:
    """
    OpenAI GPT-5.4-mini call with web_search tool via Responses API.

    Returns parsed dict with 'synthesis' and 'sources', or None on failure.
    """
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key:
        logger.warning("OPENAI_API_KEY not set, cannot use OpenAI web search")
        return None

    today = date.today().strftime("%B %d, %Y")

    instructions = f"""Today is {today}. You are a research assistant with web search.

Search the web to find information for the user's query.
Prioritize recent, substantive, and credible sources.

After searching, respond with a JSON object in exactly this format:

{{
  "synthesis": "Your synthesized findings as a readable, detailed research note. Include specific facts, numbers, quotes, and attributions. Multiple paragraphs are fine. Write for someone who needs to act on this information.",
  "sources": [
    {{
      "url": "https://...",
      "domain": "example.com",
      "title": "Page title",
      "excerpt": "Key relevant content from this source (1-3 sentences)"
    }}
  ]
}}

Guidelines:
- The synthesis should be substantive — not a summary of summaries, but real detail
- Attribute claims to sources within the synthesis text
- Include all meaningfully distinct sources (not just the top 1-2)
- Excerpts should capture the most informative content from each source
- If sources conflict, note the disagreement in the synthesis
- Return ONLY the JSON object, no markdown fences, no preamble"""

    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    payload = {
        "model": OPENAI_MODEL,
        "instructions": instructions,
        "tools": [
            {
                "type": "web_search",
                "user_location": {
                    "type": "approximate",
                    "country": "US",
                    "city": "Berkeley",
                    "region": "California",
                    "timezone": "America/Los_Angeles",
                },
            }
        ],
        "input": query,
    }

    try:
        t0 = time.time()
        resp = requests.post(OPENAI_API_URL, headers=headers, json=payload, timeout=timeout)

        if heartbeat:
            heartbeat()

        elapsed_ms = int((time.time() - t0) * 1000)
        logger.info(f"search-web: OpenAI API call took {elapsed_ms}ms")

        if resp.status_code != 200:
            logger.error(f"OpenAI API error {resp.status_code}: {resp.text[:500]}")
            return None

        data = resp.json()

        # Extract text from the response output items
        text_content = ""
        annotations = []
        for item in data.get("output", []):
            if item.get("type") == "message":
                for content_block in item.get("content", []):
                    if content_block.get("type") == "output_text":
                        text_content += content_block.get("text", "")
                        annotations.extend(content_block.get("annotations", []))

        if not text_content.strip():
            # Fallback: try output_text top-level field
            text_content = data.get("output_text", "")

        if not text_content.strip():
            logger.warning("OpenAI returned no text content")
            return None

        # Try to parse structured JSON from the response
        parsed = _parse_json_response(text_content)
        if parsed is None:
            # Build structured result from raw text + citation annotations
            logger.info("search-web: Building structured result from annotations")
            sources = []
            for ann in annotations:
                if ann.get("type") == "url_citation":
                    sources.append({
                        "url": ann.get("url", ""),
                        "domain": _extract_domain(ann.get("url", "")),
                        "title": ann.get("title", ""),
                        "excerpt": "",
                    })
            parsed = {
                "synthesis": text_content.strip(),
                "sources": sources,
            }

        parsed["_elapsed_ms"] = elapsed_ms
        parsed["_model"] = OPENAI_MODEL
        return parsed

    except requests.exceptions.Timeout:
        logger.error(f"OpenAI API timeout after {timeout}s")
        return None
    except Exception as e:
        logger.error(f"OpenAI web search failed: {e}")
        traceback.print_exc()
        return None


# ------------------------------
# Claude API with web_search tool (fallback)
# ------------------------------

def _call_claude_web_search(query: str, timeout: float = 90.0,
                            heartbeat=None) -> Optional[Dict[str, Any]]:
    """
    Single Claude Sonnet call with web_search tool enabled.

    Returns parsed dict with 'synthesis' and 'sources', or None on failure.
    """
    api_key = os.getenv("CLAUDE_API_KEY", "").strip()
    if not api_key:
        logger.error("CLAUDE_API_KEY not set")
        return None

    today = date.today().strftime("%B %d, %Y")

    system_prompt = f"""Today is {today}. You are a research assistant with web search.

Use your web_search tool to find information for the user's query. Search again with different phrasing only if initial results are insufficient.
Prioritize recent, substantive, and credible sources.

After searching, respond with a JSON object in exactly this format:

{{
  "synthesis": "Your synthesized findings as a readable, detailed research note. Include specific facts, numbers, quotes, and attributions. Multiple paragraphs are fine. Write for someone who needs to act on this information.",
  "sources": [
    {{
      "url": "https://...",
      "domain": "example.com",
      "title": "Page title",
      "excerpt": "Key relevant content from this source (1-3 sentences)"
    }}
  ]
}}

Guidelines:
- The synthesis should be substantive — not a summary of summaries, but real detail
- Attribute claims to sources within the synthesis text
- Include all meaningfully distinct sources (not just the top 1-2)
- Excerpts should capture the most informative content from each source
- If sources conflict, note the disagreement in the synthesis
- Return ONLY the JSON object, no markdown fences, no preamble"""

    headers = {
        "x-api-key": api_key,
        "anthropic-version": "2023-06-01",
        "content-type": "application/json",
    }

    payload = {
        "model": CLAUDE_MODEL,
        "max_tokens": 2400,
        "system": system_prompt,
        "tools": [
            {
                "type": "web_search_20250305",
                "name": "web_search",
                "max_uses": 2
            }
        ],
        "messages": [
            {"role": "user", "content": query}
        ],
    }

    try:
        t0 = time.time()
        resp = requests.post(CLAUDE_API_URL, headers=headers, json=payload, timeout=timeout)

        if heartbeat:
            heartbeat()

        elapsed_ms = int((time.time() - t0) * 1000)
        logger.info(f"search-web: Claude API call took {elapsed_ms}ms")

        if resp.status_code != 200:
            logger.error(f"Claude API error {resp.status_code}: {resp.text[:500]}")
            return None

        data = resp.json()

        # Collect all text blocks from the response
        text_content = ""
        for block in data.get("content", []):
            if block.get("type") == "text":
                text_content += block["text"]

        if not text_content.strip():
            logger.warning("Claude returned no text content")
            return None

        # Parse the JSON
        parsed = _parse_json_response(text_content)
        if parsed is None:
            # Fallback: treat the whole response as synthesis with no structured sources
            logger.warning("search-web: Could not parse structured JSON, using raw text")
            parsed = {
                "synthesis": text_content.strip(),
                "sources": [],
            }

        parsed["_elapsed_ms"] = elapsed_ms
        parsed["_model"] = CLAUDE_MODEL
        return parsed

    except requests.exceptions.Timeout:
        logger.error(f"Claude API timeout after {timeout}s")
        return None
    except Exception as e:
        logger.error(f"Claude web search failed: {e}")
        traceback.print_exc()
        return None


def _parse_json_response(text: str) -> Optional[Dict[str, Any]]:
    """Parse Claude's JSON response, handling code fences and minor issues."""
    cleaned = text.strip()

    # Strip markdown code fences
    if cleaned.startswith("```"):
        first_nl = cleaned.find("\n")
        if first_nl >= 0:
            cleaned = cleaned[first_nl + 1:]
    if cleaned.endswith("```"):
        cleaned = cleaned[:-3]
    cleaned = cleaned.strip()

    # Direct parse
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        pass

    # Find outermost braces
    start = cleaned.find("{")
    end = cleaned.rfind("}") + 1
    if start >= 0 and end > start:
        try:
            return json.loads(cleaned[start:end])
        except json.JSONDecodeError:
            pass

    return None


# ------------------------------
# High-level search (replaces llm_search)
# ------------------------------

def llm_search(query: str, llm_generate=None, max_chars: int = 8000,
               max_urls: int = 10, max_workers: int = 6,
               wall_time_limit: float = 90.0, heartbeat=None,
               grobid_url: str = None) -> Optional[Dict[str, Any]]:
    """
    Search using OpenAI GPT-5.4-mini (primary) or Claude Sonnet (fallback).

    Tries OpenAI Responses API first (cheaper). Falls back to Claude if
    OPENAI_API_KEY is not set or the call fails.

    Parameters kept for interface compatibility; llm_generate, max_urls, max_workers,
    grobid_url are accepted but unused.

    Returns:
        Dict with 'synthesis' (str) and 'sources' (list of dicts), or None.
    """
    # Try OpenAI first (cheaper)
    result = _call_openai_web_search(query, timeout=wall_time_limit, heartbeat=heartbeat)

    # Fall back to Claude if OpenAI failed
    if not result:
        logger.info("search-web: Falling back to Claude web search")
        result = _call_claude_web_search(query, timeout=wall_time_limit, heartbeat=heartbeat)

    if not result:
        return None

    # Enrich source entries with domain if missing
    for src in result.get("sources", []):
        if not src.get("domain") and src.get("url"):
            src["domain"] = _extract_domain(src["url"])

    return result


# ------------------------------
# Tool interface for infospace
# ------------------------------

def tool(input_value, **kwargs):
    """
    Web search tool using Claude Sonnet + web_search.

    Args:
        input_value: Query string (for backward compatibility)
        **kwargs: query, agent_name, executor, resource_manager, heartbeat, etc.

    Returns:
        uniform_return with a single Note ID containing {synthesis, sources, query}
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    query = kwargs.get('query') or input_value or kwargs.get('value', '')
    if not isinstance(query, str):
        query = ''
    if not query:
        return _fail(executor, 'query parameter required (search query)')

    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return _fail(executor, 'agent_name required in kwargs')

    if not os.getenv('OPENAI_API_KEY') and not os.getenv('CLAUDE_API_KEY'):
        return _fail(executor, 'OPENAI_API_KEY or CLAUDE_API_KEY environment variable required')

    # Perform search
    try:
        result = llm_search(
            query=query,
            wall_time_limit=90.0,
            heartbeat=kwargs.get('heartbeat'),
        )
    except Exception as e:
        return _fail(executor, 'search_failed', extra={"exception": str(e)})

    if not result or not result.get("synthesis"):
        return _fail(executor, 'no_results',
                     value=f'No relevant results found for: {query}',
                     extra={"query": query})

    resource_manager = kwargs.get('resource_manager')

    # Text-only content: synthesis. Sources/query in tool_metadata (internal)
    synthesis = result["synthesis"]
    tool_meta = {"query": query, "sources": result.get("sources", []), "source_count": len(result.get("sources", [])),
                 "model": result.get("_model", CLAUDE_MODEL), "elapsed_ms": result.get("_elapsed_ms", 0)}
    note_id = _create_note(synthesis, agent_name, resource_manager, tool_metadata=tool_meta)
    if not note_id:
        return _fail(executor, 'Failed to create Note from search results')

    # Display value: first ~200 chars of synthesis for the planner to see
    synth_preview = result["synthesis"][:200]
    if len(result["synthesis"]) > 200:
        synth_preview += "..."
    source_count = len(result.get("sources", []))
    display_value = f"[{source_count} sources] {synth_preview}"

    logger.info(f"search-web created Note {note_id} for query: {query} "
                f"({source_count} sources, {len(result['synthesis'])} chars)")

    return _success(
        executor,
        display_value,
        note_id,
        {"source_count": source_count, "query": query},
    )


# ------------------------------
# Standalone test
# ------------------------------

if __name__ == "__main__":
    import sys

    query = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else \
        "Tesla FSD 14.2.2.4 daily driver experience, wait for 14.3?"

    if not os.getenv("OPENAI_API_KEY") and not os.getenv("CLAUDE_API_KEY"):
        print("Set OPENAI_API_KEY or CLAUDE_API_KEY environment variable")
        sys.exit(1)

    print(f"Searching: {query}\n")
    result = llm_search(query)

    if not result:
        print("No results.")
        sys.exit(1)

    print("=" * 80)
    print("SYNTHESIS:")
    print("=" * 80)
    print(result["synthesis"])
    print()
    print("=" * 80)
    print(f"SOURCES ({len(result.get('sources', []))}):")
    print("=" * 80)
    for i, src in enumerate(result.get("sources", []), 1):
        print(f"\n  [{i}] {src.get('title', '(no title)')}")
        print(f"      {src.get('url', '')}")
        print(f"      {src.get('excerpt', '')[:150]}")
    print(f"\nElapsed: {result.get('_elapsed_ms', 0)}ms")