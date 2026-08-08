"""
Semantic Scholar academic paper search tool.
"""

import json
import logging
import os
import time
import requests
import urllib.parse
from typing import List, Dict, Any, Optional
from urllib.error import HTTPError, URLError
# infospace_executor was the planner-side runtime; the chat ReAct loop
# bypasses it. Keep the import optional so this module loads in chat-only
# builds — the InfospaceExecutor annotations degrade to Any there.
try:
    from infospace_executor import InfospaceExecutor
except ImportError:
    InfospaceExecutor = Any  # type: ignore[assignment,misc]

# Import grobid parser
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils.grobid import parse_pdf_grobid

logger = logging.getLogger(__name__)


class SearchUnavailable(Exception):
    """Search backend unreachable/throttled — distinct from a search that ran and found nothing."""


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed",
        value=value or reason,
        reason=reason,
        extra=extra,
    )


def _success(
    executor: InfospaceExecutor,
    value: str,
    resource_id: Optional[str],
    extra: Optional[Dict[str, Any]] = None,
):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)

ss_api_key = os.getenv("SEMANTIC_SCHOLAR_API_KEY") or None

def _create_note(text_content: str, agent_name: str, resource_manager, source_skill: str = 'semantic-scholar',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note via resource_manager. Content is text-only; metadata in tool_metadata (internal)."""
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=text_content,
        format_type='text',
        source_skill=source_skill,
        source_value=(text_content or '')[:100],
        note_name='',
        extra_props={'tool_metadata': tool_metadata or {}}
    )
    
    if success:
        return note_id
    else:
        logger.error(f"Failed to create Note: {error_msg}")
        return None

def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'semantic-scholar') -> Optional[str]:
    """Create a Collection via resource_manager and return its ID."""
    if not resource_manager:
        logger.error("resource_manager required for creating Collections")
        return None
    
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name,
        note_ids,
        'list',
        f'{len(note_ids)} papers',
        '',
        '',
        {}
    )
    
    if success:
        logger.info(f"Created Collection {collection_id} with {len(note_ids)} items")
        return collection_id
    else:
        logger.error(f"Failed to create Collection: {error_msg}")
        return None

def _enhance_result_with_grobid(result: Dict[str, Any], grobid_url: str) -> Dict[str, Any]:
    """
    Enhance a search result with GROBID parsing if PDF is available.
    
    Args:
        result: Search result dict with metadata containing pdf_url
        grobid_url: GROBID server URL
        
    Returns:
        Enhanced result dict (original if grobid fails)
    """
    pdf_url = result.get('metadata', {}).get('pdf_url')
    if not pdf_url:
        return result
    
    title = result.get('metadata', {}).get('title', 'Untitled')
    
    # Parse PDF with GROBID (handles download internally)
    grobid_result = parse_pdf_grobid(pdf_url=pdf_url, title=title, grobid_url=grobid_url)
    if not grobid_result:
        logger.warning(f"GROBID parsing failed for {title}, keeping original result")
        return result
    
    # Fill in missing fields (only if API value is missing or empty)
    metadata = result.get('metadata', {})
    if not metadata.get('title') or metadata.get('title') == 'Untitled':
        if grobid_result.get('title') and grobid_result['title'] != 'Unknown':
            metadata['title'] = grobid_result['title']
    
    if not metadata.get('authors') or len(metadata.get('authors', [])) == 0:
        if grobid_result.get('authors'):
            # Convert comma-separated string to list
            authors_str = grobid_result['authors']
            if authors_str:
                metadata['authors'] = [a.strip() for a in authors_str.split(',') if a.strip()]
    
    # Replace text with concatenated chunks (with section headers)
    chunks = grobid_result.get('chunks', [])
    if chunks:
        chunk_texts = []
        for section_title, section_text in chunks:
            chunk_texts.append(f"{section_title}\n{section_text}")
        result['text'] = "\n\n".join(chunk_texts)
        result['char_count'] = len(result['text'])
    elif grobid_result.get('abstract'):
        # If no chunks but we have abstract, use abstract as text
        result['text'] = grobid_result['abstract']
        result['char_count'] = len(result['text'])
    
    logger.info(f"Enhanced {title} with GROBID parsing: {len(chunks)} chunks, {result['char_count']} chars")
    return result

def search_papers(query: str, limit: int = 6) -> List[Dict[str, Any]]:
    """
    Search Semantic Scholar for papers.
    
    Args:
        query: Search query string
        limit: Maximum number of results (default 10)
        
    Returns:
        List of paper dicts with uniform structure
    """
    api_key = os.getenv("SEMANTIC_SCHOLAR_API_KEY") or None
    
    # Use direct API calls instead of async library to avoid uvloop conflicts
    # The semanticscholar library uses asyncio which conflicts with FastAPI's uvloop
    logger.info(f"Using direct API for Semantic Scholar search: {query}")
    return _search_papers_direct(query, limit, api_key)

def _search_papers_direct(query: str, limit: int, api_key: str = None) -> List[Dict[str, Any]]:
    """
    Direct API call to Semantic Scholar with rate limit handling.

    Implements exponential backoff for 429 (rate limit) responses:
    - Retry 1: 2 seconds
    - Retry 2: 4 seconds
    - Retry 3: 8 seconds
    - After 3 retries, or on any other HTTP/network error: raises
      SearchUnavailable so callers can distinguish "search broken/throttled"
      from "search ran and found nothing" ([]).
    """
    # Define the API endpoint URL
    url = f"http://api.semanticscholar.org/graph/v1/paper/search"

    # Define the query parameters
    query_params = {"query": query, "limit": limit, "fields":"paperId,title,abstract,authors,externalIds,year,citationCount,venue,openAccessPdf,url"}

    data = _s2_request(url, query_params, api_key)
    logger.info(f"Found {len(data.get('data', []))} papers")
    results = []
    for paper in data.get("data", []):
        abstract = paper.get("abstract", "No abstract available")
        title = paper.get("title", "Untitled")
        # Safely extract author names - handle various API response formats
        authors = []
        for a in paper.get("authors", []):
            if isinstance(a, dict):
                author_name = a.get("name") or a.get("authorId") or "Unknown"
            elif isinstance(a, str):
                author_name = a
            else:
                # Fallback: try to convert to string
                author_name = str(a) if a else "Unknown"
            authors.append(str(author_name))  # Ensure it's always a string
        year = paper.get("year", 0)
        citations = paper.get("citationCount", 0)
        venue = paper.get("venue", "")
        paper_id = paper.get("paperId", "")

        # Get PDF URL
        pdf_url = None
        open_access = paper.get("openAccessPdf")
        if open_access:
            pdf_url = open_access.get("url")

        # Get DOI
        doi = None
        external_ids = paper.get("externalIds", {})
        if external_ids:
            doi = external_ids.get("DOI")
            # Try ArXiv as fallback
            if not pdf_url and "ArXiv" in external_ids:
                arxiv_id = external_ids["ArXiv"]
                pdf_url = f"https://arxiv.org/pdf/{arxiv_id}.pdf"

        if not ((type(abstract) == str and len(abstract) > 0) and pdf_url):
            logger.debug(f"skipping paper {title} because it has no abstract or PDF URL: {abstract} {pdf_url}")
            continue
        result = {
            "text": abstract,
            "format": "paper",
            "metadata": {
                "title": title,
                "authors": authors,
                "year": year,
                "citations": citations,
                "venue": venue,
                "paper_id": paper_id,
                "doi": doi,
                "pdf_url": pdf_url,
                "uri": pdf_url  # Standardized URI field for consistency with search-web and search primitives
            },
            "char_count": len(abstract)
        }
        results.append(result)
        logger.info(f"Found paper: {title}")

    return results


def fetch_references(paper_id: str, limit: int = 40) -> Dict[str, Any]:
    """Structured reference list for one paper, straight from the S2 graph.

    These are resolved graph entities, not strings scraped out of a
    bibliography: each carries its own paperId, so a reference can be
    followed without a title lookup. GROBID's citation parser is the
    fallback for PDFs S2 has no record of, not the primary path.

    Returns {"title", "total", "references": [...]}; raises
    SearchUnavailable on a transport/API failure.
    """
    api_key = os.getenv("SEMANTIC_SCHOLAR_API_KEY") or None
    pid = (paper_id or '').strip()
    if not pid:
        raise SearchUnavailable("paper_id is required to fetch references")
    url = f"https://api.semanticscholar.org/graph/v1/paper/{urllib.parse.quote(pid, safe=':')}"
    params = {"fields": "title,references.paperId,references.title,"
                        "references.year,references.authors,"
                        "references.venue,references.externalIds"}
    data = _s2_request(url, params, api_key)
    refs = []
    for r in (data.get("references") or []):
        if not isinstance(r, dict):
            continue
        ext = r.get("externalIds") or {}
        refs.append({
            "title": r.get("title") or "(untitled)",
            "year": r.get("year"),
            "authors": [a.get("name") for a in (r.get("authors") or [])
                        if isinstance(a, dict) and a.get("name")],
            "venue": r.get("venue") or "",
            "paper_id": r.get("paperId"),
            "doi": ext.get("DOI"),
            "arxiv": ext.get("ArXiv"),
        })
    return {"title": data.get("title") or "(unknown paper)",
            "total": len(refs),
            "references": refs[:max(1, int(limit))]}


def _s2_request(url: str, params: Dict[str, Any],
                api_key: str = None) -> Dict[str, Any]:
    """One GET against the S2 graph API with exponential backoff.

    Retries only 429 (2s, 4s, 8s). Every other failure raises
    SearchUnavailable so callers can tell "S2 is broken/throttled" from
    "S2 ran and found nothing". Shared by search and reference lookup so
    the retry policy has exactly one implementation.
    """
    headers = {"x-api-key": api_key} if api_key else {}
    max_retries = 3
    base_delay = 2.0

    for attempt in range(max_retries + 1):
        try:
            response = requests.get(url, params=params, headers=headers,
                                    timeout=30)
        except requests.RequestException as e:
            logger.error(f"Semantic Scholar request failed: {e}")
            raise SearchUnavailable(f"Semantic Scholar unreachable: {e}") from e

        if response.status_code == 200:
            return response.json()

        if response.status_code == 429:
            if attempt < max_retries:
                delay = base_delay * (2 ** attempt)
                logger.warning(f"Rate limit (429) on attempt {attempt + 1}/{max_retries + 1}. Waiting {delay}s before retry...")
                time.sleep(delay)
                continue
            logger.error(f"Rate limit exceeded after {max_retries} retries. Giving up.")
            raise SearchUnavailable(
                f"Semantic Scholar is rate-limited (429 after {max_retries} retries) — "
                "transient availability problem, not an empty result; retry later")

        # Deterministic error — retrying won't help, fail fast
        logger.error(f"Semantic Scholar API error {response.status_code}: {response.text[:200]}")
        hint = " (likely invalid/expired API key)" if response.status_code == 403 else ""
        raise SearchUnavailable(
            f"Semantic Scholar returned HTTP {response.status_code}{hint} — "
            "search unavailable, not an empty result")


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    from utils.chat_tool_stub import build_tool_kwargs, CapturingResourceManager, translate_result
    try:
        limit = int(args.get("limit", 10))
    except (TypeError, ValueError):
        limit = 10

    # Reference mode: an explicit paper_id selects a citation lookup
    # instead of a search. Kept as a separate call rather than folded into
    # every search result — 10 papers x ~40 references each would be a
    # multi-thousand-line observation, and the agent wants the references
    # of one chosen paper, not of everything it just found.
    paper_id = args.get("paper_id")
    if isinstance(paper_id, str) and paper_id.strip():
        try:
            data = fetch_references(paper_id, limit=args.get("limit", 40))
        except SearchUnavailable as e:
            return {"status": "error", "text": f"semantic-scholar: {e}"}
        refs = data["references"]
        if not refs:
            return {"status": "empty",
                    "text": f"Semantic Scholar has no reference list for "
                            f"{data['title']!r} — its record may predate "
                            f"reference indexing. For an unindexed PDF, "
                            f"extract references from the PDF itself."}
        lines = [f"References of {data['title']!r} "
                 f"({data['total']} total, showing {len(refs)}):"]
        for i, r in enumerate(refs, 1):
            who = ", ".join(r["authors"][:2])
            if len(r["authors"]) > 2:
                who += " et al."
            link = (f" arXiv:{r['arxiv']}" if r.get("arxiv")
                    else (f" doi:{r['doi']}" if r.get("doi") else ""))
            venue = f" — {r['venue']}" if r.get("venue") else ""
            lines.append(f"{i}. {r['title']} ({r.get('year') or 'n.d.'})"
                         f" — {who or 'unknown'}{venue}{link}")
        return {"status": "ok", "text": "\n".join(lines)}

    query = args.get("query", "")
    if not isinstance(query, str) or not query.strip():
        return {"status": "error",
                "text": "semantic-scholar requires non-empty `query` "
                        "(or a `paper_id` to list that paper's references)"}

    mgr = CapturingResourceManager()
    result = tool(query, **build_tool_kwargs(
        character_name=character_name, backend=backend, manager=mgr,
        query=query, limit=limit,
    ))
    return translate_result(result, manager=mgr,
                            empty_text=f"no papers found for {query!r}")


def tool(input_value, runtime=None, **kwargs):
    """
    Semantic Scholar search tool.
    
    Args:
        input_value: Query string (preferred, for backward compatibility)
        **kwargs: query (required), agent_name (required), resource_manager (required), limit (optional), grobid_url (optional)
        
    Returns:
        Collection ID containing structured Note for each paper result
    """
    # Extract grobid_url and pdf_parser from kwargs (from YAML config)
    grobid_url = kwargs.get('grobid_url')
    pdf_parser = kwargs.get('pdf_parser')
    
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    # Prefer 'query' parameter, fallback to input_value for backward compatibility
    query = kwargs.get('query') or input_value or kwargs.get('value', '')
    if not isinstance(query, str):
        query = ''
    if not query:
        return _fail(executor, 'query parameter required (paper search query)')
    
    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return _fail(executor, 'agent_name required in kwargs')
    
    resource_manager = kwargs.get('resource_manager')
    if not resource_manager:
        return _fail(executor, 'resource_manager required in kwargs')
    
    limit = kwargs.get('limit', 10)

    # Search papers
    try:
        results = search_papers(query, limit=limit)
    except SearchUnavailable as e:
        logger.error(f"semantic-scholar search unavailable: {e}")
        return _fail(executor, str(e))
    
    # Enhance results with GROBID if grobid_url is provided and pdf_parser is not "pymupdf"
    use_pymupdf_only = (pdf_parser or "").lower() == "pymupdf"
    if grobid_url and results and not use_pymupdf_only:
        logger.info(f"Enhancing {len(results)} results with GROBID parsing")
        enhanced_results = []
        for result in results:
            enhanced = _enhance_result_with_grobid(result, grobid_url)
            enhanced_results.append(enhanced)
        results = enhanced_results
    
    if not results:
        # Return empty Collection for no results
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(
            executor,
            '0 items []',
            empty_coll_id,
            {"item_count": 0, "query": query},
        )
    
    # Create a Note for each paper result (text-only content, metadata in tool_metadata)
    note_ids = []
    for result in results:
        text_content = result.get('text', '') or ''
        tool_meta = result.get('metadata', {})
        note_id = _create_note(text_content, agent_name, resource_manager, tool_metadata=tool_meta)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for paper: {tool_meta.get('title', 'unknown')}")
    
    if not note_ids:
        return _fail(executor, 'Failed to create any Notes from search results')
    
    # Create Collection containing all paper Notes
    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return _fail(executor, 'Failed to create Collection')
    
    # Format collection value as "X items [Note_1, ...]"
    item_count = len(note_ids)
    display_ids = note_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'
    collection_value = f"{item_count} items [{note_list_str}]"
    
    logger.info(f"semantic-scholar created Collection {collection_id} with {len(note_ids)} papers for query: {query}")
    return _success(
        executor,
        collection_value,
        collection_id,
        {"item_count": item_count, "query": query, "note_ids": note_ids},
    )

if __name__ == "__main__":
    # Test
    result = tool("transformer architecture", agent_name='Jill')
    print(result)

