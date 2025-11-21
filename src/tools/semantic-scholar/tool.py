"""
Semantic Scholar academic paper search tool.
"""

import json
import logging
import os
import time
import requests
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

ss_api_key = os.getenv("SEMANTIC_SCHOLAR_API_KEY") or None

def _create_note(content: Any, agent_name: str, resource_manager, source_skill: str = 'semantic-scholar') -> Optional[str]:
    """Create a Note via resource_manager and return its ID."""
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=content,
        format_type='json' if isinstance(content, dict) else 'text',
        source_skill=source_skill,
        source_value=str(content)[:100],
        note_name='',  # Empty name for auto-generated notes
        extra_props={}  # No extra properties
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
    - After 3 retries: fail gracefully
    """
    import subprocess
    import urllib.parse
    
    # Define the API endpoint URL
    url = f"http://api.semanticscholar.org/graph/v1/paper/search"

    # Define the query parameters
    query_params = {"query": query, "limit": limit, "fields":"paperId,title,abstract,authors,externalIds,year,citationCount,venue,openAccessPdf,url"}

    # Define headers with API key (if provided)
    headers = {}
    if api_key:
        headers["x-api-key"] = api_key

    # Rate limit handling with exponential backoff
    max_retries = 3
    base_delay = 2.0  # Start with 2 seconds
    
    for attempt in range(max_retries + 1):
        # Send the API request
        response = requests.get(url, params=query_params, headers=headers)
        
        # Check response status
        if response.status_code == 200:
            data = response.json()   
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
                    logger.error(f"skipping paper {title} because it has no abstract or PDF URL: {abstract} {pdf_url}")
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
                        "uri": pdf_url  # Standardized URI field for consistency with query-web and search primitives
                    },
                    "char_count": len(abstract)
                }
                results.append(result)
                logger.info(f"Found paper: {title}")
            
            return results
        
        elif response.status_code == 429:
            # Rate limit exceeded
            if attempt < max_retries:
                delay = base_delay * (2 ** attempt)
                logger.warning(f"Rate limit (429) on attempt {attempt + 1}/{max_retries + 1}. Waiting {delay}s before retry...")
                time.sleep(delay)
                continue
            else:
                logger.error(f"Rate limit exceeded after {max_retries} retries. Giving up.")
                return []
        
        else:
            # Other error - log and fail
            logger.error(f"Semantic Scholar API error {response.status_code} on attempt {attempt + 1}")
            if attempt < max_retries:
                # Brief retry for transient errors
                delay = 1.0
                logger.info(f"Retrying after {delay}s...")
                time.sleep(delay)
                continue
            else:
                print("Status:", response.status_code)
                print("Headers:")
                for k, v in response.headers.items():
                    print(f"{k}: {v}")
                return []
    
    # Should not reach here, but handle gracefully
    logger.error("Unexpected error in Semantic Scholar search")
    return []
        
def tool(value, runtime=None, **kwargs):
    """
    Semantic Scholar search tool.
    
    Args:
        value: Search query string
        **kwargs: agent_name (required), resource_manager (required), limit (optional)
        
    Returns:
        Collection ID containing structured Note for each paper result
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'Query must be a string'
        }
    
    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return {
            'status': 'failed',
            'reason': 'agent_name required in kwargs'
        }
    
    resource_manager = kwargs.get('resource_manager')
    if not resource_manager:
        return {
            'status': 'failed',
            'reason': 'resource_manager required in kwargs'
        }
    
    limit = kwargs.get('limit', 10)
    
    # Search papers
    results = search_papers(value, limit=limit)
    
    if not results:
        # Return empty Collection for no results
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return {'status': 'failed', 'reason': 'Failed to create empty Collection'}
        return empty_coll_id
    
    # Create a Note for each paper result (each is structured JSON)
    note_ids = []
    for result in results:
        note_id = _create_note(result, agent_name, resource_manager)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for paper: {result.get('metadata', {}).get('title', 'unknown')}")
    
    if not note_ids:
        return {'status': 'failed', 'reason': 'Failed to create any Notes from search results'}
    
    # Create Collection containing all paper Notes
    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return {'status': 'failed', 'reason': 'Failed to create Collection'}
    
    logger.info(f"semantic-scholar created Collection {collection_id} with {len(note_ids)} papers for query: {value}")
    return collection_id

if __name__ == "__main__":
    # Test
    result = tool("transformer architecture", agent_name='Jill')
    print(result)

