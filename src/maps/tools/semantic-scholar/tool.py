"""
Semantic Scholar academic paper search tool.
"""

import json
import logging
import os
from typing import List, Dict, Any
import zenoh
from zenoh import QueryTarget, ConsolidationMode

logger = logging.getLogger(__name__)

# Open zenoh session for creating Notes and Collections
config = zenoh.Config()
zenoh_session = zenoh.open(config)

def _create_note(content: Any, agent_name: str, source_skill: str = 'semantic-scholar') -> str:
    """Create a Note via Zenoh and return its ID."""
    for reply in zenoh_session.get(
        "cognitive/map/note/create",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        payload=json.dumps({
            'character_name': agent_name,
            'content': content,
            'format': 'json' if isinstance(content, dict) else 'text',
            'source_skill': source_skill,
            'source_value': str(content)[:100]
        }).encode('utf-8'),
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                return response.get('info_id')
        break
    return None

def _create_collection(note_ids: List[str], agent_name: str, source_skill: str = 'semantic-scholar') -> str:
    """Create a Collection via Zenoh and return its ID."""
    for reply in zenoh_session.get(
        "cognitive/map/collection/create",
        target=QueryTarget.BEST_MATCHING,
        consolidation=ConsolidationMode.NONE,
        payload=json.dumps({
            'character_name': agent_name,
            'content': note_ids,
            'format': 'list',
            'source_skill': source_skill,
            'source_value': f'{len(note_ids)} papers'
        }).encode('utf-8'),
        timeout=5.0
    ):
        if reply.ok:
            response = json.loads(reply.ok.payload.to_bytes().decode('utf-8'))
            if response.get('success'):
                collection_id = response.get('info_id')
                logger.info(f"Created Collection {collection_id} with {len(note_ids)} items")
                return collection_id
        break
    logger.error("Failed to create Collection via Zenoh")
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
    
    try:
        from semanticscholar import SemanticScholar
        api_key=None
        sch = SemanticScholar(api_key=api_key)
        
        # Specify all fields upfront to batch fetch in single request
        fields = ['paperId', 'title', 'abstract', 'authors', 'year', 
                  'citationCount', 'venue', 'openAccessPdf', 'externalIds']
        
        papers_iter = sch.search_paper(query, limit=limit, fields=fields)
        
        # Iterate only up to limit to prevent pagination
        results = []
        for paper in papers_iter:
            if len(results) >= limit-1:
                break
            # Extract available fields
            abstract = paper.abstract or "No abstract available"
            title = paper.title or "Untitled"
            authors = [a.name for a in (paper.authors or [])]
            year = paper.year or 0
            citations = paper.citationCount or 0
            venue = paper.venue or ""
            paper_id = paper.paperId or ""
            
            # Get PDF URL - check openAccessPdf first, then externalIds
            pdf_url = None
            if hasattr(paper, 'openAccessPdf') and paper.openAccessPdf:
                pdf_url = paper.openAccessPdf.get('url')
            
            # Get DOI if available
            doi = None
            if hasattr(paper, 'externalIds') and paper.externalIds:
                doi = paper.externalIds.get('DOI')
                # Try ArXiv URL as fallback
                if not pdf_url and 'ArXiv' in paper.externalIds:
                    arxiv_id = paper.externalIds['ArXiv']
                    pdf_url = f"https://arxiv.org/pdf/{arxiv_id}.pdf"
            
            # Build uniform result structure
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
                    "pdf_url": pdf_url
                },
                "char_count": len(abstract)
            }
            results.append(result)
        
        return results
        
    except ImportError:
        # Fallback to direct API calls if library not available
        logger.warning("semanticscholar library not installed, using direct API")
        return _search_papers_direct(query, limit, api_key)
    except Exception as e:
        logger.error(f"Semantic Scholar search failed: {e}")
        return _search_papers_direct(query, limit, api_key)

def _search_papers_direct(query: str, limit: int, api_key: str = None) -> List[Dict[str, Any]]:
    """
    Direct API call fallback if library not available.
    """
    import requests
    
    url = "https://api.semanticscholar.org/graph/v1/paper/search"
    params = {
        "query": query,
        "limit": limit,
        "fields": "paperId,title,abstract,authors,year,citationCount,venue,openAccessPdf,externalIds"
    }
    
    headers = {}
    if api_key:
        headers["x-api-key"] = api_key
    
    try:
        response = requests.get(url, params=params, headers=headers, timeout=10)
        response.raise_for_status()
        data = response.json()
        
        results = []
        for paper in data.get("data", []):
            abstract = paper.get("abstract", "No abstract available")
            title = paper.get("title", "Untitled")
            authors = [a.get("name", "Unknown") for a in paper.get("authors", [])]
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
                    "pdf_url": pdf_url
                },
                "char_count": len(abstract)
            }
            results.append(result)
        
        return results
        
    except Exception as e:
        logger.error(f"Direct API search failed: {e}")
        return []

def tool(value, **kwargs):
    """
    Semantic Scholar search tool.
    
    Args:
        value: Search query string
        **kwargs: agent_name (required), limit (optional)
        
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
    
    limit = kwargs.get('limit', 10)
    
    # Search papers
    results = search_papers(value, limit=limit)
    
    if not results:
        # Return empty Collection for no results
        empty_coll_id = _create_collection([], agent_name)
        if not empty_coll_id:
            return {'status': 'failed', 'reason': 'Failed to create empty Collection'}
        return empty_coll_id
    
    # Create a Note for each paper result (each is structured JSON)
    note_ids = []
    for result in results:
        note_id = _create_note(result, agent_name)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for paper: {result.get('metadata', {}).get('title', 'unknown')}")
    
    if not note_ids:
        return {'status': 'failed', 'reason': 'Failed to create any Notes from search results'}
    
    # Create Collection containing all paper Notes
    collection_id = _create_collection(note_ids, agent_name)
    if not collection_id:
        return {'status': 'failed', 'reason': 'Failed to create Collection'}
    
    logger.info(f"semantic-scholar created Collection {collection_id} with {len(note_ids)} papers for query: {value}")
    return collection_id

if __name__ == "__main__":
    # Test
    result = tool("transformer architecture")
    print(result)

