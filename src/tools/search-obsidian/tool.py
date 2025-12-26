"""
Obsidian Local REST API search tool.
Searches Obsidian notes via Obsidian Local REST API plugin and returns results in search-web format.
"""

import json
import logging
import os
import time
import requests
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

# ------------------------------
# Configuration
# ------------------------------

# Default to Obsidian Local REST API port
OBSIDIAN_URL = os.getenv("OBSIDIAN_URL", "http://127.0.0.1:27123")
OBSIDIAN_API_KEY = os.getenv("OBSIDIAN_API_KEY", "")

# ------------------------------
# Helpers for creating Notes/Collections
# ------------------------------

def _create_note(content: Any, agent_name: str, resource_manager, source_skill: str = 'search-obsidian') -> Optional[str]:
    """Create a Note and return its ID."""
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    format_type = 'json' if isinstance(content, dict) else 'text'
    success, note_id, error_msg, location = resource_manager.create_note(
        agent_name, content, format_type, source_skill, str(content)[:100], '', {}
    )
    
    if success:
        return note_id
    else:
        logger.error(f"Failed to create Note: {error_msg}")
        return None

def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'search-obsidian') -> Optional[str]:
    """Create a Collection and return its ID."""
    if not resource_manager:
        logger.error("Resource manager not available")
        return None
    
    success, collection_id, error_msg, location = resource_manager.create_collection(
        agent_name, note_ids, 'list', source_skill, f'{len(note_ids)} search results', '', {}
    )
    
    if success:
        logger.info(f"Created Collection {collection_id} with {len(note_ids)} items")
        return collection_id
    else:
        logger.error(f"Failed to create Collection: {error_msg}")
        return None

# ------------------------------
# MCP Server API calls
# ------------------------------

def _discover_endpoints_from_openapi(base_url: str) -> List[str]:
    """
    Try to fetch OpenAPI spec and discover search endpoints.
    
    Args:
        base_url: Base URL of MCP server
        
    Returns:
        List of discovered endpoint paths
    """
    endpoints = []
    
    # Try to fetch OpenAPI spec
    openapi_paths = ["/openapi.json", "/api/openapi.json", "/swagger.json"]
    
    headers = {
        "Content-Type": "application/json",
        "User-Agent": "CognitiveWorkbench/1.0"
    }
    
    if OBSIDIAN_API_KEY:
        headers["Authorization"] = f"Bearer {OBSIDIAN_API_KEY}"
    
    for spec_path in openapi_paths:
        try:
            url = f"{base_url.rstrip('/')}{spec_path}"
            response = requests.get(url, headers=headers, timeout=5.0)
            if response.status_code == 200:
                spec = response.json()
                # Look for paths that contain "search" or "note"
                paths = spec.get("paths", {})
                for path, methods in paths.items():
                    if "search" in path.lower() or "note" in path.lower():
                        endpoints.append(path)
                logger.info(f"Discovered {len(endpoints)} endpoints from OpenAPI spec")
                return endpoints
        except Exception as e:
            logger.debug(f"Failed to fetch OpenAPI spec from {spec_path}: {e}")
    
    return []


def _fetch_note_content(file_path: str, headers: Dict[str, str]) -> Optional[str]:
    """
    Fetch the content of a note from Obsidian Local REST API.
    
    Args:
        file_path: Path to the note file (e.g., "note.md" or "/folder/note.md")
        headers: HTTP headers with authentication
        
    Returns:
        Note content as string, or None if fetch fails
    """
    # Ensure path doesn't start with /vault
    if file_path.startswith("/vault/"):
        file_path = file_path[7:]
    elif file_path.startswith("vault/"):
        file_path = file_path[6:]
    
    # Ensure path starts with /
    if not file_path.startswith("/"):
        file_path = "/" + file_path
    
    url = f"{OBSIDIAN_URL.rstrip('/')}/vault{file_path}"
    
    try:
        response = requests.get(url, headers=headers, timeout=10.0)
        if response.status_code == 200:
            # Obsidian Local REST API returns text/markdown content
            return response.text
        else:
            logger.debug(f"Failed to fetch {file_path}: status={response.status_code}")
            return None
    except Exception as e:
        logger.debug(f"Error fetching {file_path}: {e}")
        return None


def _search_obsidian(query: str, max_results: int = 10) -> List[Dict[str, Any]]:
    """
    Search Obsidian notes via Obsidian Local REST API.
    
    Strategy:
    1. Get list of all files from /vault/
    2. Filter files that match the query (by filename or fetch and check content)
    3. Fetch content for matching files
    4. Return results in uniform format
    
    Args:
        query: Search query string
        max_results: Maximum number of results to return
        
    Returns:
        List of note dicts with uniform structure
    """
    results = []
    
    headers = {
        "Content-Type": "application/json",
        "User-Agent": "CognitiveWorkbench/1.0"
    }
    
    # Add API key if provided
    if OBSIDIAN_API_KEY:
        headers["Authorization"] = f"Bearer {OBSIDIAN_API_KEY}"
    
    # Step 1: Get list of all files from /vault/
    vault_url = f"{OBSIDIAN_URL.rstrip('/')}/vault/"
    try:
        response = requests.get(vault_url, headers=headers, timeout=10.0)
        if response.status_code != 200:
            logger.warning(f"Failed to get vault file list: status={response.status_code}")
            return []
        
        # Parse response - Obsidian Local REST API returns JSON object with "files" key
        data = response.json()
        if isinstance(data, dict) and "files" in data:
            file_list = data["files"]
        elif isinstance(data, list):
            file_list = data
        else:
            logger.warning(f"Unexpected vault response format: {type(data)}")
            return []
        
        if not isinstance(file_list, list):
            logger.warning(f"File list is not an array: {type(file_list)}")
            return []
        
        logger.info(f"Found {len(file_list)} files in vault")
        
        # Step 2: Filter files that match the query (by filename)
        query_lower = query.lower()
        matching_files = []
        
        for file_path in file_list:
            if not isinstance(file_path, str):
                continue
            
            # Skip directories (they end with /)
            if file_path.endswith('/'):
                continue
            
            # Filter to markdown files
            if not file_path.endswith('.md'):
                continue
            
            # Check if filename contains query
            filename = file_path.split('/')[-1].lower()
            if query_lower in filename:
                matching_files.append(file_path)
        
        # If not enough matches by filename, fetch content and search
        if len(matching_files) < max_results:
            # Try fetching content for more files
            for file_path in file_list:
                if len(matching_files) >= max_results * 2:  # Get extra candidates
                    break
                if file_path not in matching_files and file_path.endswith('.md'):
                    matching_files.append(file_path)
        
        # Step 3: Fetch content for matching files and filter by content
        final_results = []
        for file_path in matching_files[:max_results * 2]:  # Fetch more than needed
            content = _fetch_note_content(file_path, headers)
            if content:
                # Check if content matches query
                if query_lower in content.lower() or query_lower in file_path.lower():
                    final_results.append({
                        "path": file_path,
                        "text": content,
                        "name": file_path.split('/')[-1]
                    })
                    if len(final_results) >= max_results:
                        break
        
        # Step 4: Convert to uniform format
        results = _parse_mcp_response(final_results, query)
        logger.info(f"Found {len(results)} matching notes for query '{query}'")
        return results
        
    except requests.exceptions.ConnectionError as e:
        logger.error(f"Connection error to Obsidian API: {e}")
        return []
    except Exception as e:
        logger.error(f"Error searching Obsidian vault: {e}")
        import traceback
        traceback.print_exc()
        return []

def _parse_mcp_response(data: Any, query: str) -> List[Dict[str, Any]]:
    """
    Parse MCP server response into uniform note format.
    
    Handles various response formats:
    - Direct array of notes
    - Wrapped in 'results' or 'data' key
    - MCP tool response format with 'content'
    """
    notes = []
    
    # Handle different response structures
    if isinstance(data, list):
        raw_notes = data
    elif isinstance(data, dict):
        # Try common keys
        if "results" in data:
            raw_notes = data["results"]
        elif "data" in data:
            raw_notes = data["data"]
        elif "content" in data:
            # MCP tool response format
            content = data["content"]
            if isinstance(content, list):
                raw_notes = content
            elif isinstance(content, dict) and "text" in content:
                # Single note in content.text
                raw_notes = [{"text": content["text"], "path": content.get("path", "")}]
            else:
                raw_notes = []
        elif "notes" in data:
            raw_notes = data["notes"]
        else:
            # Try to extract if it's a single note
            if "text" in data or "content" in data:
                raw_notes = [data]
            else:
                raw_notes = []
    else:
        raw_notes = []
    
    # Convert to uniform format
    for note in raw_notes:
        if not isinstance(note, dict):
            continue
        
        # Extract text content
        text = note.get("text") or note.get("content") or note.get("body") or ""
        if not text:
            continue
        
        # Extract path/URI
        path = note.get("path") or note.get("file") or note.get("uri") or ""
        if not path:
            # Try to construct from title
            title = note.get("title") or ""
            if title:
                path = f"obsidian://vault/{title.replace(' ', '-').lower()}.md"
            else:
                path = "obsidian://vault/note.md"
        
        # Ensure obsidian:// URI format
        if not path.startswith("obsidian://"):
            if path.startswith("/"):
                path = f"obsidian://vault{path}"
            else:
                path = f"obsidian://vault/{path}"
        
        # Create uniform result structure matching search-web
        result = {
            "text": text,
            "format": "markdown",
            "metadata": {
                "source_url": path,
                "uri": path,
                "domain": "obsidian",
                "elapsed_ms": 0  # Will be set by caller
            },
            "char_count": len(text)
        }
        
        notes.append(result)
    
    return notes

# ------------------------------
# Public entry point
# ------------------------------

def tool(input_value, **kwargs):
    """
    Obsidian Local REST API search tool.
    
    Searches Obsidian vault by:
    1. Getting file list from /vault/
    2. Filtering markdown files matching query in filename
    3. Fetching content for matching files
    4. Filtering by content match
    
    Args:
        input_value: Query string (preferred)
        **kwargs: legacy query (fallback), agent_name (required), resource_manager (required), max_results (optional)
    
    Returns:
        Collection ID containing structured Note for each search result
    """
    query = input_value or kwargs.get('value') or kwargs.get('query', '')
    if not isinstance(query, str):
        query = ''
    if not query:
        return {
            'status': 'failed',
            'reason': 'input_value parameter required (search query)'
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
    
    max_results = kwargs.get('max_results', 10)
    
    # Perform search
    start_time = time.time()
    try:
        results = _search_obsidian(query, max_results=max_results)
        
        # Set elapsed_ms for each result
        elapsed_ms = int((time.time() - start_time) * 1000)
        for result in results:
            result["metadata"]["elapsed_ms"] = elapsed_ms
            
    except Exception as e:
        logger.error(f"Obsidian MCP search failed: {e}")
        import traceback
        traceback.print_exc()
        return {
            'status': 'failed',
            'reason': f'Search failed: {e}'
        }
    
    if not results:
        # Return empty Collection for no results
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return {'status': 'failed', 'reason': 'Failed to create empty Collection', 'value': None, 'resource_id': None}
        return {'status': 'success', 'value': '0 items []', 'resource_id': empty_coll_id}
    
    # Create a Note for each result (each is structured JSON)
    note_ids = []
    for result in results:
        note_id = _create_note(result, agent_name, resource_manager)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for result: {result.get('metadata', {}).get('uri', 'unknown')}")
    
    if not note_ids:
        return {'status': 'failed', 'reason': 'Failed to create any Notes from search results', 'value': None, 'resource_id': None}
    
    # Create Collection containing all result Notes
    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return {'status': 'failed', 'reason': 'Failed to create Collection', 'value': None, 'resource_id': None}
    
    # Format collection value as "X items [Note_1, ...]"
    item_count = len(note_ids)
    display_ids = note_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'
    collection_value = f"{item_count} items [{note_list_str}]"
    
    logger.info(f"search-obsidian created Collection {collection_id} with {len(note_ids)} results for query: {query}")
    return {'status': 'success', 'value': collection_value, 'resource_id': collection_id}

if __name__ == "__main__":
    # Test
    result = tool("machine learning", agent_name='TestAgent')
    print(result)

