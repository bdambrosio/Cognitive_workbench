"""
Obsidian vault tool — read, search, write, and list via Obsidian Local REST API.
"""

import json
import logging
import os
import time
import requests
import yaml
from typing import List, Dict, Any, Optional
from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)


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
    resource_id: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)

# ------------------------------
# Configuration
# ------------------------------

OBSIDIAN_URL = os.getenv("OBSIDIAN_URL", "http://127.0.0.1:27123")
OBSIDIAN_API_KEY = os.getenv("OBSIDIAN_API_KEY", "")


def _headers() -> Dict[str, str]:
    """Build HTTP headers for Obsidian REST API requests."""
    h = {"Content-Type": "application/json", "User-Agent": "CognitiveWorkbench/1.0"}
    if OBSIDIAN_API_KEY:
        h["Authorization"] = f"Bearer {OBSIDIAN_API_KEY}"
    return h


def _normalize_path(file_path: str) -> str:
    """Normalize a vault-relative path for the REST API (ensure leading /)."""
    if file_path.startswith("/vault/"):
        file_path = file_path[7:]
    elif file_path.startswith("vault/"):
        file_path = file_path[6:]
    if not file_path.startswith("/"):
        file_path = "/" + file_path
    return file_path

# ------------------------------
# Helpers for creating Notes/Collections
# ------------------------------

def _create_note(content: Any, agent_name: str, resource_manager, source_skill: str = 'obsidian') -> Optional[str]:
    """Create a Note and return its ID."""
    if not resource_manager:
        return None
    format_type = 'json' if isinstance(content, dict) else 'text'
    success, note_id, error_msg, _ = resource_manager.create_note(
        agent_name, content, format_type, source_skill, str(content)[:100], '', {}
    )
    if success:
        return note_id
    logger.error(f"Failed to create Note: {error_msg}")
    return None


def _create_collection(note_ids: List[str], agent_name: str, resource_manager, source_skill: str = 'obsidian') -> Optional[str]:
    """Create a Collection and return its ID."""
    if not resource_manager:
        return None
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name, note_ids, 'list', source_skill, f'{len(note_ids)} search results', '', {}
    )
    if success:
        return collection_id
    logger.error(f"Failed to create Collection: {error_msg}")
    return None

# ------------------------------
# Action: read
# ------------------------------

def _action_read(executor, agent_name, resource_manager, **kwargs):
    """Read a specific file by vault-relative path."""
    path = kwargs.get('path', '')
    if not path:
        return _fail(executor, 'path parameter required for read action')

    norm = _normalize_path(path)
    url = f"{OBSIDIAN_URL.rstrip('/')}/vault{norm}"
    try:
        resp = requests.get(url, headers=_headers(), timeout=10.0)
        if resp.status_code == 404:
            return _fail(executor, f'File not found: {path}')
        if resp.status_code != 200:
            return _fail(executor, f'Read failed: HTTP {resp.status_code}')
        content = resp.text
    except requests.exceptions.ConnectionError:
        return _fail(executor, 'Cannot connect to Obsidian REST API')
    except Exception as e:
        return _fail(executor, f'Read error: {e}')

    note_id = _create_note(content, agent_name, resource_manager)
    if not note_id:
        return _fail(executor, 'Failed to create Note from file content')
    return _success(executor, f'{path} ({len(content)} chars)', resource_id=note_id,
                    extra={"path": path, "char_count": len(content)})

# ------------------------------
# Action: write
# ------------------------------

def _action_write(executor, agent_name, resource_manager, **kwargs):
    """Create or overwrite a file at a vault-relative path."""
    path = kwargs.get('path', '')
    content = kwargs.get('content', '')
    frontmatter = kwargs.get('frontmatter')

    if not path:
        return _fail(executor, 'path parameter required for write action')
    if not content and not frontmatter:
        return _fail(executor, 'content parameter required for write action')

    # Prepend YAML frontmatter if provided
    body = ''
    if frontmatter and isinstance(frontmatter, dict):
        body = '---\n' + yaml.dump(frontmatter, default_flow_style=False).rstrip() + '\n---\n\n'
    body += content

    norm = _normalize_path(path)
    url = f"{OBSIDIAN_URL.rstrip('/')}/vault{norm}"
    try:
        resp = requests.put(
            url,
            headers={**_headers(), "Content-Type": "text/markdown"},
            data=body.encode('utf-8'),
            timeout=10.0,
        )
        if resp.status_code not in (200, 201, 204):
            return _fail(executor, f'Write failed: HTTP {resp.status_code}')
    except requests.exceptions.ConnectionError:
        return _fail(executor, 'Cannot connect to Obsidian REST API')
    except Exception as e:
        return _fail(executor, f'Write error: {e}')

    logger.info(f"obsidian write: {path} ({len(body)} chars)")
    return _success(executor, f'Wrote {path} ({len(body)} chars)',
                    extra={"path": path, "char_count": len(body)})

# ------------------------------
# Action: list
# ------------------------------

def _action_list(executor, agent_name, resource_manager, **kwargs):
    """List files and subdirectories at a vault-relative path."""
    path = kwargs.get('path', '/')

    # Ensure trailing slash for directory listing
    if not path.endswith('/'):
        path += '/'
    norm = _normalize_path(path)
    url = f"{OBSIDIAN_URL.rstrip('/')}/vault{norm}"
    try:
        resp = requests.get(url, headers=_headers(), timeout=10.0)
        if resp.status_code == 404:
            return _fail(executor, f'Directory not found: {path}')
        if resp.status_code != 200:
            return _fail(executor, f'List failed: HTTP {resp.status_code}')
        data = resp.json()
    except requests.exceptions.ConnectionError:
        return _fail(executor, 'Cannot connect to Obsidian REST API')
    except Exception as e:
        return _fail(executor, f'List error: {e}')

    # Obsidian REST API returns {"files": [...]} for directory listing
    if isinstance(data, dict) and "files" in data:
        file_list = data["files"]
    elif isinstance(data, list):
        file_list = data
    else:
        file_list = []

    # The REST API returns entries relative to the requested path when querying
    # a subdirectory, but returns full vault paths when querying the root.
    # Normalize: strip the prefix if present, otherwise use as-is.
    prefix = path.lstrip('/')
    entries = []
    for f in file_list:
        if not isinstance(f, str):
            continue
        # If entry has the prefix, strip it; otherwise it's already relative
        if prefix and f.startswith(prefix):
            relative = f[len(prefix):]
        else:
            relative = f
        if not relative:
            continue
        # Only show immediate children (no nested paths unless it's a dir marker)
        if '/' in relative.rstrip('/'):
            continue
        entries.append(relative)

    listing_text = f"# {path}\n\n" + '\n'.join(f'- {e}' for e in sorted(entries)) if entries else f"# {path}\n\n(empty)"
    note_id = _create_note(listing_text, agent_name, resource_manager)
    if not note_id:
        return _fail(executor, 'Failed to create Note for directory listing')
    return _success(executor, f'{len(entries)} entries in {path}', resource_id=note_id,
                    extra={"path": path, "entry_count": len(entries), "entries": entries[:50]})

# ------------------------------
# Action: search (original behavior)
# ------------------------------

def _action_search(executor, agent_name, resource_manager, **kwargs):
    """Search Obsidian vault by query string."""
    query = kwargs.get('query') or kwargs.get('input_value', '')
    if not query:
        return _fail(executor, 'query parameter required for search action')

    max_results = kwargs.get('max_results', 10)
    headers = _headers()

    # Get file list
    vault_url = f"{OBSIDIAN_URL.rstrip('/')}/vault/"
    try:
        response = requests.get(vault_url, headers=headers, timeout=10.0)
        if response.status_code != 200:
            return _fail(executor, f'Failed to get vault file list: HTTP {response.status_code}')

        data = response.json()
        if isinstance(data, dict) and "files" in data:
            file_list = data["files"]
        elif isinstance(data, list):
            file_list = data
        else:
            return _fail(executor, f'Unexpected vault response format')

    except requests.exceptions.ConnectionError:
        return _fail(executor, 'Cannot connect to Obsidian REST API')
    except Exception as e:
        return _fail(executor, f'Search error: {e}')

    # Filter: filename match first, then content match
    query_lower = query.lower()
    filename_matches = []
    other_md_files = []

    for fp in file_list:
        if not isinstance(fp, str) or fp.endswith('/') or not fp.endswith('.md'):
            continue
        filename = fp.split('/')[-1].lower()
        if query_lower in filename:
            filename_matches.append(fp)
        else:
            other_md_files.append(fp)

    # Candidates: all filename matches + enough others to check content
    candidates = filename_matches + other_md_files[:max_results * 2]

    start_time = time.time()
    results = []
    for fp in candidates:
        if len(results) >= max_results:
            break
        content = _fetch_note_content(fp, headers)
        if content and (query_lower in content.lower() or query_lower in fp.lower()):
            path_uri = f"obsidian://vault/{fp}" if not fp.startswith('/') else f"obsidian://vault{fp}"
            results.append({
                "text": content,
                "format": "markdown",
                "metadata": {
                    "source_url": path_uri,
                    "uri": path_uri,
                    "domain": "obsidian",
                    "path": fp,
                    "elapsed_ms": int((time.time() - start_time) * 1000),
                },
                "char_count": len(content),
            })

    if not results:
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(executor, '0 items []', empty_coll_id,
                        {"item_count": 0, "query": query})

    note_ids = []
    for r in results:
        nid = _create_note(r, agent_name, resource_manager)
        if nid:
            note_ids.append(nid)

    if not note_ids:
        return _fail(executor, 'Failed to create any Notes from search results')

    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return _fail(executor, 'Failed to create Collection')

    item_count = len(note_ids)
    display_ids = note_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'

    logger.info(f"obsidian search: {item_count} results for '{query}'")
    return _success(executor, f'{item_count} items [{note_list_str}]', collection_id,
                    {"item_count": item_count, "query": query, "note_ids": note_ids})


def _fetch_note_content(file_path: str, headers: Dict[str, str]) -> Optional[str]:
    """Fetch the content of a note from Obsidian Local REST API."""
    norm = _normalize_path(file_path)
    url = f"{OBSIDIAN_URL.rstrip('/')}/vault{norm}"
    try:
        response = requests.get(url, headers=headers, timeout=10.0)
        if response.status_code == 200:
            return response.text
        return None
    except Exception:
        return None

# ------------------------------
# Public entry point
# ------------------------------

_ACTIONS = {
    'search': _action_search,
    'read': _action_read,
    'write': _action_write,
    'list': _action_list,
}


def tool(input_value=None, **kwargs):
    """
    Obsidian vault tool — dispatches to action handlers.

    Actions: search, read, write, list
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return _fail(executor, 'agent_name required')

    resource_manager = kwargs.get('resource_manager')
    if not resource_manager:
        return _fail(executor, 'resource_manager required')

    action = kwargs.get('action', '').strip().lower()

    # Backward compatibility: no action specified → treat as search
    if not action:
        action = 'search'
        if not kwargs.get('query'):
            kwargs['query'] = input_value or kwargs.get('value', '')

    handler = _ACTIONS.get(action)
    if not handler:
        return _fail(executor, f"Unknown action '{action}'. Valid: {', '.join(_ACTIONS.keys())}")

    # Pass input_value through for search backward compat
    kwargs['input_value'] = input_value
    # Executor merges executor/agent_name/resource_manager into kwargs; handlers take those
    # positionally — omit from **kwargs to avoid "multiple values for argument" TypeError.
    _omit = frozenset(('executor', 'agent_name', 'resource_manager'))
    handler_kwargs = {k: v for k, v in kwargs.items() if k not in _omit}
    return handler(executor, agent_name, resource_manager, **handler_kwargs)
