#!/usr/bin/env python3
"""
Standalone test driver for search-obsidian tool.

Tests the search-obsidian tool with mocked resource manager.
Can test against real MCP server if OBSIDIAN_MCP_URL is configured.

Usage:
    python test_search_obsidian.py [--query "search term"] [--api-key "token"] [--mock-only]
    
Examples:
    # Test with Bearer token
    python test_search_obsidian.py --api-key "top-secret" --query "grobid"
    
    # Test with environment variable
    export OBSIDIAN_MCP_API_KEY="top-secret"
    python test_search_obsidian.py --query "grobid"
"""

import argparse
import json
import os
import sys
from typing import Dict, Any, Optional

# Add parent directory to path to import tool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import tool module - will be reloaded after setting env vars if needed
import tool as tool_module
from tool import tool, _search_obsidian_mcp


# ==========================================
# Mock Resource Manager
# ==========================================

class MockResourceManager:
    """Mock resource manager for testing."""
    
    def __init__(self):
        self.notes = {}
        self.collections = {}
        self.note_counter = 0
        self.collection_counter = 0
    
    def create_note(self, character_name, content, format_type, source_skill, source_value, note_name, extra_props):
        """Create a mock note."""
        self.note_counter += 1
        note_id = f"Note_{self.note_counter:04d}"
        self.notes[note_id] = {
            'character_name': character_name,
            'content': content,
            'format_type': format_type,
            'source_skill': source_skill,
            'source_value': source_value,
            'note_name': note_name,
            'extra_props': extra_props
        }
        return True, note_id, None, None
    
    def create_collection(self, character_name, note_ids, collection_type, description, collection_name, location, extra_props):
        """Create a mock collection."""
        self.collection_counter += 1
        collection_id = f"Collection_{self.collection_counter:04d}"
        self.collections[collection_id] = {
            'character_name': character_name,
            'note_ids': note_ids,
            'collection_type': collection_type,
            'description': description,
            'collection_name': collection_name,
            'location': location,
            'extra_props': extra_props
        }
        return True, collection_id, None, None


# ==========================================
# Test Functions
# ==========================================

def test_search_function(query: str, mock_only: bool = False, verbose: bool = False) -> Dict[str, Any]:
    """Test the _search_obsidian_mcp function directly."""
    print(f"\n{'='*60}")
    print(f"Testing _search_obsidian_mcp with query: '{query}'")
    print(f"{'='*60}")
    
    if mock_only:
        print("⚠️  Mock-only mode: Skipping actual MCP server call")
        return {
            'success': False,
            'reason': 'Mock-only mode',
            'results': []
        }
    
    # Enable debug logging if verbose
    if verbose:
        import logging
        logging.basicConfig(level=logging.DEBUG)
        logger = logging.getLogger('tools.search_obsidian.tool')
        logger.setLevel(logging.DEBUG)
    
    # Try to fetch OpenAPI spec for diagnostics
    import requests
    import os
    base_url = os.getenv('OBSIDIAN_MCP_URL', 'http://127.0.0.1')
    headers = {
        "Content-Type": "application/json",
        "User-Agent": "CognitiveWorkbench/1.0"
    }
    api_key = os.getenv('OBSIDIAN_MCP_API_KEY')
    if api_key:
        headers['Authorization'] = f"Bearer {api_key}"
        print(f"   Using Bearer token authentication")
    else:
        print(f"   ⚠️  No API key set - requests may fail if auth is required")
    
    print(f"\nChecking OpenAPI spec availability...")
    for spec_path in ["/openapi.json", "/api/openapi.json"]:
        try:
            url = f"{base_url.rstrip('/')}{spec_path}"
            response = requests.get(url, headers=headers, timeout=3.0)
            if response.status_code == 200:
                print(f"✅ Found OpenAPI spec at {spec_path}")
                if verbose:
                    spec = response.json()
                    paths = list(spec.get("paths", {}).keys())[:10]
                    print(f"   Available paths: {paths}")
                break
        except Exception as e:
            if verbose:
                print(f"   {spec_path}: {e}")
    
    try:
        results = _search_obsidian_mcp(query, max_results=5)
        print(f"\n✅ Search completed: {len(results)} results")
        
        if results:
            print("\nSample result structure:")
            sample = results[0]
            print(json.dumps(sample, indent=2))
        else:
            print("\n⚠️  No results found. This could mean:")
            print("   - MCP server is not running")
            print("   - No matching notes in vault")
            print("   - Endpoint discovery failed (check server logs)")
        
        return {
            'success': True,
            'results': results,
            'count': len(results)
        }
    except Exception as e:
        print(f"\n❌ Search failed: {e}")
        import traceback
        traceback.print_exc()
        return {
            'success': False,
            'error': str(e),
            'results': []
        }


def test_tool_function(query: str, mock_only: bool = False) -> Dict[str, Any]:
    """Test the tool function with mocked resource manager."""
    print(f"\n{'='*60}")
    print(f"Testing tool() with query: '{query}'")
    print(f"{'='*60}")
    
    # Create mock resource manager
    resource_manager = MockResourceManager()
    
    # Test parameters
    kwargs = {
        'agent_name': 'TestAgent',
        'resource_manager': resource_manager,
        'max_results': 5
    }
    
    try:
        result = tool(query, **kwargs)
        
        print(f"\nTool result:")
        print(json.dumps(result, indent=2))
        
        # Validate result structure
        if result.get('status') == 'success':
            print(f"\n✅ Tool execution successful")
            print(f"   Collection ID: {result.get('resource_id')}")
            print(f"   Value: {result.get('value')}")
            
            # Show created notes if available
            if resource_manager.notes:
                print(f"\n   Created {len(resource_manager.notes)} notes:")
                for note_id, note_data in list(resource_manager.notes.items())[:3]:
                    content_preview = str(note_data['content'])[:100]
                    print(f"     - {note_id}: {content_preview}...")
            
            # Show collection if available
            if resource_manager.collections:
                coll_id = result.get('resource_id')
                if coll_id in resource_manager.collections:
                    coll_data = resource_manager.collections[coll_id]
                    print(f"\n   Collection {coll_id} contains {len(coll_data['note_ids'])} notes")
        
        elif result.get('status') == 'failed':
            print(f"\n❌ Tool execution failed: {result.get('reason')}")
        
        return result
        
    except Exception as e:
        print(f"\n❌ Tool test failed: {e}")
        import traceback
        traceback.print_exc()
        return {
            'status': 'failed',
            'error': str(e)
        }


def test_error_cases():
    """Test error handling."""
    print(f"\n{'='*60}")
    print("Testing error cases")
    print(f"{'='*60}")
    
    resource_manager = MockResourceManager()
    
    # Test 1: Missing query
    print("\n1. Testing missing query...")
    result = tool("", agent_name='TestAgent', resource_manager=resource_manager)
    assert result.get('status') == 'failed', "Should fail with missing query"
    assert 'value parameter required' in result.get('reason', ''), "Should mention value parameter"
    print("   ✅ Correctly handles missing query")
    
    # Test 2: Missing agent_name
    print("\n2. Testing missing agent_name...")
    result = tool("test query", resource_manager=resource_manager)
    assert result.get('status') == 'failed', "Should fail with missing agent_name"
    assert 'agent_name required' in result.get('reason', ''), "Should mention agent_name"
    print("   ✅ Correctly handles missing agent_name")
    
    # Test 3: Missing resource_manager
    print("\n3. Testing missing resource_manager...")
    result = tool("test query", agent_name='TestAgent')
    assert result.get('status') == 'failed', "Should fail with missing resource_manager"
    assert 'resource_manager required' in result.get('reason', ''), "Should mention resource_manager"
    print("   ✅ Correctly handles missing resource_manager")
    
    print("\n✅ All error cases handled correctly")


# ==========================================
# Main
# ==========================================

def main():
    parser = argparse.ArgumentParser(description='Test search-obsidian tool')
    parser.add_argument('--query', default='grobid', help='Search query (default: grobid)')
    parser.add_argument('--mock-only', action='store_true', help='Skip actual MCP server calls, test with mocks only')
    parser.add_argument('--test-errors', action='store_true', help='Test error handling')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output with debug logging')
    parser.add_argument('--api-key', help='Bearer token for authentication (overrides OBSIDIAN_MCP_API_KEY env var)')
    parser.add_argument('--url', help='MCP server URL (overrides OBSIDIAN_MCP_URL env var)')
    args = parser.parse_args()
    
    # Set environment variables - need to reload module since tool.py reads env vars at import time
    if args.api_key:
        os.environ['OBSIDIAN_MCP_API_KEY'] = args.api_key
    if args.url:
        os.environ['OBSIDIAN_MCP_URL'] = args.url
    
    # Reload the tool module to pick up new environment variables
    # This is necessary because tool.py reads OBSIDIAN_MCP_API_KEY at module import time
    import importlib
    importlib.reload(tool_module)
    # Re-import the functions after reload so they use the updated env vars
    global tool, _search_obsidian_mcp
    tool = tool_module.tool
    _search_obsidian_mcp = tool_module._search_obsidian_mcp
    
    print("="*60)
    print("SEARCH-OBSIDIAN TOOL TEST DRIVER")
    print("="*60)
    print(f"MCP URL: {os.getenv('OBSIDIAN_MCP_URL', 'http://127.0.0.1')}")
    api_key = os.getenv('OBSIDIAN_MCP_API_KEY')
    if api_key:
        # Mask the key for display
        masked_key = api_key[:4] + '*' * (len(api_key) - 8) + api_key[-4:] if len(api_key) > 8 else '***'
        print(f"API Key: {masked_key} (set)")
    else:
        print(f"API Key: Not set")
        api_key = 'top-secret'
    print(f"Mock-only mode: {args.mock_only}")
    
    # Test error cases if requested
    if args.test_errors:
        test_error_cases()
        return
    
    # Test search function
    search_result = test_search_function(args.query, mock_only=args.mock_only, verbose=args.verbose)
    
    # Test tool function
    tool_result = test_tool_function(args.query, mock_only=args.mock_only)
    
    # Summary
    print(f"\n{'='*60}")
    print("TEST SUMMARY")
    print(f"{'='*60}")
    
    if not args.mock_only:
        if search_result.get('success'):
            print(f"✅ Search function: {search_result.get('count', 0)} results")
        else:
            print(f"❌ Search function: {search_result.get('error', 'Unknown error')}")
    
    if tool_result.get('status') == 'success':
        print(f"✅ Tool function: Success")
        print(f"   Collection: {tool_result.get('resource_id')}")
    else:
        print(f"❌ Tool function: {tool_result.get('reason', 'Unknown error')}")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()

