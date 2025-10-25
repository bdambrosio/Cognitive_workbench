"""
Standalone test for web-search tool (no Zenoh required).

Tests:
1. Google CSE API (if keys available)
2. HTTP fetch + text extraction
3. Tool function with mocked LLM client
"""

import os
import sys
import importlib.util
from pathlib import Path
from unittest.mock import Mock

# Load web-search tool module (directory has hyphen, can't use standard import)
tool_path = Path(__file__).parent.parent / 'src' / 'maps' / 'tools' / 'web-search' / 'tool.py'
spec = importlib.util.spec_from_file_location("web_search_tool", tool_path)
web_search_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web_search_tool)

# Import functions from loaded module
_google_search = web_search_tool._google_search
_http_get = web_search_tool._http_get
_extract_domain = web_search_tool._extract_domain
tool = web_search_tool.tool


def test_extract_domain():
    """Test domain extraction utility."""
    print("\n=== Testing domain extraction ===")
    test_cases = [
        ("https://www.example.com/path", "www.example.com"),
        ("http://github.com", "github.com"),
        ("https://news.ycombinator.com/item?id=123", "news.ycombinator.com"),
    ]
    
    for url, expected in test_cases:
        result = _extract_domain(url)
        status = "✓" if result == expected else "✗"
        print(f"{status} {url} → {result}")


def test_google_search():
    """Test Google CSE API (requires GOOGLE_API_KEY and GOOGLE_CX)."""
    print("\n=== Testing Google CSE ===")
    
    if not os.getenv('GOOGLE_API_KEY') or not os.getenv('GOOGLE_CX'):
        print("⚠ Skipped: GOOGLE_API_KEY and GOOGLE_CX not set")
        return
    
    try:
        results = _google_search("Python programming language", num=3)
        print(f"✓ Found {len(results)} URLs")
        for i, url in enumerate(results[:3], 1):
            print(f"  {i}. {url}")
    except Exception as e:
        print(f"✗ Failed: {e}")


def test_http_get():
    """Test HTTP fetch."""
    print("\n=== Testing HTTP fetch ===")
    
    test_urls = [
        "https://www.example.com",
        "https://httpbin.org/html",
    ]
    
    for url in test_urls:
        try:
            html = _http_get(url, timeout=5.0)
            if html:
                print(f"✓ {url} → {len(html)} chars")
            else:
                print(f"✗ {url} → No content")
        except Exception as e:
            print(f"✗ {url} → {e}")


def test_tool_function():
    """Test tool() function with mocked LLM client."""
    print("\n=== Testing tool() function ===")
    
    # Mock LLM client
    from llm_client import ZenohLLMClient
    client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
    
    # Test with API keys (if available in environment)
    if os.getenv('GOOGLE_API_KEY') and os.getenv('GOOGLE_CX'):
        print("API keys found in environment - testing real search")
        
        try:
            result = tool("Python programming language", llm_client=client)
            
            if isinstance(result, str) and result.startswith("# Search Results"):
                print(f"✓ Got markdown results ({len(result)} chars)")
                # Print first 500 chars
                print("\nSample output:")
                print(result)
            else:
                print(f"✗ Unexpected result: {result}")
        except Exception as e:
            print(f"✗ Failed: {e}")
    else:
        print("⚠ Skipped: GOOGLE_API_KEY and GOOGLE_CX not set in environment")


if __name__ == "__main__":
    print("=" * 60)
    print("Web Search Tool - Standalone Tests")
    print("=" * 60)
    
    test_extract_domain()
    test_google_search()
    test_http_get()
    test_tool_function()
    
    print("\n" + "=" * 60)
    print("Tests complete")
    print("=" * 60)

