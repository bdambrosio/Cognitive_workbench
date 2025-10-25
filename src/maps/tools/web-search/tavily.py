"""
Web search tool using Tavily API.
"""
import os
import requests

def tool(value, **kwargs):
    """
    Search the web using Tavily API.
    
    Args:
        value: Search query string
        **kwargs: Optional parameters (none used)
    
    Returns:
        Markdown-formatted string with search results
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'Query must be a string'
        }
    
    api_key = os.getenv('TAVILY_API_KEY')
    if not api_key:
        return {
            'status': 'failed',
            'reason': 'TAVILY_API_KEY not found in environment'
        }
    
    # Call Tavily API
    response = requests.post(
        'https://api.tavily.com/search',
        json={
            'api_key': api_key,
            'query': value,
            'max_results': 5
        },
        timeout=10
    )
    
    if response.status_code != 200:
        return {
            'status': 'failed',
            'reason': f'Tavily API error: {response.status_code}'
        }
    
    data = response.json()
    results = data.get('results', [])
    
    if not results:
        return f"# Search Results for: {value}\n\nNo results found."
    
    # Format as markdown
    output = [f"# Search Results for: {value}\n"]
    
    for i, result in enumerate(results, 1):
        title = result.get('title', 'Untitled')
        content = result.get('content', 'No content available')
        url = result.get('url', '')
        
        output.append(f"## {i}. {title}")
        output.append(content)
        output.append(f"Source: {url}\n")
    
    return '\n'.join(output)

