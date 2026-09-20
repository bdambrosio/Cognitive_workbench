"""Tavily /search, returning each result with the page's own text.

For programs that want the pages, not an observation cut to a ReAct budget.
`src/tools/tavily/tool.py` makes its request here too.

Env vars:
  TAVILY_API_KEY — required
"""
import os
from typing import Any, Dict, List

import requests

API_URL = "https://api.tavily.com/search"
# `advanced` costs 2 credits against `basic`'s 1, and is pinned anyway:
# raw page extraction is best-effort on both, but materially more
# reliable here. Measured live 2026-08-19 over three queries — basic
# returned page text for 4 of 9 results, advanced for 9 of 9. A result
# with no page text is a lead, not a source.
SEARCH_DEPTH = "advanced"
_TIMEOUT = 60.0


class TavilyError(RuntimeError):
    """The search could not be made or its answer could not be used."""


def search(query: str, max_results: int = 4) -> List[Dict[str, Any]]:
    """The results for one query: each a dict with `url`, `title`, `content`
    (the query-relevant extract) and `raw_content` (the page as markdown, or
    None when Tavily could not read it). An empty list means Tavily found
    nothing. Raises TavilyError for anything else, including a 200 whose body
    has no `results`: an answer that cannot be read is not "nothing found"."""
    api_key = os.getenv("TAVILY_API_KEY", "").strip()
    if not api_key:
        raise TavilyError("TAVILY_API_KEY is not set")
    try:
        resp = requests.post(
            API_URL,
            headers={"Authorization": f"Bearer {api_key}",
                     "Content-Type": "application/json"},
            json={"query": query, "max_results": max_results,
                  "search_depth": SEARCH_DEPTH, "include_answer": False,
                  "include_raw_content": "markdown"},
            timeout=_TIMEOUT)
    except requests.exceptions.Timeout as e:
        raise TavilyError(f"timed out after {_TIMEOUT}s") from e
    except requests.exceptions.RequestException as e:
        raise TavilyError(f"request failed: {e}") from e
    if resp.status_code != 200:
        raise TavilyError(f"HTTP {resp.status_code}: {resp.text[:200]}")
    try:
        body = resp.json()
    except ValueError as e:
        raise TavilyError(f"unparseable response: {e}") from e
    if not isinstance(body, dict) or "results" not in body:
        raise TavilyError(f"no `results` in the response: {str(body)[:200]}")
    return [r for r in body["results"] or [] if isinstance(r, dict)]
