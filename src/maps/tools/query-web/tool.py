"""
Lightweight LLM-assisted web search + extract (Google CSE) — Selenium-free.

Env vars:
  GOOGLE_API_KEY
  GOOGLE_CX
"""

import concurrent.futures
import json
import logging
import os
import time
import traceback
from datetime import date
from itertools import zip_longest
from typing import List, Dict, Any, Optional
import requests
import urllib.parse as en
import warnings

import wordfreq as wf
from unstructured.partition.html import partition_html

# ------------------------------
# Logging setup
# ------------------------------
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Suppress verbose warnings from unstructured
warnings.filterwarnings('ignore', category=UserWarning, module='unstructured')

# ------------------------------
# Small utilities
# ------------------------------

def _today_prefix() -> str:
    return " as of " + date.today().strftime("%b-%d-%Y")

def _extract_domain(url: str) -> str:
    try:
        from urllib.parse import urlparse
        netloc = urlparse(url).netloc or ""
        return netloc.lower()
    except Exception:
        return ""

def _http_get(url: str, timeout: float = 10.0, headers: Optional[Dict[str, str]] = None) -> Optional[str]:
    try:
        hdrs = {
            "User-Agent": "Mozilla/5.0 (compatible; LLMSearch/1.0; +https://example.org/bot)",
            "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
        }
        if headers:
            hdrs.update(headers)
        r = requests.get(url, timeout=timeout, headers=hdrs)
        if r.status_code == 200 and r.text:
            # basic size guard
            text = r.text
            if len(text) > 2_500_000:  # ~2.5MB hard cap
                return text[:2_500_000]
            return text
    except Exception:
        pass
    return None

# ------------------------------
# Google CSE
# ------------------------------

def _google_search(query: str, num: int = 10) -> List[str]:
    key = os.getenv("GOOGLE_API_KEY", "").strip()
    cx = os.getenv("GOOGLE_CX", "").strip()
    if not key or not cx:
        return []

    # Prefer recent by adding a date-sort if the query implies recency
    sort = ""  # default relevance
    q_lower = query.lower()
    if "today" in q_lower or "latest" in q_lower or "breaking" in q_lower:
        # date-sorted; CSE supports sort expr; leaving blank often works better than bad params
        # You can also inject a time range via qdr if your CX supports it, e.g., 'q=...&sort=date:r:s'
        pass

    try:
        url = (
            "https://www.googleapis.com/customsearch/v1?"
            f"key={en.quote(key)}&cx={en.quote(cx)}&num={num}&q={en.quote(query)}{sort}"
        )
        r = requests.get(url, timeout=10)
        data = r.json()
        items = data.get("items", [])
        links = []
        for it in items:
            link = (it.get("link") or "").strip()
            if link:
                links.append(link)
        return links
    except Exception:
        traceback.print_exc()
        return []

# ------------------------------
# Content selection / extraction
# ------------------------------

def _compute_keyword_weights(keywords: List[str]) -> Dict[str, int]:
    weights: Dict[str, int] = {}
    for kw in keywords:
        z = wf.zipf_frequency(kw, "en")
        w = max(0, int(8 - z))
        if w > 0:
            weights[kw] = w
            parts = kw.split()
            if len(parts) > 1:
                for p in parts:
                    z2 = wf.zipf_frequency(p, "en")
                    w2 = max(0, int((8 - z2) * 0.5))
                    if w2 > 0:
                        weights[p] = max(weights.get(p, 0), w2)
    return weights

def _extract_subtext(text: str, keywords: List[str], keyword_weights: Dict[str, int], max_chars: int) -> str:
    # Split on paragraph boundaries (double newlines or more)
    # Preserve paragraph structure by treating paragraphs as units
    paragraphs = []
    for para in text.split("\n\n"):
        para = para.strip()
        if para:  # Only keep non-empty paragraphs
            paragraphs.append(para)
    
    # Score paragraphs (not individual lines)
    score = []
    for para in paragraphs:
        s = 0
        low = para.lower()
        for kw, w in keyword_weights.items():
            if kw.lower() in low:
                s += w
        score.append((s, para))

    score.sort(key=lambda x: x[0], reverse=True)
    acc = []
    total = 0
    max_score = sum(keyword_weights.values()) if keyword_weights else 1
    # favor high-signal paragraphs while staying within char budget
    for s, para in score:
        if s <= 0:
            continue
        # Account for double newline separator when adding paragraphs
        separator_len = 2 if acc else 0
        if total + len(para) + separator_len > max_chars and s < max_score // 6:
            continue
        acc.append(para)
        total += len(para) + separator_len
        if total >= 2*max_chars:
            break
    # Join paragraphs with double newlines to preserve structure
    result = "\n\n".join(acc)
    if len(result) > max_chars:
        logger.warning(f"Extracted text exceeds max_chars: {len(result)} > {max_chars}")
        result = result[:int(max_chars)]
    return result

def _html_to_text_extract(html: str, query: str, max_chars: int) -> str:
    # partition_html returns elements (titles, narrative text, etc.). We'll join their string forms.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        elements = partition_html(text=html)
    # Join elements with double newlines to preserve paragraph structure
    joined = "\n\n".join(str(e).strip() for e in elements if str(e).strip())
    kws = query.split()
    kw_weights = _compute_keyword_weights(kws)
    return _extract_subtext(joined, kws, kw_weights, max_chars=max_chars), joined

# ------------------------------
# LLM-assisted TL;DR
# ------------------------------

def _llm_tldr(LLM_client, text: str, query: str, max_chars: int, timeout: float = 10.0, heartbeat=None) -> str:
    """
    Your environment should provide LLM_client.generate(...)
    Keep this call shape; fill in your client inside your app.
    """
    prompt = ["Your task is to analyze the following Text to identify if it is relevant to the Query.\n",
              """Query:
{{$query}}

Text:

{{$text}}


Respond using the following JSON format:

{"relevant":<true / false>}

If the text is relevant to the query, set relevant to 'true' , else set relevant to 'false'.
Respond only with the JSON, no commentary, no code fences, no reasoning:


"""]
    try:
        raw = LLM_client.generate(messages=prompt, bindings={"query": query, "text": text}, max_tokens = max_chars, temperature=0.2, is_json=True, timeout=timeout)
        
        # Send heartbeat after LLM call
        if heartbeat:
            heartbeat()
        
        # Accept either dict or string JSON
        if isinstance(raw.text, dict):
            if str(raw.text.get("relevant", "")).lower().startswith("true"):
                return True
            else:
                logger.error(f"No relevant content found for query: {query}")
                return False
        return False
    except Exception as e:
        logger.error(f"LLM TLDR failed: {e}")
        traceback.print_exc()
        return False

# ------------------------------
# URL processing
# ------------------------------

def _is_mostly_numbers(text: str) -> bool:
    """Check if text is mostly digits (frequency dumps, logs, etc.)"""
    alnum = [c for c in text if c.isalnum()]
    if not alnum:
        return False
    digits = sum(1 for c in alnum if c.isdigit())
    return digits / len(alnum) > 0.5

def _detect_format_from_content(content: str, url: str) -> str:
    """Detect format from content and URL (simplified version for query-web)."""
    url_lower = url.lower()
    
    # Check URL extension
    if url_lower.endswith('.pdf'):
        return "pdf"
    if url_lower.endswith(('.md', '.markdown')):
        return "markdown"
    if url_lower.endswith(('.html', '.htm')):
        return "html"
    if url_lower.endswith('.txt'):
        return "text"
    
    # Check content (first 1024 chars)
    content_sample = content[:1024].lower()
    if content.startswith('%PDF'):
        return "pdf"
    if content.startswith('<') or '<html' in content_sample:
        return "html"
    
    # Default to html for web search results
    return "html"

def _process_url(url: str, query: str, client, per_url_timeout: float, max_chars: int, heartbeat=None) -> Dict[str, Any]:
    start = time.time()
    html = _http_get(url, timeout=per_url_timeout)
    if not html:
        return _create_empty_result(url, start)
    
    try:
        # Detect format
        file_format = _detect_format_from_content(html, url)
        
        # Extract filtered text (query-relevant excerpts)
        extract, full_text = _html_to_text_extract(html, query=query, max_chars=max_chars)
        if not extract or len(extract) < 16:
            return _create_empty_result(url, start, file_format)
        
        # Reject garbage content
        if _is_mostly_numbers(extract):
            logger.warning(f"Rejected mostly-numeric content from {url}")
            return _create_empty_result(url, start, file_format)
        
        # LLM filter for relevance
        remaining_time = max(3.0, per_url_timeout - (time.time() - start) - 1.0)
        tldr = _llm_tldr(client, extract, query=query, max_chars=max_chars, timeout=remaining_time, heartbeat=heartbeat)
        filtered_text = extract
        
        # Return uniform structure matching fetch-text
        if tldr:
            return {
            "text": filtered_text,
            "format": file_format,
            "metadata": {
                "source_url": url,
                "domain": _extract_domain(url),
                "elapsed_ms": int((time.time() - start) * 1000)
            },
            "char_count": len(filtered_text)
        }
    except Exception:
        traceback.print_exc()
        return _create_empty_result(url, start)

def _create_empty_result(url: str, start_time: float, file_format: str = "html") -> Dict[str, Any]:
    """Create empty result with uniform structure."""
    return {
        "text": "",
        "format": file_format,
        "metadata": {
            "source_url": url,
            "domain": _extract_domain(url),
            "elapsed_ms": int((time.time() - start_time) * 1000)
        },
        "char_count": 0
    }

# ------------------------------
# Public entry point
# ------------------------------

def llm_search(query: str, client, max_chars: int = 8000, max_urls: int = 10, max_workers: int = 6, wall_time_limit: float = 10.0, heartbeat=None) -> List[Dict[str, Any]]:
    """
    High-level:
      1) Google CSE for initial URL set (two phrasings interleaved).
      2) Concurrently fetch + extract + LLM TL;DR relevant slices.
      3) Return list of {domain, url, extract, elapsed_ms} (only those with content).
    """


    # 1) Build two phrasings: original and LLM-rephrased (like your old flow)
    q_orig = query
    if "today" in query.lower() or "latest" in query.lower():
        q_orig = f"{_today_prefix()} {query}"

    urls_orig = _google_search(q_orig)[:max_urls]

    # LLM rephrase
    rephr = ""
    try:
        response = client.generate(
            messages=[f"""Rephrase the following query as a best-practice google search query. 

#Query:
{query}

Keep any dates, times, locations, keywords, and subject specifiers and any other significant details that narrow the search. 
Respond only with the rephrased query followed by the </end> tag, no commentary, no code fences, no reasoning.
End your response with:
</end>
"""],
            max_tokens=150,
            temperature=0.4,
            is_json=False,
            stops=['</end>']
        )
        
        # Send heartbeat after LLM call
        if heartbeat:
            heartbeat()
        
        rephrase = response.text
    except Exception as e:
        logger.error(f"LLM rephrasing failed: {e}")
        traceback.print_exc()
        rephrase = ""

    if rephrase:
        urls_rephr = _google_search(rephrase)[:max_urls] if rephr else []
    else:
        urls_rephr = []

    urls = [v for v in zip_longest(urls_orig, urls_rephr) for v in v if v]
    # keep order but de-dup
    seen = set()
    interleaved = []
    for u in urls:
        if u not in seen:
            interleaved.append(u)
            seen.add(u)

    # 2) Concurrent process with a global wall-time budget
    results: List[Dict[str, Any]] = []
    in_flight = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as ex:
        t0 = time.time()
        idx = 0
        while (idx < len(interleaved) or in_flight) and (time.time() - t0) < wall_time_limit:
            # launch new tasks while under budget
            while idx < len(interleaved) and len(in_flight) < max_workers and (time.time() - t0) < wall_time_limit:
                url = interleaved[idx]
                idx += 1
                per_url_timeout = max(5.0, wall_time_limit - (time.time() - t0 - 1.0))
                fut = ex.submit(_process_url, url, query, client, per_url_timeout, max_chars/4, heartbeat)
                logger.info(f"Submitted task for url: {url}")
                in_flight.append(fut)

            # harvest any finished
            still = []
            for fut in in_flight:
                if fut.done():
                    try:
                        item = fut.result()
                        logger.info(f"Completed task: {item}")
                        if item and item.get("text"):
                            results.append(item)
                    except Exception as e:
                        logger.error(f"Error processing task: {e}")
                        traceback.print_exc()
                        pass
                    still.append(None)
                else:
                    # Check if we should timeout this future
                    remaining_time = wall_time_limit - (time.time() - t0)
                    if remaining_time <= 0:
                        fut.cancel()
                        still.append(None)
                    else:
                        try:
                            # Wait with timeout - if it completes, get result
                            item = fut.result(timeout=0.1)
                            logger.info(f"Completed task: {item}")
                            if item and item.get("text"):
                                results.append(item)
                            still.append(None)
                        except concurrent.futures.TimeoutError:
                            still.append(fut)
                        except Exception:
                            still.append(None)
            in_flight = [f for f in still if f is not None]
            time.sleep(0.05 if len(in_flight) < max_workers else 0.2)

        # cancel remaining if wall time exceeded
        for fut in in_flight:
            fut.cancel()

    # 3) Optional basic re-rank: prefer domains with text length and query hits
    ql = query.lower()
    def _score(it):
        text = it.get("text", "")
        hit = 2 if ql[:32] in text.lower() else 0
        return (len(text), hit)

    results.sort(key=_score, reverse=True)
    return results

# ------------------------------
# Tool interface for infospace
# ------------------------------

def tool(value, **kwargs):
    """
    Web search tool using Google CSE + LLM extraction.
    
    Args:
        value: Search query string
        **kwargs: Optional llm_client (will create if not provided)
    
    Returns:
        Markdown-formatted string with search results
    """
    if not isinstance(value, str):
        return {
            'status': 'failed',
            'reason': 'Query must be a string'
        }
    
    # Get or create LLM client
    llm_client = kwargs.get('llm_client')
    if not llm_client:
        try:
            from llm_client import ZenohLLMClient
            llm_client = ZenohLLMClient(server_name='vllm', model_name='models/Qwen3-Next:1.5B')
        except Exception as e:
            return {
                'status': 'failed',
                'reason': f'Failed to create LLM client: {e}'
            }
    
    # Check for required API keys
    if not os.getenv('GOOGLE_API_KEY') or not os.getenv('GOOGLE_CX'):
        return {
            'status': 'failed',
            'reason': 'GOOGLE_API_KEY and GOOGLE_CX environment variables required'
        }
    
    # Perform search
    try:
        results = llm_search(
            query=value,
            client=llm_client,
            max_chars=32000,
            max_urls=7,
            max_workers=6,
            wall_time_limit=12.0,
            heartbeat=kwargs.get('heartbeat')
        )
    except Exception as e:
        return {
            'status': 'failed',
            'reason': f'Search failed: {e}'
        }
    
    if not results:
        return json.dumps({
            'query': value,
            'results': [],
            'count': 0
        })
    
    # Return structured JSON matching fetch-text format (uniform structure)
    result_items = []
    for result in results:
        # Result already has uniform structure from _process_url
        result_items.append(result)
    
    return json.dumps({
        'query': value,
        'results': result_items,
        'count': len(result_items)
    }, indent=2)

if __name__ == "__main__":
    from llm_client import ZenohLLMClient
    client = ZenohLLMClient(server_name='openai', model_name='gpt-4.1')
    results = llm_search("What is the weather in Tokyo?", client, max_chars=1000, max_urls=10, max_workers=4, wall_time_limit=16.0)
    print(results)