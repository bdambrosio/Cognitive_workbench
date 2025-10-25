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
    # very simple paragraph-ish splitting; unstructured already normalizes blocks
    blocks = [blk.strip() for blk in text.split("\n") if blk.strip()]
    score = []
    for blk in blocks:
        s = 0
        low = blk.lower()
        for kw, w in keyword_weights.items():
            if kw.lower() in low:
                s += w
        score.append((s, blk))

    score.sort(key=lambda x: x[0], reverse=True)
    acc = []
    total = 0
    max_score = sum(keyword_weights.values()) if keyword_weights else 1
    # favor high-signal blocks while staying within char budget
    for s, blk in score:
        if s <= 0:
            continue
        if total + len(blk) + 1 > max_chars and s < max_score // 10:
            continue
        acc.append(blk)
        total += len(blk) + 1
        if total >= 2*max_chars:
            break
    return "\n".join(acc)

def _html_to_text_extract(html: str, query: str, max_chars: int) -> str:
    # partition_html returns elements (titles, narrative text, etc.). We'll join their string forms.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        elements = partition_html(text=html)
    joined = "\n".join(str(e).strip() for e in elements if str(e).strip())
    kws = query.split()
    kw_weights = _compute_keyword_weights(kws)
    return _extract_subtext(joined, kws, kw_weights, max_chars=max_chars)

# ------------------------------
# LLM-assisted TL;DR
# ------------------------------

def _llm_tldr(LLM_client, text: str, query: str, max_chars: int) -> str:
    """
    Your environment should provide LLM_client.generate(...)
    Keep this call shape; fill in your client inside your app.
    """
    prompt = ["Your task is to analyze the following Text to identify content relevant to the Query.\n",
              """Query:
{{$query}}

Text:

{{$text}}


Respond using the following JSON format. 
If there is relevant content, set relevant to 'true' and set extract to the relevant content.
If there is no relevant content, set relevant to 'false'.
Respond only with the JSON, no commentary, no code fences, no reasoning    :
{"relevant":<true / false>, "extract":"<only relevant content>"}

"""]
    try:
        raw = LLM_client.generate(messages=prompt, bindings={"query": query, "text": text}, max_tokens = max_chars, temperature=0.2, is_json=True)
        # Accept either dict or string JSON
        if isinstance(raw.text, dict):
            if str(raw.text.get("relevant", "")).lower().startswith("true"):
                return raw.text.get("extract", "")
            else:
                logger.error(f"No relevant content found for query: {query}")
                return ""
    except Exception as e:
        traceback.print_exc()
        pass
    return raw.text

# ------------------------------
# URL processing
# ------------------------------

def _process_url(url: str, query: str, client, per_url_timeout: float, max_chars: int) -> Dict[str, Any]:
    start = time.time()
    html = _http_get(url, timeout=per_url_timeout)
    if not html:
        return {"url": url, "domain": _extract_domain(url), "extract": "", "elapsed_ms": int((time.time()-start)*1000)}
    try:
        extract = _html_to_text_extract(html, query=query, max_chars=max_chars)
        if not extract or len(extract) < 16:
            return {"url": url, "domain": _extract_domain(url), "extract": "", "elapsed_ms": int((time.time()-start)*1000)}
        tldr = _llm_tldr(client, extract, query=query, max_chars=max_chars)
        return {"url": url, "domain": _extract_domain(url), "extract": tldr or extract, "elapsed_ms": int((time.time()-start)*1000)}
    except Exception:
        traceback.print_exc()
        return {"url": url, "domain": _extract_domain(url), "extract": "", "elapsed_ms": int((time.time()-start)*1000)}

# ------------------------------
# Public entry point
# ------------------------------

def llm_search(query: str, client, max_chars: int = 8000, max_urls: int = 10, max_workers: int = 4, wall_time_limit: float = 16.0) -> List[Dict[str, Any]]:
    """
    High-level:
      1) Google CSE for initial URL set (two phrasings interleaved).
      2) Concurrently fetch + extract + LLM TL;DR relevant slices.
      3) Return list of {domain, url, extract, elapsed_ms} (only those with content).
    """
    t0 = time.time()

    # 1) Build two phrasings: original and LLM-rephrased (like your old flow)
    q_orig = query
    if "today" in query.lower() or "latest" in query.lower():
        q_orig = f"{_today_prefix()} {query}"

    urls_orig = _google_search(q_orig)[:max_urls]

    # LLM rephrase
    rephr = ""
    try:
        rephr = client.generate(
            prompt=[f"""Rephrase the following query as a best-practice google search query. 
Keep any dates, times, locations, keywords, and subject specifiers and any other significant details that narrow the search. 
Respond only with the rephrased query, no commentary, no code fences, no reasoning.
#Query:
{query}"""],
            max_tokens=150,
            temperature=0.2,
        )
        if isinstance(rephr, dict):
            rephr = rephr.get("text") or rephr.get("content") or ""
        rephr = str(rephr).strip()
    except Exception:
        rephr = ""

    urls_rephr = _google_search(rephr)[:max_urls] if rephr else []

    # de-dup + interleave
    set_rephr = set(urls_rephr)
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
        idx = 0
        while (idx < len(interleaved) or in_flight) and (time.time() - t0) < wall_time_limit:
            # launch new tasks while under budget
            while idx < len(interleaved) and len(in_flight) < max_workers and (time.time() - t0) < wall_time_limit:
                url = interleaved[idx]
                idx += 1
                per_url_timeout = max(3.0, wall_time_limit - (time.time() - t0) - 1.0)
                fut = ex.submit(_process_url, url, query, client, per_url_timeout, max_chars)
                logger.info(f"Submitted task for url: {url}")
                in_flight.append(fut)

            # harvest any finished
            still = []
            for fut in in_flight:
                if fut.done():
                    try:
                        item = fut.result()
                        logger.info(f"Completed task: {item}")
                        if item.get("extract"):
                            results.append(item)
                    except Exception:
                        # swallow and continue
                        pass
                else:
                    still.append(fut)
            in_flight = still
            time.sleep(0.05 if len(in_flight) < max_workers else 0.2)

        # cancel remaining if wall time exceeded
        for fut in in_flight:
            fut.cancel()

    # 3) Optional basic re-rank: prefer domains with extract length and query hits
    ql = query.lower()
    def _score(it):
        ext = it.get("extract", "")
        hit = 2 if ql[:32] in ext.lower() else 0
        return (len(ext), hit)

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
            max_chars=1000,
            max_urls=10,
            max_workers=4,
            wall_time_limit=16.0
        )
    except Exception as e:
        return {
            'status': 'failed',
            'reason': f'Search failed: {e}'
        }
    
    if not results:
        return f"# Search Results for: {value}\n\nNo results found."
    
    # Format as markdown
    output = [f"# Search Results for: {value}\n"]
    
    for i, result in enumerate(results, 1):
        url = result.get('url', '')
        domain = result.get('domain', _extract_domain(url))
        extract = result.get('extract', 'No content available')
        
        output.append(f"## {i}. {domain}")
        output.append(extract)
        output.append(f"Source: {url}\n")
    
    return '\n'.join(output)

if __name__ == "__main__":
    from llm_client import ZenohLLMClient
    client = ZenohLLMClient(server_name='openai', model_name='gpt-4.1')
    results = llm_search("What is the weather in Tokyo?", client, max_chars=1000, max_urls=10, max_workers=4, wall_time_limit=16.0)
    print(results)