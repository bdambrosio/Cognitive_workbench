"""
Web search returning the sources' own text (Tavily /search).

The complement to `search-web`: that tool hands the agent one model's
synthesis and keeps the documents at arm's length; this one hands over
ranked pages with their own words attached, so a claim built on the
result rests on a document rather than on a summary of one. That is why
`tavily` is listed in claims._DIRECT_OBSERVATION_TOOLS and `search-web`
is not.

Env vars:
  TAVILY_API_KEY — required

The output contract matches search-web's: an observation the agent
reads, plus `meta` sources carrying the URL of every page quoted, so the
justify trail can resolve claim → document.
"""

import logging
import os
import re
from typing import Any, Dict, List
from urllib.parse import urlparse

import requests

logger = logging.getLogger(__name__)

API_URL = "https://api.tavily.com/search"

# `advanced` costs 2 credits against `basic`'s 1, and is pinned anyway:
# raw page extraction is best-effort on both, but materially more
# reliable here. Measured live 2026-08-19 over three queries — basic
# returned page text for 4 of 9 results, advanced for 9 of 9. A result
# with no page text is a lead, not a source, which is the one thing this
# tool exists to provide.
SEARCH_DEPTH = "advanced"

_DEFAULT_RESULTS = 4
_MAX_RESULTS = 6
_SNIPPET_CAP = 500       # chars of one result's snippet
_PER_RESULT_CAP = 2000   # chars of one result's page text
# One budget across snippets AND page text. The first cut governed page
# text only and let four advanced-depth snippets through uncapped, for a
# 17.8k-char observation on a four-result call — twice search-web's
# average and more than fetch-text's whole page cap. `content` is a few
# sentences on basic depth but runs to 2000 chars of multi-chunk extract
# on advanced, which is what this tool always asks for.
_TOTAL_CAP = 8_000
_TIMEOUT = 60.0

# Markdown noise: an image embed carries no text worth reading, and a
# link's target is a URL the agent cannot follow from inside an
# observation anyway — the result URLs it *can* follow are printed
# separately above. Both eat the budget badly: on the Instagram result of
# the first live call, CDN image URLs consumed nearly the whole 2500-char
# allowance while the snippet carried the address and phone in 200. The
# alternation tolerates one level of nested parens so `[(510)
# 848-0114](tel:(510) 848-0114)` collapses to the number rather than
# splitting mid-URL. Deliberately not a general markdown stripper: one
# call site, and fetch-text keeps its image references on purpose.
_MD_IMAGE = re.compile(r'!\[[^\]]*\]\((?:[^()]|\([^()]*\))*\)')
_MD_LINK = re.compile(r'\[([^\]]*)\]\((?:[^()]|\([^()]*\))*\)')


def _strip_md_noise(text: str) -> str:
    """Drop image embeds and collapse links to their text."""
    return _MD_LINK.sub(r'\1', _MD_IMAGE.sub('', text))


def _clean(text: Any) -> str:
    """Collapse whitespace runs so one field cannot break the layout of
    the observation block it sits in."""
    return " ".join(str(text or "").split())


def _domain(url: str) -> str:
    try:
        return urlparse(url).netloc.lower().removeprefix("www.")
    except ValueError as e:
        logger.warning(f"tavily: cannot parse domain from {url!r}: {e}")
        return ""


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    log = logger or logging.getLogger(__name__)

    query = args.get("query", "")
    if not isinstance(query, str) or not query.strip():
        return {"status": "error", "text": "tavily requires non-empty `query`"}

    api_key = os.getenv("TAVILY_API_KEY", "").strip()
    if not api_key:
        return {"status": "error",
                "text": "TAVILY_API_KEY is not set; use search-web instead"}

    try:
        max_results = int(args.get("max_results") or _DEFAULT_RESULTS)
    except (TypeError, ValueError):
        log.warning(f"tavily: unusable max_results {args.get('max_results')!r}; "
                    f"using {_DEFAULT_RESULTS}")
        max_results = _DEFAULT_RESULTS
    max_results = max(1, min(_MAX_RESULTS, max_results))

    try:
        resp = requests.post(
            API_URL,
            headers={"Authorization": f"Bearer {api_key}",
                     "Content-Type": "application/json"},
            json={"query": query,
                  "max_results": max_results,
                  "search_depth": SEARCH_DEPTH,
                  "include_answer": False,
                  "include_raw_content": "markdown"},
            timeout=_TIMEOUT,
        )
    except requests.exceptions.Timeout:
        return {"status": "error", "text": f"tavily timed out after {_TIMEOUT}s"}
    except Exception as e:
        log.error(f"tavily: request failed: {e}")
        return {"status": "error", "text": f"tavily request failed: {e}"}

    if resp.status_code != 200:
        log.error(f"tavily: HTTP {resp.status_code}: {resp.text[:300]}")
        return {"status": "error",
                "text": f"tavily API error {resp.status_code}: {resp.text[:200]}"}

    try:
        results = resp.json().get("results") or []
    except ValueError as e:
        log.error(f"tavily: unparseable response: {e}")
        return {"status": "error", "text": f"tavily returned unparseable JSON: {e}"}

    if not results:
        return {"status": "empty", "text": f"tavily found nothing for {query!r}"}

    # Snippets are claimed off the budget first, before any page text.
    # They are query-relevant extract where the head of a page is
    # whatever the nav bar says, and on the first live call they were
    # repeatedly the part that carried the answer. Taking them up front
    # also makes the budget hold: charging them as the loop went meant a
    # result reached after the budget ran dry still printed its snippet,
    # for 12.5k chars on a six-result call against a 8k cap.
    snippets: List[str] = []
    for r in results:
        s = _clean(_strip_md_noise(str(r.get("content") or "")))
        if len(s) > _SNIPPET_CAP:
            s = s[:_SNIPPET_CAP].rstrip() + " …[snippet truncated]"
        snippets.append(s)
    spent = sum(len(s) for s in snippets)

    blocks: List[str] = []
    sources: List[Dict[str, str]] = []
    missing = 0
    for i, (r, snippet) in enumerate(zip(results, snippets), 1):
        url = _clean(r.get("url"))
        title = _clean(r.get("title")) or "(untitled)"
        lines = [f"[{i}] {title}", f"    {url}"]
        if snippet:
            lines.append(f"snippet: {snippet}")

        # The page's own text, against a per-result cap and whatever the
        # snippets left of the shared budget. Extraction is best-effort
        # on Tavily's side, and a result that came back without it has to
        # say so: a snippet is not a read of the page, and an agent told
        # otherwise would cite it as one.
        raw = _strip_md_noise(str(r.get("raw_content") or "")).strip()
        if not raw:
            missing += 1
            lines.append("page: [not extracted — snippet only; fetch-text this "
                         "url to read the page]")
        else:
            room = min(_PER_RESULT_CAP, _TOTAL_CAP - spent)
            if room <= 0:
                lines.append("page: [budget spent on earlier results; "
                             "fetch-text this url to read the page]")
            else:
                body = raw[:room].rstrip()
                spent += len(body)
                # The truncation notice is charged as framing, not as
                # page text, so the budget above bounds the content
                # exactly rather than to within a notice per result.
                if len(raw) > len(body):
                    body += (f"\n…[page text truncated at {len(body)} chars of "
                             f"{len(raw)}; fetch-text this url for the rest]")
                lines.append(f"page: {body}")

        blocks.append("\n".join(lines))
        sources.append({"url": url, "domain": _domain(url),
                        "title": title, "excerpt": snippet})

    header = f"{len(results)} result(s) for {query!r}"
    if missing:
        header += f" — {missing} without extracted page text"
    text = header + "\n\n" + "\n\n".join(blocks)

    return {"status": "ok", "text": text,
            "meta": [{"source_skill": "tavily",
                      "tool_metadata": {"query": query, "sources": sources}}]}
