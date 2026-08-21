---
name: search-web
description: Ask a question of a frontier model that has web search. Returns its synthesized answer, with sources when it searched — it may answer from its own knowledge and cite nothing, which is a valid result. Use for current information, or for general questions outside your own knowledge. What comes back is another model's account, not the source text; quote it as such.
args:
  query: required string — what you want to know (a question, not a search-engine query string)
---

# search-web

Search the web using an LLM with built-in web search (OpenAI GPT-5.6 Luna primary, Claude Sonnet fallback). The LLM performs searches, evaluates results, and returns a single synthesized note with source attribution.

## Query guidance

The LLM handles search strategy internally. The `query` should express *what you want to know*, not *where to search*.

Good queries:
- `"consensus on FSD 14.2.2.4 as a daily driver — wait for 14.3 or use now?"`
- `"recent developments in constitutional AI, 2025-2026"`
- `"practical differences between RLHF and DPO for language model alignment"`

Bad queries (over-specified sourcing wastes tokens and confuses the search):
- `"FSD 14.2.2.4 consensus Tesla forums Reddit r/teslamotors official release notes"`
- `"search arxiv.org and google scholar for constitutional AI papers"`

## When to use vs `fetch-text`

- `search-web` — you're looking for information and don't yet have the URL.
- `fetch-text` — you have a specific URL and want the complete content.

## Required environment

- `OPENAI_API_KEY` (primary) or `CLAUDE_API_KEY` (fallback). At least one must be set.

## Examples

```json
{"thought": "check for recent reports on FSD 14.2", "tool": "search-web", "query": "Tesla FSD 14.2.2.4 consensus daily driver wait or use 2026"}
```

```json
{"thought": "find current EU climate policy", "tool": "search-web", "query": "EU climate policy mandates 2026 recent changes"}
```
