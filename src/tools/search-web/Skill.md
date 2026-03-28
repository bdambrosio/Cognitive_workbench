---
name: search-web
type: python
description: "Search web using LLM with web_search tool. Returns a single text Note with synthesized findings."
---

# search-web

Search the web using an LLM with built-in web search (OpenAI GPT-5.4-mini primary, Claude Sonnet fallback). The LLM performs searches, reads and evaluates the results, and returns a single synthesized research note with source attribution.

## Input

- `query`: Query string — the *question* you want answered, not a search engine query string

## Query Guidance

**Critical:** Claude handles the search strategy internally — it decides what to search for, rephrases queries, and evaluates sources. The `query` should express *what you want to know*, not *where to search*.

Good queries:
- `"consensus on FSD 14.2.2.4 as a daily driver — wait for 14.3 or use now?"`
- `"recent developments in constitutional AI, 2025-2026"`
- `"practical differences between RLHF and DPO for language model alignment"`

Bad queries (over-specified sourcing instructions that waste tokens and confuse the search):
- `"FSD 14.2.2.4 consensus Tesla forums Reddit r/teslamotors official release notes user reviews"` 
- `"search arxiv.org and google scholar for constitutional AI papers"`

Let Claude decide where to look. Tell it *what* you need to know.

## Output

Success (`status: "success"`):
- `resource_id`: Note ID containing text content.
- Note content is synthesized body text (primary output).
- Metadata is stored transparently and accessible via `get-metadata`.

Failure (`status: "failed"`):
- `reason`: Error description (e.g., `no_results`, `CLAUDE_API_KEY environment variable required`)

## Behavior

- Makes a single LLM API call with `web_search` tool enabled
- Primary: OpenAI GPT-5.4-mini via Responses API (cheaper, location-aware for Berkeley, CA)
- Fallback: Claude Sonnet if OPENAI_API_KEY is not set or call fails
- Returns one Note (not a Collection) — the synthesis is immediately usable by the agent
- Sources are preserved for citation, follow-up, or credibility assessment
- Requires `OPENAI_API_KEY` or `CLAUDE_API_KEY` environment variable

## Metadata Access

Tool metadata (`query`, `source_count`, `sources`, `model`, `elapsed_ms`) is accessible via `get-metadata`.

Metadata Note content shape:
```json
{"query":"original search query","source_count":7,"sources":[{"url":"https://example.com/article","domain":"example.com","title":"Article Title","excerpt":"..."}],"model":"gpt-5.4-mini","elapsed_ms":5200}
```

## Common Workflows

**Direct use (most common — synthesis is already done):**
```json
{"type":"search-web","query":"transformer architecture recent improvements 2026","out":"$findings"}
{"type":"extract","target":"$findings","instruction":"List the key architectural improvements mentioned","out":"$improvements"}
```

**Use synthesis directly in a larger analysis:**
```json
{"type":"search-web","query":"climate policy EU 2026","out":"$eu_policy"}
{"type":"search-web","query":"climate policy US 2026","out":"$us_policy"}
{"type":"synthesize","target":["$eu_policy","$us_policy"],"focus":"compare EU and US approaches","out":"$comparison"}
```

**Access source URLs for follow-up:**
```json
{"type":"search-web","query":"recent papers on RLHF alternatives","out":"$search"}
{"type":"pluck","target":"$search","field":"metadata.sources","out":"$source_list"}
```

**Combine with academic search:**
```json
{"type":"search-web","query":"practical applications of constitutional AI","out":"$web_findings"}
{"type":"semantic-scholar","query":"constitutional AI","out":"$papers"}
{"type":"synthesize","target":["$web_findings","$papers"],"focus":"bridge between research and practice","out":"$report"}
```

## Planning Notes

- The body text is the primary output — substantive and actionable, not a stub
- Use `extract` on the Note to pull out specific aspects of the synthesis text
- Use `get-metadata` to access tool metadata (query, sources, model, elapsed_ms)
- For complete unfiltered page content from a specific URL found in sources, use `fetch-text`
