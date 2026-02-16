---
name: search-web
type: python
description: "Search web using Claude Sonnet with web_search tool. Returns a single text Note with synthesized findings."
---

# search-web

Search the web using Claude Sonnet's built-in web_search tool. Claude performs multiple searches with varied phrasings, reads and evaluates the results, and returns a single synthesized research note with source attribution.

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
- Metadata is stored in a separate metadata Note linked by a `meta` Relation.

Failure (`status: "failed"`):
- `reason`: Error description (e.g., `no_results`, `CLAUDE_API_KEY environment variable required`)

## Behavior

- Makes a single Claude Sonnet API call with `web_search` tool enabled (up to 10 searches)
- Claude searches with multiple phrasings, reads pages, filters for relevance, and synthesizes
- Returns one Note (not a Collection) — the synthesis is immediately usable by the agent
- Sources are preserved for citation, follow-up, or credibility assessment
- Requires `CLAUDE_API_KEY` environment variable

## Metadata Access

Tool metadata (`query`, `source_count`, `sources`, `model`, `elapsed_ms`) is linked as a metadata Note via a `meta` Relation.

Metadata Note content shape:
```json
{"query":"original search query","source_count":7,"sources":[{"url":"https://example.com/article","domain":"example.com","title":"Article Title","excerpt":"..."}],"model":"claude-sonnet-4-5-20250929","elapsed_ms":12500}
```

## Key Differences from Google CSE Version

- **Single Note, not Collection**: No need to iterate/map over results — the synthesis is ready to use
- **Pre-synthesized**: Claude has already read, evaluated, and synthesized the sources
- **No local dependencies**: No `wordfreq`, `unstructured`, GROBID, or HTML extraction needed
- **No `llm_generate` needed**: The tool calls Claude directly; does not use the workbench LLM
- **Better for opinion/forum content**: Claude can access and synthesize content that Google CSE returns but raw HTTP fetches often can't read (JavaScript-rendered pages, etc.)

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
- Use relation primitives (`related`, `find-relations`) to access metadata Notes when needed
- For complete unfiltered page content from a specific URL found in sources, use `fetch-text`
- Typical latency is 15-45 seconds (Claude performs multiple web searches internally)
- Default timeout is 120 seconds — complex queries with many searches may take 60+ seconds
- Cost is ~$0.01-0.05 per search call at current Sonnet API pricing
