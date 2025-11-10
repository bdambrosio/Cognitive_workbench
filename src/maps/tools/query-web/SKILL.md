---
name: query-web
type: python
trusted: true
description: Search the web using Google CSE + LLM extraction and return formatted results with relevant content and URLs. Returns a Note containing a JSON array of results.
parameters:
  source: args.query
examples:
  - '{"type":"query-web","args":{"query":"transformer architecture papers"},"out":"$results","expect":"should find recent papers on transformers"}'
---

# Web Search Tool
## Input
- Query string (e.g., "weather forecast Berkeley CA October 2025")
## Output
- JSON with query and results array. Each result has uniform structure:
{
  "query": "weather forecast Berkeley CA October 2025",
  "results": [
    {
      "text": "Station List\nNational Weather Service Marine Forecast ...",
      "format": "html",
      "metadata": {
        "source_url": "https://www.ndbc.noaa.gov/data/Forecasts/FZUS56.KMTR.html",
        "domain": "www.ndbc.noaa.gov",
        "elapsed_ms": 1206
      },
      "char_count": 112
    },
    ...
  ]
}

## Configuration
Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables.

## Examples

**Query:** "transformer architecture papers"

**Returns:**
```json
{
  "query": "transformer architecture papers",
  "results": [
    {
      "text": "The dominant sequence transduction models are based on complex recurrent or convolutional neural networks...",
      "format": "html",
      "metadata": {
        "source_url": "https://arxiv.org/abs/1706.03762",
        "domain": "arxiv.org",
        "elapsed_ms": 1234
      },
      "char_count": 523
    },
    ...
  ],
  "count": 10
}
```

## Common Workflows

### Pattern 1: Search for user consumption
When user needs direct answer, summarize results with query as focus:
```json
{"type":"query-web","args":{"query":"what are transformers in AI"},"out":"$results","expect":"should find transformers"}
{"type":"summarize","target":"$results","args":{"focus":"what are transformers"},"out":"$summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Search and form a collection from result
When results need a collection split into individual notes per URL:
```json
{"type":"query-web","args":{"query":"transformer papers"},"out":"$results","expect":"should find papers"}
{"type":"expand","target":"$results","out":"$notes_collection"}
```

### Pattern 3: Get full text from search results
When you need complete text (not just excerpts), use fetch-text:
```json
{"type":"query-web","args":{"query":"transformer papers"},"out":"$results","expect":"should find papers"}
{"type":"expand","target":"$results","out":"$items"}
{"type":"map","target":"$items","operation":"as-json","args":{"field":"metadata.source_url"},"out":"$urls"}
{"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}
```

Note: Do NOT create a collection with only the web-search result Note and try to index/search it - this returns the same single item. Use Pattern 1 (summarize) or Pattern 2 (expand) instead.

