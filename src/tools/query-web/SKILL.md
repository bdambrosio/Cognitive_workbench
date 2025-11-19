---
name: query-web
type: python
trusted: true
description: Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.source_url, metadata.domain, format, char_count (Level 4 tool).
parameters:
  source: args.query
examples:
  - '{"type":"query-web","args":{"query":"transformer architecture papers"},"out":"$results","expect":"should find recent papers on transformers"}'
  - '{"type":"project","target":"$results","fields":["metadata.source_url","metadata.domain"],"out":"$urls"}'
  - '{"type":"filter-structured","target":"$results","where":"char_count > 1000","out":"$long_articles"}'
---

# Web Search Tool (Level 4)
## Input
- Query string (e.g., "weather forecast Berkeley CA October 2025")
## Output
- Collection ID containing one structured Note per search result
- Each Note contains JSON with uniform structure:
{
  "text": "Station List\nNational Weather Service Marine Forecast ...",
  "format": "html",
  "metadata": {
    "source_url": "https://www.ndbc.noaa.gov/data/Forecasts/FZUS56.KMTR.html",
    "domain": "www.ndbc.noaa.gov",
    "elapsed_ms": 1206
  },
  "char_count": 112
}

## Configuration
Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables.

## Example Note Structure

Each Note in the returned Collection contains:
```json
{
  "text": "The dominant sequence transduction models are based on...",
  "format": "html",
  "metadata": {
    "source_url": "https://arxiv.org/abs/1706.03762",
    "domain": "arxiv.org",
    "elapsed_ms": 1234
  },
  "char_count": 523
}
```

## Common Workflows

### Pattern 1: Search and summarize for user
When user needs direct answer, summarize Collection directly:
```json
{"type":"query-web","args":{"query":"what are transformers in AI"},"out":"$results","expect":"should find transformers"}
{"type":"summarize","target":"$results","args":{"focus":"what are transformers"},"out":"$summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Search and process Collection
Results are already a Collection - use map/filter directly (NO expand needed):
```json
{"type":"query-web","args":{"query":"transformer papers"},"out":"$results","expect":"should find papers"}
{"type":"filter-collection","target":"$results","args":{"predicate":"contains arxiv.org"},"out":"$arxiv_only"}
```

### Pattern 3: Get full text from search results
Extract URLs using project, then fetch full text:
```json
{"type":"query-web","args":{"query":"transformer papers"},"out":"$results","expect":"should find papers"}
{"type":"project","target":"$results","fields":["metadata.source_url"],"out":"$urls"}
{"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}
```

