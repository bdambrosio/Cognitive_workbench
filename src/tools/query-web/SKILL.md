---
name: query-web
type: python
trusted: true
description: "Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count (Level 4 tool)."
schema_hint:
  value: "string (query)"
  out: "$variable"
examples:
  - '{"type":"query-web","value":"transformer architecture papers","out":"$results"}'
  - '{"type":"project","target":"$results","fields":["metadata.uri","metadata.domain"],"out":"$urls"}'
  - '{"type":"filter-structured","target":"$results","where":"char_count > 1000","out":"$long_articles"}'
---

# Web Search Tool (Level 4)
## Input
- Query string (e.g., "weather forecast Berkeley CA October 2025")
## Output
- Collection ID containing one structured Note per search result
- Each Note contains JSON with uniform structure:

```json
{
  "text": "Station List\nNational Weather Service Marine Forecast ...",
  "format": "html",
  "metadata": {
    "source_url": "https://www.ndbc.noaa.gov/data/Forecasts/FZUS56.KMTR.html",
    "uri": "https://www.ndbc.noaa.gov/data/Forecasts/FZUS56.KMTR.html",
    "domain": "www.ndbc.noaa.gov",
    "elapsed_ms": 1206
  },
  "char_count": 112
}
```

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
    "uri": "https://arxiv.org/abs/1706.03762",
    "domain": "arxiv.org",
    "elapsed_ms": 1234
  },
  "char_count": 523
}
```

**Note**: The `uri` field is a standardized URI field (alias for `source_url`) for consistency with `semantic-scholar` and search primitives. Use `metadata.uri` in `project` operations for consistent access across all tools.

## Common Workflows

**Search and summarize:**
```json
{"type":"query-web","value":"what are transformers in AI","out":"$results"}
{"type":"summarize","target":"$results","focus":"what are transformers","out":"$summary"}
```

**Filter results:**
```json
{"type":"query-web","value":"transformer papers","out":"$results"}
{"type":"filter-collection","target":"$results","predicate":"contains arxiv.org","out":"$arxiv_only"}
```

