---
name: semantic-scholar
type: python
trusted: true
description: Search academic papers using Semantic Scholar API. Returns abstracts and metadata including PDF URLs when available.
parameters:
  source: args.query
examples:
  - '{"type":"semantic-scholar","args":{"query":"transformer architecture"},"out":"$papers","expect":"should find papers on transformers"}'
---

# Semantic Scholar Search Tool

## Input
- Query string (e.g., "attention mechanisms in neural networks")

## Output Structure
Returns same format as query-web for consistency:
- `results` array contains paper items (compatible with expand, map, Collection operations)
- Each item has `text` (abstract), `metadata` (title, authors, pdf_url, etc.)
- `metadata.pdf_url` may be null for paywalled papers

```json
{
  "query": "transformer architecture",
  "results": [
    {
      "text": "The dominant sequence transduction models...",
      "format": "paper",
      "metadata": {
        "title": "Attention Is All You Need",
        "authors": ["Ashish Vaswani", "Noam Shazeer", "..."],
        "year": 2017,
        "citations": 75000,
        "venue": "NeurIPS",
        "pdf_url": "https://arxiv.org/pdf/1706.03762.pdf",
        "paper_id": "...",
        "doi": "..."
      },
      "char_count": 523
    },
    ...
  ],
  "count": 10
}
```

## Configuration
Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable.

## Common Workflows

### Pattern 1: Quick summary
```json
{"type":"semantic-scholar","args":{"query":"BERT model"},"out":"$papers","expect":"should find BERT papers"}
{"type":"summarize","target":"$papers","args":{"focus":"what is BERT"},"out":"$summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Get full text from papers
```json
{"type":"semantic-scholar","args":{"query":"GPT architecture"},"out":"$papers","expect":"should find GPT papers"}
{"type":"expand","target":"$papers","out":"$items"}
{"type":"map","target":"$items","operation":"as-json","args":{"field":"metadata.pdf_url"},"out":"$urls"}
{"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}
```

### Pattern 3: Filter and analyze
```json
{"type":"semantic-scholar","args":{"query":"neural networks"},"out":"$papers","expect":"should find papers"}
{"type":"expand","target":"$papers","out":"$items"}
{"type":"filter-collection","target":"$items","args":{"predicate":"citations > 1000"},"out":"$top_papers"}
{"type":"relate","target":"$top_papers","out":"$analysis"}
```

