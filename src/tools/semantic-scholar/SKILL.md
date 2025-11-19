---
name: semantic-scholar
type: python
trusted: true
description: Search academic papers. Returns Collection of JSON Notes with fields text, metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.pdf_url, metadata.venue (Level 4 tool).
parameters:
  source: args.query
examples:
  - '{"type":"semantic-scholar","args":{"query":"transformer architecture"},"out":"$papers","expect":"should find papers on transformers"}'
  - '{"type":"project","target":"$papers","fields":["metadata.title","metadata.year"],"out":"$titles"}'
  - '{"type":"filter-structured","target":"$papers","where":"metadata.citations > 100","out":"$high_impact"}'
---

# Semantic Scholar Search Tool (Level 4)

## Input
- Query string (e.g., "attention mechanisms in neural networks")

## Output Structure
- Collection ID containing one structured Note per paper result
- Each Note contains JSON with uniform structure:
  - `text` (abstract)
  - `metadata` (title, authors, pdf_url, citations, year, venue, etc.)
  - `metadata.pdf_url` may be null for paywalled papers

```json
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
}
```

## Configuration
Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable.

## Common Workflows

### Pattern 1: Quick summary
Results are already a Collection - summarize directly (NO expand needed):
```json
{"type":"semantic-scholar","args":{"query":"BERT model"},"out":"$papers","expect":"should find BERT papers"}
{"type":"summarize","target":"$papers","args":{"focus":"what is BERT"},"out":"$summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Get full text from papers
Extract PDF URLs using project, then fetch full text:
```json
{"type":"semantic-scholar","args":{"query":"GPT architecture"},"out":"$papers","expect":"should find GPT papers"}
{"type":"project","target":"$papers","fields":["metadata.pdf_url"],"out":"$urls"}
{"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}
```

### Pattern 3: Filter and analyze
Filter Collection directly (NO expand needed):
```json
{"type":"semantic-scholar","args":{"query":"neural networks"},"out":"$papers","expect":"should find papers"}
{"type":"filter-collection","target":"$papers","args":{"predicate":"citations > 1000"},"out":"$top_papers"}
{"type":"relate","target":"$top_papers","out":"$analysis"}
```

