---
name: semantic-scholar
type: python
trusted: true
description: "Search academic papers. Returns Collection of JSON Notes with fields text (full paper text via GROBID when PDF available, otherwise abstract), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.uri (alias: pdf_url), metadata.venue (Level 4 tool)."
examples:
  - '{"type":"semantic-scholar","value":"transformer architecture","out":"$papers"}'
  - '{"type":"project","target":"$papers","fields":["metadata.title","metadata.year"],"out":"$titles"}'
  - '{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}'
  - '{"type":"filter-structured","target":"$papers","where":"metadata.citations > 100","out":"$high_impact"}'
---

# Semantic Scholar Search Tool (Level 4)

## Input
- Query string (e.g., "attention mechanisms in neural networks")

## Output Structure
- Collection ID containing one structured Note per paper result
- Each Note contains JSON with uniform structure:
  - `text` (full paper text extracted via GROBID when PDF available, otherwise abstract)
  - `metadata` (title, authors, uri/pdf_url, citations, year, venue, etc.)
  - `metadata.uri` (alias: `metadata.pdf_url`) may be null for paywalled papers
- When GROBID is configured and PDF is available, `text` contains full paper content with section headers. Otherwise, `text` contains the abstract.
- Metadata fields (`title`, `authors`) are automatically enhanced from GROBID parsing when API values are missing or empty.

```json
{
  "text": "Introduction\nThis paper presents a novel approach...\n\nMethods\nWe propose a transformer architecture...\n\nResults\nOur experiments demonstrate...",
  "format": "paper",
  "metadata": {
    "title": "Attention Is All You Need",
    "authors": ["Ashish Vaswani", "Noam Shazeer", "..."],
    "year": 2017,
    "citations": 75000,
    "venue": "NeurIPS",
    "pdf_url": "https://arxiv.org/pdf/1706.03762.pdf",
    "uri": "https://arxiv.org/pdf/1706.03762.pdf",
    "paper_id": "...",
    "doi": "..."
  },
  "char_count": 45230
}
```

## Configuration
- Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable.
- Requires `grobid_url` in YAML config (`llm_config.grobid`) for full text extraction. When GROBID is configured, papers with open PDF URLs are automatically parsed to extract full text and enhance metadata.

## Common Workflows

### Pattern 1: Quick summary
Results are already a Collection - summarize directly (NO split needed):
```json
{"type":"semantic-scholar","value":"BERT model","out":"$papers"}
{"type":"summarize","target":"$papers","focus":"what is BERT","out":"$summary"}
{"type":"say","target":"user","value":"$summary"}
```

### Pattern 2: Get full text from papers
With GROBID configured, full text is already in `text` field. No need for fetch-text:
```json
{"type":"semantic-scholar","value":"GPT architecture","out":"$papers"}
{"type":"pluck","target":"$papers","field":"text","out":"$full_texts"}
```
Note: If GROBID is not configured, extract PDF URLs and use fetch-text:
```json
{"type":"semantic-scholar","value":"GPT architecture","out":"$papers"}
{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}
{"type":"map","target":"$urls","operation":"fetch-text","out":"$full_texts"}
```

### Pattern 3: Filter and analyze
Filter Collection directly (NO split needed):
```json
{"type":"semantic-scholar","value":"neural networks","out":"$papers"}
{"type":"filter-collection","target":"$papers","predicate":"citations > 1000","out":"$top_papers"}
{"type":"relate","target":"$top_papers","out":"$analysis"}
```

