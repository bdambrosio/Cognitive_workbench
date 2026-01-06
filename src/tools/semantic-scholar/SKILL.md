---
name: semantic-scholar
type: python
description: "Search academic papers. Returns Collection of JSON Notes with fields text (full paper text via GROBID when PDF available, otherwise abstract), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.uri (alias: pdf_url), metadata.venue"
---

# Semantic Scholar Search Tool

Search academic papers using Semantic Scholar API. Returns Collection of structured Notes with full paper text when PDF available.

## Purpose

Find academic papers. Returns full paper text extracted via GROBID when PDF available, otherwise returns abstract. Metadata fields are automatically enhanced from GROBID parsing when API values are missing or empty.

## Input

- `value`: Query string (e.g., "attention mechanisms in neural networks")
- `limit`: Optional result limit (int, default: 10)

## Output

Returns Collection ID containing one structured Note per paper result. Each Note contains JSON with uniform structure:
- `text`: Full paper text extracted via GROBID when PDF available, otherwise abstract
- `format`: "paper"
- `metadata.title`: Paper title
- `metadata.authors`: List of authors
- `metadata.year`: Publication year
- `metadata.citations`: Citation count
- `metadata.uri`: PDF URL (alias: `metadata.pdf_url`, may be null for paywalled papers)
- `metadata.venue`: Conference/journal name
- `metadata.paper_id`: Semantic Scholar paper ID
- `metadata.doi`: DOI if available
- `char_count`: Character count

## Behavior & Performance

- When GROBID is configured and PDF is available, `text` contains full paper content with section headers
- Otherwise, `text` contains the abstract
- Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable
- Requires `grobid_url` in YAML config (`llm_config.grobid`) for full text extraction

## Guidelines

- Use `metadata.uri` in `project` operations for consistent access across all tools
- For full text extraction, ensure GROBID is configured
- Format matches search-web and search-obsidian for consistency

## Usage Examples

Search and summarize:
```json
{"type":"semantic-scholar","value":"BERT model","out":"$papers"}
{"type":"summarize","target":"$papers","focus":"what is BERT","out":"$summary"}
```

Get full text (GROBID configured):
```json
{"type":"semantic-scholar","value":"GPT architecture","out":"$papers"}
{"type":"pluck","target":"$papers","field":"text","out":"$full_texts"}
```

Filter results:
```json
{"type":"semantic-scholar","value":"neural networks","out":"$papers"}
{"type":"filter-collection","target":"$papers","predicate":"citations > 1000","out":"$top_papers"}
```
