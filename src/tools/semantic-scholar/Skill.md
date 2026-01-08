---
name: semantic-scholar
type: python
description: "Search academic papers. Returns Collection of JSON Notes with fields text (full paper text via GROBID when PDF available, otherwise abstract), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.uri (alias: pdf_url), metadata.venue"
---

# semantic-scholar

Search academic papers using Semantic Scholar API. Returns Collection of structured Notes with full paper text when PDF available.

## Input

- `value`: Query string (e.g., "attention mechanisms in neural networks")
- `limit`: Optional result limit (int, default: 10)

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing structured Notes, each with:
  - `text`: Full paper text (via GROBID) or abstract
  - `format`: "paper"
  - `metadata.title`: Paper title
  - `metadata.authors`: List of authors
  - `metadata.year`: Publication year
  - `metadata.citations`: Citation count
  - `metadata.uri`: PDF URL (may be null for paywalled papers)
  - `metadata.venue`: Conference/journal name
  - `char_count`: Character count

## Behavior

- When GROBID configured and PDF available, `text` contains full paper content
- Otherwise `text` contains the abstract
- Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable
- Requires `grobid_url` in YAML config for full text extraction

## Examples

```json
{"type":"semantic-scholar","value":"BERT model","out":"$papers"}
{"type":"summarize","target":"$papers","focus":"what is BERT","out":"$summary"}
```
