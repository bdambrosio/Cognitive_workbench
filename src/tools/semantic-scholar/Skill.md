---
name: semantic-scholar
type: python
description: "Search academic papers. Returns Collection of JSON Notes with fields text (full paper text via GROBID when PDF available, otherwise abstract), metadata.title, metadata.authors, metadata.year, metadata.citations, metadata.uri (alias: pdf_url), metadata.venue"
---

# semantic-scholar

Search academic papers using Semantic Scholar API. Returns Collection of structured Notes with full paper text when PDF available.

## Input

- `query`: Query string (e.g., "attention mechanisms in neural networks")
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

## Content Structure

Each Note in the returned Collection has the following JSON structure:
```json
{
  "text": "Full paper text or abstract...",
  "format": "paper",
  "metadata": {
    "title": "Paper Title",
    "authors": ["Author 1", "Author 2"],
    "year": 2023,
    "citations": 150,
    "uri": "https://example.com/paper.pdf",
    "venue": "NeurIPS"
  },
  "char_count": 5000
}
```

**Important:** All result data is in the Note's `content` field (a dict). Engine metadata (creation date, source tool, etc.) is separate and accessed via `get_resource_metadata()`, not via `content['metadata']`.

## Field Access Examples

**Extract URLs for fetching:**
```json
{"type":"semantic-scholar","query":"BERT model","out":"$papers"}
{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}
{"type":"pluck","target":"$urls","field":"metadata.uri","out":"$url_list"}
```

**Extract paper metadata:**
```json
{"type":"project","target":"$papers","fields":["metadata.title","metadata.year","metadata.citations"],"out":"$paper_info"}
```

**Filter by year:**
```json
{"type":"filter-structured","target":"$papers","where":"metadata.year > 2020","out":"$recent_papers"}
```

## Examples

```json
{"type":"semantic-scholar","query":"BERT model","out":"$papers"}
{"type":"summarize","target":"$papers","focus":"what is BERT","out":"$summary"}
```
