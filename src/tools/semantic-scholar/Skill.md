---
name: semantic-scholar
description: Search academic papers and journals. Returns paper text — full text when available via GROBID, otherwise abstracts. Use for scholarly literature, not general web content.
args:
  query: required string — search terms (e.g. "constitutional AI", "BERT pretraining")
  limit: optional int (default 10) — maximum results
---

# semantic-scholar

Search the Semantic Scholar academic corpus. Returns paper text concatenated into a single observation: full paper text via GROBID extraction when the PDF is available, otherwise the abstract only.

## When to use vs `search-web`

- `semantic-scholar` — scholarly literature, peer-reviewed work, papers, citations.
- `search-web` — current events, general web content, anything that isn't academic.

For a comprehensive search across both, do both — they cover different corpora.

## Examples

```json
{"thought": "find recent constitutional AI papers", "tool": "semantic-scholar", "query": "constitutional AI alignment"}
```

```json
{"thought": "load BERT-architecture papers, just the top three", "tool": "semantic-scholar", "query": "BERT bidirectional transformer pretraining", "limit": 3}
```

## Notes

- Free public API; no key required for basic search.
- Full-text extraction requires a local GROBID instance. Without GROBID, results fall back to abstracts.
