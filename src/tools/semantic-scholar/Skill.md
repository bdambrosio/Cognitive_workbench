---
name: semantic-scholar
description: Search academic papers, or list one paper's references. Returns abstracts plus metadata (including a pdf_url) — use fetch-text on that pdf_url to read the paper itself. Use for scholarly literature, not general web content.
args:
  query: required string (unless paper_id given) — search terms (e.g. "constitutional AI", "BERT pretraining")
  paper_id: optional string — return this paper's reference list instead of searching. Accepts an S2 paperId, "arXiv:1706.03762", or "DOI:10.1145/3442188"
  limit: optional int — max results for a search (default 10), or max references returned (default 40)
---

# semantic-scholar

Search the Semantic Scholar academic corpus, or pull the structured
reference list of a single paper.

## Two modes

- **Search** (`query`) — returns each paper's **abstract** plus metadata:
  title, authors, year, venue, citation count, DOI, `paper_id`, and
  `pdf_url`. Abstracts only; this call does not read PDFs.
- **References** (`paper_id`) — returns that paper's reference list as
  resolved records: title, year, authors, venue, and an arXiv id or DOI
  where one exists. These come from the citation graph, so each entry is
  a real paper you can look up directly rather than a string scraped
  from a bibliography.

## Reading the actual paper

Search gives you a `pdf_url`. Pass it to `fetch-text` to read the paper.
For a research PDF, `fetch-text` returns a section index first, so you
can then request the one or two sections you actually need instead of
pulling tens of thousands of words into context.

## When to use vs `search-web`

- `semantic-scholar` — scholarly literature, peer-reviewed work, papers, citations.
- `search-web` — current events, general web content, anything that isn't academic.

For a comprehensive search across both, do both — they cover different corpora.

## Examples

```json
{"thought": "find recent constitutional AI papers", "tool": "semantic-scholar", "query": "constitutional AI alignment"}
```

```json
{"thought": "what does the transformer paper build on?", "tool": "semantic-scholar", "paper_id": "arXiv:1706.03762"}
```

```json
{"thought": "load BERT-architecture papers, just the top three", "tool": "semantic-scholar", "query": "BERT bidirectional transformer pretraining", "limit": 3}
```

## Notes

- Free public API; no key required. Set `SEMANTIC_SCHOLAR_API_KEY` for
  higher rate limits — unauthenticated use throttles quickly (429), and
  a throttled call reports itself as unavailable rather than as
  "no results".
- Papers with neither an abstract nor a PDF are dropped from search results.
- A paper S2 has no record of has no references here; extract them from
  the PDF itself in that case.
