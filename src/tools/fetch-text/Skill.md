---
name: fetch-text
description: Fetch the text content of a web page or PDF from a URL. Handles HTML, PDF, Markdown, plain text — auto-detects format. Research PDFs come back as a section index you can then read section by section.
args:
  url: required string — the URL or base64-encoded PDF to fetch
  section: optional string — for a research PDF, return just this section's text. Use a name exactly as it appeared in the section index from the first call.
  render_js: optional boolean — render the page in a headless browser before extracting text. Use for a JS-gated page that returns a shell of navigation and "Loading…" rather than content. The automatic browser fallback only fires when static extraction yields under 100 characters, so a few hundred characters of boilerplate never trips it. Slower than a plain fetch, so reach for it when a normal call came back with chrome and no substance, or for a portal already known to be gated. It errors rather than falling back silently when the browser is unavailable or renders nothing.
---

# fetch-text

Retrieve and extract the text of a URL. Auto-detects the format (HTML /
PDF / Markdown / plain text).

## Behavior

- **Research PDF** (parsed into sections): the first call returns the
  title, the abstract, and a **section index** — every section name with
  its size — rather than the body text. Call again with the same `url`
  plus `section` to read one section. A long paper runs to tens of
  thousands of words; the index lets you read the two sections that
  answer the question instead of flooding the conversation with the rest.
- **Other PDFs**: page-by-page text extraction, truncated at 8000 chars.
- HTML: structured text with links and image references preserved.
- Markdown / plain: returned as-is.
- Page candidate images (Open Graph, Twitter cards, in-page `<img>` tags)
  are surfaced as part of the result so a downstream `display` can pick a
  primary image without re-fetching.

## Reading a paper

1. `{"tool": "fetch-text", "url": "<pdf url>"}` → title, abstract, section index.
2. Pick the sections that bear on the question — not all of them.
3. `{"tool": "fetch-text", "url": "<same url>", "section": "Results"}` → that section.

Asking for a section that isn't in the index returns an error listing the
names that are, so re-read the index rather than guessing. Section names
match case-insensitively, and an unambiguous prefix works.

For a paper's **reference list**, use `semantic-scholar` with a
`paper_id` — the citation graph gives resolved records, which beats
parsing a bibliography out of the PDF.

## When to use vs search-web

- `fetch-text` — you already have a specific URL and want its content.
- `search-web` — you're looking for information and don't yet have the URL.

## Examples

```json
{"thought": "see what's in this paper", "tool": "fetch-text", "url": "https://arxiv.org/pdf/1706.03762"}
```

```json
{"thought": "the index listed a Results section — read it", "tool": "fetch-text", "url": "https://arxiv.org/pdf/1706.03762", "section": "Results"}
```

```json
{"thought": "load today's weather forecast page", "tool": "fetch-text", "url": "https://forecast.weather.gov/MapClick.php?lat=37.87&lon=-122.27"}
```

## Notes

- Section support needs a reachable GROBID server. `GROBID_URL` overrides
  the default `http://localhost:8070/api/processFulltextDocument`; set it
  to an empty string to disable. With GROBID unavailable, PDFs fall back
  to flat page text and the `section` arg has nothing to resolve against.
