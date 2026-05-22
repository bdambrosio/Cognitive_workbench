---
name: fetch-text
description: Fetch the text content of a web page or PDF from a URL. Handles HTML, PDF, Markdown, plain text — auto-detects format.
args:
  url: required string — the URL or base64-encoded PDF to fetch
---

# fetch-text

Retrieve and extract the full text of a URL. Auto-detects the format (HTML / PDF / Markdown / plain text) and returns the extracted text.

## Behavior

- HTML: structured text with links and image references preserved.
- PDF: text extraction page by page.
- Markdown / plain: returned as-is.
- Page candidate images (Open Graph, Twitter cards, in-page `<img>` tags) are surfaced as part of the result so a downstream `display` can pick a primary image without re-fetching.

## When to use vs search-web

- `fetch-text` — you already have a specific URL and want the complete content.
- `search-web` — you're looking for information and don't yet have the URL.

## Examples

```json
{"thought": "read the paper from the URL", "tool": "fetch-text", "url": "https://arxiv.org/pdf/1706.03762.pdf"}
```

```json
{"thought": "load today's weather forecast page", "tool": "fetch-text", "url": "https://forecast.weather.gov/MapClick.php?lat=37.87&lon=-122.27"}
```
