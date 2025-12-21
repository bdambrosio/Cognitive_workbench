---
name: fetch-text
description: Fetch all text from URL or base64 PDF. Collection-aware (extracts first item if given Collection). Auto-detects format (PDF/HTML/MD/TXT) and extracts complete text content.
type: python
schema_hint:
  target: "$variable or Note ID or Collection ID or URL string"
  out: "$variable"
parameters:
  - name: url_or_content
    type: string
    description: URL string, base64-encoded PDF content, or Collection ID (uses first item's content as URL)
examples:
  - '{"type":"fetch-text","target":"https://example.com/doc.pdf","out":"$text"}'
  - '{"type":"fetch-text","target":"https://example.com/page.html","out":"$text"}'
  - '{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}'
  - '{"type":"fetch-text","target":"$urls","out":"$text"}'
---

# Fetch Text

Fetches complete text content from URLs or base64 PDFs. Auto-detects format (PDF/HTML/Markdown/Text) and extracts all text without filtering.

**Collection-aware**: If given a Collection ID, extracts the first Note's `content` field and uses it as the URL. **IMPORTANT**: The Collection must contain Notes where `content` is a URL string. For structured Notes (e.g., from `semantic-scholar` or `search-web`), use `project` first to extract URLs into a Collection of URL strings.

## Input
- `url_or_content`: URL string, base64-encoded PDF content, or Collection ID (uses first item's `content` field as URL)

## Output
Returns structured JSON with:
- `text`: Full extracted text content (all text, no filtering)
- `format`: Detected format ("pdf", "html", "markdown", "text")
- `metadata`: Format-specific metadata (source_url, pdf_metadata for PDFs, html_metadata for HTML)
- `page_count`: Number of pages (PDF only)
- `char_count`: Total character count

## Usage
Direct URL fetch:
```json
{"type":"fetch-text","target":"https://arxiv.org/pdf/1706.03762.pdf","out":"$paper_text"}
```

HTML page:
```json
{"type":"fetch-text","target":"https://example.com/article.html","out":"$article_text"}
```

Collection input (extract URLs first):
```json
{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}
{"type":"fetch-text","target":"$urls","out":"$paper_text"}
```

## Differences from search-web
- **fetch-text**: Gets ALL text from a single URL (no filtering, no search)
- **search-web**: Searches web, returns FILTERED excerpts from multiple URLs (query-relevant snippets)

Use fetch-text when you have a specific URL and want complete content. Use search-web when searching for information.

