---
name: fetch-text
description: Fetch all text from URL or base64 PDF. Auto-detects format (PDF/HTML/MD/TXT) and extracts complete text content.
type: python
trusted: true
parameters:
  - name: url_or_content
    type: string
    description: URL string or base64-encoded PDF content
examples:
  - '{"type":"fetch-text","target":"https://example.com/doc.pdf","out":"$text"}'
  - '{"type":"fetch-text","target":"https://example.com/page.html","out":"$text"}'
---

# Fetch Text

Fetches complete text content from URLs or base64 PDFs. Auto-detects format (PDF/HTML/Markdown/Text) and extracts all text without filtering.

## Input
- `url_or_content`: URL string pointing to any text-based content, or base64-encoded PDF content (backward compatibility)

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

## Differences from query-web
- **fetch-text**: Gets ALL text from a single URL (no filtering, no search)
- **query-web**: Searches web, returns FILTERED excerpts from multiple URLs (query-relevant snippets)

Use fetch-text when you have a specific URL and want complete content. Use query-web when searching for information.

