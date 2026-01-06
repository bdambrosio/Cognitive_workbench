---
name: fetch-text
type: python
description: "Fetch all text from URL or base64 PDF. Collection-aware (extracts first item if given Collection). Auto-detects format (PDF/HTML/MD/TXT) and extracts complete text content"
---

# Fetch Text Tool

Fetches complete text content from URLs or base64 PDFs. Auto-detects format (PDF/HTML/Markdown/Text) and extracts all text without filtering.

## Purpose

Get complete text content from a specific URL or PDF. Collection-aware: if given a Collection ID, extracts the first Note's `content` field and uses it as the URL.

## Input

- `target`: URL string, base64-encoded PDF content, Note ID, or Collection ID (uses first item's `content` field as URL)

## Output

Returns structured JSON Note with:
- `text`: Full extracted text content (all text, no filtering)
- `format`: Detected format ("pdf", "html", "markdown", "text")
- `metadata`: Format-specific metadata (source_url, pdf_metadata for PDFs, html_metadata for HTML)
- `page_count`: Number of pages (PDF only)
- `char_count`: Total character count

## Behavior & Performance

- Auto-detects format from content
- Extracts complete text without filtering
- For Collections: extracts first Note's content field as URL
- IMPORTANT: Collection must contain Notes where `content` is a URL string. For structured Notes (e.g., from `semantic-scholar` or `search-web`), use `project` first to extract URLs into a Collection of URL strings

## Guidelines

- Use when you have a specific URL and want complete content
- Use `search-web` when searching for information (returns filtered excerpts from multiple URLs)
- For structured search results, extract URLs first: `{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}` then `{"type":"fetch-text","target":"$urls","out":"$paper_text"}`

## Usage Examples

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
