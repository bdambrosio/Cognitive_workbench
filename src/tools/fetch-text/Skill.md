---
name: fetch-text
type: python
description: "Fetch all text from URL or base64 PDF. Collection-aware (extracts first item if given Collection). Auto-detects format (PDF/HTML/MD/TXT) and extracts complete text content"
---

# fetch-text

Fetch complete text content from URLs or PDFs. Auto-detects format and extracts all text.

## Input

- `target`: URL string, base64-encoded PDF, Note ID, or Collection ID (uses first item's `content` as URL)

## Output

Success (`status: "success"`):
- `value`: JSON string with:
  - `text`: Full extracted text
  - `format`: `"pdf"` | `"html"` | `"markdown"` | `"text"`
  - `metadata`: Source URL and format-specific metadata
  - `page_count`: Number of pages (PDF only)
  - `char_count`: Total character count

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- Auto-detects format from content
- Extracts complete text without filtering
- For Collections: extracts first Note's content field as URL

## Planning Notes

- Use when you have a specific URL and want complete content
- Use `search-web` when searching for information (returns filtered excerpts)
- For structured search results, extract URLs first with `project`

## Examples

```json
{"type":"fetch-text","target":"https://arxiv.org/pdf/1706.03762.pdf","out":"$paper_text"}
{"type":"project","target":"$papers","fields":["metadata.uri"],"out":"$urls"}
{"type":"fetch-text","target":"$urls","out":"$paper_text"}
```
