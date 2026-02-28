---
name: fs-read
type: python
description: "Read a text, JSON, or PDF file under scenarios/<world_name>/fs and return a Note."
schema_hint: {"path": "string (relative)", "max_chars": "int", "as_json": "bool", "grobid_url": "string"}
---

# fs-read

Read a file from the filesystem sandbox and return its content as a Note. Supports text, JSON, and PDF files.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (required)
- `max_chars`: Optional maximum characters to read (text and PDF files only)
- `as_json`: Force JSON parse (default: false, text files only)
- `grobid_url`: Optional GROBID server URL for enhanced PDF parsing (from world_config)

## File Types

- **Text files**: Read as plain text
- **JSON files**: Returned as text (JSON string body)
- **PDF files**: Extracted using GROBID (if `grobid_url` provided) or pymupdf fallback. Returns extracted text body

## Output

Success returns:
- `resource_id`: Note ID containing text content.
- Note content format:
  - metadata Note linked by meta Relation (path/format/size/mtime and parser metadata)
  - body text (file content)

Example content shape:
```text
<file contents...>
```

Related metadata Note content shape:
```json
{"path":"docs/readme.md","format":"text","size":1234,"mtime":"2026-02-15T10:00:00"}
```

## Examples

```json
{"type":"fs-read","path":"notes/todo.txt","out":"$todo"}
{"type":"fs-read","path":"data/config.json","as_json":true,"out":"$config"}
{"type":"fs-read","path":"papers/article.pdf","out":"$paper"}
{"type":"fs-read","path":"papers/article.pdf","max_chars":5000,"out":"$paper_preview"}
```
