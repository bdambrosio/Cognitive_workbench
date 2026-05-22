---
name: format-citation
type: python
description: "Format paper citations from semantic-scholar Notes/Collections to BibTeX format"
---

# format-citation

Format academic paper citations from semantic-scholar Notes or Collections to BibTeX citation format.

## Input

- `target`: Collection ID, Note ID, or variable containing paper metadata
- `format`: Citation format (default: "bibtex"). Currently supports: "bibtex"

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID (if input was Collection) or Note ID (if input was Note) containing formatted BibTeX citations
- Each output Note contains:
  - `text`: Formatted BibTeX citation string
  - `format`: "citation"
  - `metadata`: Original metadata preserved (if available)

## Behavior

- Processes all Notes in Collection, continuing even if individual Notes fail
- Extracts paper metadata (title, authors, year, venue, DOI) from Note content
- Formats as BibTeX entry with appropriate entry type (@article, @inproceedings, etc.)
- Creates new Notes with formatted citations, preserving original Notes
- Handles missing fields gracefully (omits unavailable fields)

## Examples

```json
{"type":"semantic-scholar","value":"transformer architecture","out":"$papers"}
{"type":"format-citation","target":"$papers","format":"bibtex","out":"$bibtex"}
```
