---
name: extract-struct
description: Extract structured metadata (title, authors, year) from paper text using LLM. Use to convert online search results metadata to JSON
type: prompt_augmentation
parameters:
  - name: paper_text
    type: string
    description: Full text or first pages of academic paper
examples:
  - '{"type":"extract-struct","target":"$paper_text","out":"$metadata"}'
---

# Extract Metadata

Extracts structured metadata from academic paper text using LLM analysis.

## Input
- `paper_text`: Full text or first few pages of paper (from fetch-text, or text field from fetch-text JSON output)

## Output
Returns JSON with:
- title: Paper title
- authors: List of author names
- year: Publication year
- venue: Conference/journal if identifiable
- abstract: Paper abstract if present

## Prompt

You are analyzing academic paper text. Extract the following metadata and return as JSON:

```json
{
  "title": "paper title",
  "authors": ["author1", "author2"],
  "year": "YYYY",
  "venue": "conference or journal name",
  "abstract": "paper abstract"
}
```

Paper text:
{{paper_text}}

Return only the JSON, no explanation.

