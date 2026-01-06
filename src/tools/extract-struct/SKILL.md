---
name: extract-struct
type: prompt_augmentation
description: "Extract structured metadata (title, authors, year) from paper text using LLM. Use to convert online search results metadata to JSON"
---

# Extract Metadata Tool

Extracts structured metadata from academic paper text using LLM analysis.

## Purpose

Convert unstructured paper text into structured JSON format with title, authors, year, venue, and abstract fields. Useful for converting online search results to standardized metadata format.

## Input

- `target`: Note ID or variable containing full text or first pages of academic paper

## Output

Returns JSON Note with:
- `title`: Paper title
- `authors`: List of author names
- `year`: Publication year
- `venue`: Conference/journal if identifiable
- `abstract`: Paper abstract if present

## Behavior & Performance

- Uses LLM to analyze paper text and extract structured fields
- Handles various paper formats and layouts
- Returns only JSON, no explanation text

## Guidelines

- Provide full text or first few pages for best results
- Works best with academic papers that have clear title/author sections
- Use with `fetch-text` to get paper content first

## Usage Examples

Extract metadata from paper:
```json
{"type":"extract-struct","target":"$paper_text","out":"$metadata"}
```
