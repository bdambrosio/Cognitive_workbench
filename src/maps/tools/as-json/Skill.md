---
name: as-json
description: Identify and extract JSON from mixed content, optionally extracting specific fields
type: prompt_augmentation
parameters: field (optional) - dot notation path to extract (e.g., "results[0].title")
examples:
  - '{"type":"as-json","target":"$llm_response","args":{"field":"data.title"},"out":"$title"}'
---

# As JSON

Interpret note content as JSON, automatically stripping surrounding text, code fences, and other noise. Optionally extract specific fields.

## Purpose

- Extract JSON from LLM responses with preambles or explanations
- Strip markdown code fences (```json ... ```)
- Handle mixed content with embedded JSON
- Extract specific field values
- Validate JSON structure

## Input Format

Accepts a Note containing JSON (clean or embedded):
- Clean JSON: `{"key": "value"}`
- Code fenced: ` ```json\n{"key": "value"}\n``` `
- With preamble: `Here's the data:\n{"key": "value"}`
- With trailing text: `{"key": "value"}\nAs you can see...`

## Parameters

**Optional:**
- **field** - Field name to extract (searches recursively, e.g., "url", "title", "email")
- **all** - If true, returns array of all matching fields; if false/omitted, returns first match (default: false)

## Output Format

- **No field**: Full parsed JSON as formatted text
- **With field (all=false)**: First matching field value as text
- **With field (all=true)**: Array of all matching field values
- **Field not found**: note-null
- **JSON not identified**: FAIL (hard error)

## Usage Examples

**Extract JSON from LLM response:**
```json
{"type":"as-json","target":"$llm_response","out":"$parsed"}
```

**Extract first matching field:**
```json
{"type":"as-json","target":"$response","args":{"field":"title"},"out":"$title"}
```

**Extract all matching fields:**
```json
{"type":"as-json","target":"$response","args":{"field":"url","all":true},"out":"$all_urls"}
```

## Guidelines

- Strips leading/trailing text and code fences automatically
- Field search is recursive - finds field at any depth
- Field not found returns note-null (soft failure, plan continues)
- No valid JSON found triggers FAIL (hard failure, plan should handle)
- Preserves data types in extracted values
- Default extracts first match; use `all: true` for all matches

## Examples

**Example 1: Extract first match**

Input:
```json
{"results": [
  {"url": "https://example.com/1", "title": "First"},
  {"url": "https://example.com/2", "title": "Second"}
]}
```

Field: "url"

Output: `https://example.com/1`

**Example 2: Extract all matches**

Input:
```json
{"results": [
  {"url": "https://example.com/1", "title": "First"},
  {"url": "https://example.com/2", "title": "Second"}
]}
```

Field: "url", all: true

Output: `["https://example.com/1", "https://example.com/2"]`

**Example 3: Strip code fence**

Input:
```
Here's the data:
```json
{"status": "success", "count": 42}
```
```

Field: "count"

Output: `42`

**Example 4: Missing field returns null**

Input: `{"name": "Alice"}`

Field: "email"

Output: note-null

