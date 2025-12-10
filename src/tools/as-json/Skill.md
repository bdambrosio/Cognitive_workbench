---
name: as-json
description: Identify and extract JSON from mixed content, optionally extracting specific fields
type: prompt_augmentation

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
- **all** - If true, returns all matching fields, one per line; if false/omitted, returns first match (default: false)

## Output Format

- **No field**: Full parsed JSON as formatted text
- **With field (all=false)**: First matching field value as text
- **With field (all=true)**: all matching field values, one per line
- **Field not found**: note-null
- **JSON not identified**: FAIL (hard error)

## Usage Examples

**Extract JSON from LLM response:**
{"type":"as-json","target":"$llm_response","out":"$parsed"}

**Extract first matching field:**
{"type":"as-json","target":"$response","args":{"field":"title"},"out":"$title"}

**Extract all matching fields:**
{"type":"as-json","target":"$response","args":{"field":"url","all":true},"out":"$all_urls"}

## Guidelines

- Strips leading/trailing text and code fences automatically
- Field search is recursive - finds field at any depth
- Field not found returns note-null (soft failure, plan continues)
- No valid JSON found triggers FAIL (hard failure, plan should handle)
- Preserves data types in extracted values
- Default extracts first match; use `all: true` for all matches

## Examples

**Extract first match:**
Input: `{"results": [{"url": "https://example.com/1"}, {"url": "https://example.com/2"}]}`  
field: "url"  
Output: `https://example.com/1`

**Extract all matches:**
Input: `{"results": [{"url": "https://example.com/1"}, {"url": "https://example.com/2"}]}`  
field: "url", all: true  
Output: `https://example.com/1\nhttps://example.com/2`

**Strip code fence:**
Input: `Here's the data:\n\`\`\`json\n{"count": 42}\n\`\`\``  
field: "count"  
Output: `42`

**Missing field:**
Input: `{"name": "Alice"}`  
field: "email"  
Output: `note-null`

Output ONLY the selected information as shown above, no introductory, explanatory, reasoning, code fences, etc.
