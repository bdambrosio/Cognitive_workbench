---
name: parse-markdown
description: Extract headers, lists, links, code blocks, and structure from markdown
type: prompt_augmentation
parameters: element (optional) - type to extract (headers|lists|links|code|all)
---

# Parse Markdown

Extract structural elements from markdown-formatted text.

## Purpose

- Extract headers and document structure
- Pull out lists and list items
- Find all links and URLs
- Extract code blocks
- Enable structure-based processing

## Input Format

Accepts a Note containing markdown text.

## Parameters

**Optional:**
- **element** - Element type to extract: "headers", "lists", "links", "code", or "all" (default)

## Output Format

Returns a Note containing extracted elements:
- **headers**: List of headers with levels
- **lists**: All list items (ordered and unordered)
- **links**: URLs and link text
- **code**: Code blocks with language tags
- **all**: Structured breakdown of all elements

## Usage Examples

**Extract all structure:**
```json
{"type":"parse-markdown","value":"$document","out":"structure"}
```

**Extract only headers:**
```json
{"type":"parse-markdown","value":"$document","args":{"element":"headers"},"out":"toc"}
```

**Extract links:**
```json
{"type":"parse-markdown","value":"$page","args":{"element":"links"},"out":"all_links"}
```

## Guidelines

- Handles standard markdown syntax
- Preserves nesting for lists
- Returns empty if no matching elements found
- Code blocks include language identifier if present

## Example

**Input Note:**
```markdown
# Main Title
## Section 1
- Item A
- Item B

[Link text](https://example.com)
```

**Element:** "headers"

**Output Note:**
```
# Main Title (level 1)
## Section 1 (level 2)
```

