---
name: as-markdown
description: Extract markdown document structure and content from mixed/embedded text (NOT for converting plain text TO markdown)
type: prompt_augmentation
parameters: element (optional) - type to extract (headers|lists|links|code|section|all)
examples:
  - '{"type":"as-markdown","target":"$llm_response","args":{"element":"headers"},"out":"$toc"}'
---

# As Markdown

Interpret note content as markdown, automatically identifying markdown within mixed content and extracting structural elements.

## Purpose

- Extract markdown from LLM responses with noise
- Parse document structure (headers, lists, links, code)
- Extract content under specific sections
- Strip surrounding non-markdown text
- Validate markdown present

## Input Format

Accepts a Note containing markdown (clean or embedded):
- Clean markdown
- Markdown with surrounding text
- Code-fenced markdown blocks
- Mixed format responses

## Parameters

**Optional:**
- **element** - Element type: "headers", "lists", "links", "code", "section", or "all" (default)
- **section** - When element="section", specify header name to extract content under
- **all** - If true, returns array of all matching elements; if false/omitted, returns first match (default: false)

## Output Format

Returns extracted elements:
- **headers**: List of headers with levels
- **lists**: All list items (ordered and unordered)
- **links (all=false)**: First link found; (all=true): All links
- **code (all=false)**: First code block; (all=true): All code blocks
- **section**: Content under specified header
- **all**: Structured breakdown of all elements
- **Element not found**: note-null
- **Markdown not identified**: FAIL (hard error)

## Usage Examples

**Extract all structure:**
```json
{"type":"as-markdown","target":"$llm_response","out":"$structure"}
```

**Extract headers for TOC:**
```json
{"type":"as-markdown","target":"$doc","args":{"element":"headers"},"out":"$toc"}
```

**Extract specific section:**
```json
{"type":"as-markdown","target":"$doc","args":{"element":"section","section":"Usage"},"out":"$usage"}
```

**Extract first code block:**
```json
{"type":"as-markdown","target":"$response","args":{"element":"code"},"out":"$code"}
```

**Extract all code blocks:**
```json
{"type":"as-markdown","target":"$response","args":{"element":"code","all":true},"out":"$all_code"}
```

## Guidelines

- Strips leading/trailing non-markdown text automatically
- Element not found returns note-null (soft failure)
- No markdown identified triggers FAIL (hard failure)
- Preserves nesting for lists
- Code blocks include language identifier if present
- Default extracts first match; use `all: true` for all matches (applies to links, code)

## Examples

**Example 1: Extract headers**

Input:
```markdown
# Main Title
## Section 1
Text here.
## Section 2
More text.
```

Element: "headers"

Output:
```
# Main Title (level 1)
## Section 1 (level 2)
## Section 2 (level 2)
```

**Example 2: Extract section content**

Input:
```markdown
# Overview
General info.
# Usage
Run the command.
# Notes
Other details.
```

Element: "section", Section: "Usage"

Output:
```
Run the command.
```

**Example 3: Extract first code block**

Input:
```markdown
```python
print("hello")
```
More text.
```javascript
console.log("world")
```
```

Element: "code"

Output:
```
python: print("hello")
```

**Example 4: Extract all code blocks**

Input:
```markdown
```python
print("hello")
```
More text.
```javascript
console.log("world")
```
```

Element: "code", all: true

Output:
```
["python: print(\"hello\")", "javascript: console.log(\"world\")"]
```

**Example 5: Missing element returns null**

Input: `# Title\nNo lists here.`

Element: "lists"

Output: note-null

