---
name: format
description: Format text content for readable display with headers and structure
type: python
trusted: true
parameters: title (optional), style (optional - document/list/outline)
---

# Format

Transforms plain text into well-formatted, structured output suitable for display in popups or documents.

## Purpose

Takes raw text content and adds visual structure - headers, spacing, bullets, or outlines - to make it more readable and professional looking.

## Input Format

Accepts:
- Plain text string
- Text with newlines (will be preserved or reformatted based on style)

## Optional Parameters

- **title** - Custom title for the formatted output (default: "Formatted Content")
- **style** - Formatting style:
  - `document` (default) - Paragraphs with wrapping
  - `list` - Bulleted list format
  - `outline` - Numbered outline format

## Output Format

Returns formatted text with:
- Header with title (centered, bordered)
- Structured content based on style
- Footer separator
- Proper spacing and indentation

## Example Usage

**Input (plain text):**
```
Mushrooms are fascinating organisms. They play a key role in ecosystems. Some are edible and nutritious.
```

**Output (document style):**
```
============================================================
                    Formatted Content                      
============================================================

Content:

  Mushrooms are fascinating organisms. They play a key role
  in ecosystems. Some are edible and nutritious.

------------------------------------------------------------
```

**Output (list style with custom title):**
```
============================================================
                    About Mushrooms                        
============================================================

Content:

  • Mushrooms are fascinating organisms
  • They play a key role in ecosystems
  • Some are edible and nutritious

------------------------------------------------------------
```

## Use Cases

- Formatting notes before display
- Creating readable reports from raw data
- Structuring information for user presentation
- Preparing content for popup displays

## Typical Pattern

```json
{"type": "format", "target": "$raw_note", "out": "formatted"}
{"type": "display", "target": "user", "value": "$formatted"}
```

## Error Handling

- Non-string input: Returns error
- Empty input: Returns error
- Long lines: Automatically wrapped at 70 characters

