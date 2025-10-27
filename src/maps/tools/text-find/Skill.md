---
name: text-find
description: Locate pattern or substring and return position with context
type: prompt_augmentation
parameters: pattern (required) - text or regex pattern to find; context_lines (optional) - lines of context to include
---

# Text Find

Locate patterns or substrings in text and return matches with context.

## Purpose

- Find specific text or patterns
- Get position and surrounding context
- Support regex pattern matching
- Enable conditional logic based on content presence

## Input Format

Accepts a Note containing text to search.

## Parameters

**Required:**
- **pattern** - Text string or regex pattern to find

**Optional:**
- **context_lines** - Number of lines before/after to include (default: 1)

## Output Format

Returns a Note containing match results:
- Match positions (line and character)
- Matched text
- Surrounding context
- Count of matches

Returns null if no matches found.

## Usage Examples

**Simple text search:**
```json
{"type":"apply","target":"text-find","value":"$document","args":{"pattern":"TODO"},"out":"todo_locations"}
```

**Pattern search with context:**
```json
{"type":"apply","target":"text-find","value":"$log","args":{"pattern":"ERROR.*timeout","context_lines":3},"out":"errors"}
```

**Find email addresses:**
```json
{"type":"apply","target":"text-find","value":"$contact_info","args":{"pattern":"[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}"},"out":"emails"}
```

## Guidelines

- Case-sensitive by default
- Supports standard regex syntax
- Returns all matches, not just first
- Context helps understand match relevance

## Example

**Input Note:**
```
Line 1: Normal text
Line 2: TODO: Fix bug
Line 3: More text
Line 4: TODO: Review code
```

**Pattern:** "TODO"

**Output Note:**
```
Found 2 matches:

Match 1 (line 2):
Line 1: Normal text
Line 2: TODO: Fix bug
Line 3: More text

Match 2 (line 4):
Line 3: More text
Line 4: TODO: Review code
```

