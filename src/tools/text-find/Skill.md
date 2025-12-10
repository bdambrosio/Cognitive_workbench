---
name: text-find
description: Locate pattern or substring and return position with context
type: prompt_augmentation
examples:
  - '{"type":"text-find","target":"$document","pattern":"conclusion","out":"$location"}'
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
{"type":"text-find","target":"$document","pattern":"TODO","out":"$todo_locations"}
```

**Pattern search with context:**
```json
{"type":"text-find","target":"$log","pattern":"ERROR.*timeout","context_lines":3,"out":"$errors"}
```

**Find email addresses:**
```json
{"type":"text-find","target":"$contact_info","pattern":"[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}","out":"$emails"}
```

## Guidelines

- Case-sensitive by default
- Supports standard regex syntax
- Returns all matches, not just first
- Context helps understand match relevance

## Example

**Input:** Note with text containing "TODO" on lines 2 and 4  
**Pattern:** "TODO"  
**Output:** Note with 2 matches, each showing line number, matched text, and 1 line of context before/after

