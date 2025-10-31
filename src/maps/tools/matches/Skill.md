---
name: matches
description: Check if text matches regex pattern or contains substring
type: prompt_augmentation
parameters: pattern (required) - text string or regex pattern to match; match_type (optional) - "substring"|"regex"|"exact"
examples:
  - '{"type":"if","condition":{"type":"tool_condition","tool":"matches","target":"$text","args":{"pattern":"error"}},"then":[...]}'
---

# Contains Pattern

Check whether text contains a pattern or substring.

## Purpose

- Enable conditional logic based on content
- Pattern matching for flow control
- Content validation
- Simple filtering

## Input Format

Accepts a Note containing text to check.

## Parameters

**Required:**
- **pattern** - Text string or regex pattern to match

**Optional:**
- **match_type** - "substring" (default), "regex", or "exact"

## Output Format

Returns a Note containing:
- "true" if pattern found
- "false" if pattern not found

## Usage Examples

**Simple substring check:**
```json
{"type":"contains-pattern","target":"$text","args":{"pattern":"error"},"out":"has_error"}
```

**Regex pattern match:**
```json
{"type":"contains-pattern","target":"$email","args":{"pattern":"^[a-z]+@example\\.com$","match_type":"regex"},"out":"is_valid_email"}
```

**Exact match:**
```json
{"type":"contains-pattern","target":"$status","args":{"pattern":"complete","match_type":"exact"},"out":"is_complete"}
```

## Match Types

- **substring**: Pattern appears anywhere in text (case-sensitive)
- **regex**: Full regex pattern matching
- **exact**: Entire text must exactly match pattern

## Guidelines

- Case-sensitive by default
- Returns boolean as text ("true"/"false")
- Use for conditional logic in plans
- Combine with other conditionals as needed

## Examples

**Substring (default):**
```
Text: "Error occurred in module X"
Pattern: "Error"
Result: true
```

**Regex:**
```
Text: "user@example.com"
Pattern: ".*@example\\.com$"
Result: true
```

**Exact:**
```
Text: "done"
Pattern: "done"
Result: true

Text: "almost done"
Pattern: "done"
Result: false
```

