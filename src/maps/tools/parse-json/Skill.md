---
name: parse-json
description: Extract values from JSON content by path or query
type: prompt_augmentation
parameters: path (optional) - JSONPath or key to extract specific value
---

# Parse JSON

Extract and access structured data from JSON content.

## Purpose

- Parse JSON strings into accessible data
- Extract specific values by path
- Validate JSON structure
- Enable downstream structured operations

## Input Format

Accepts a Note containing JSON text.

## Parameters

**Optional:**
- **path** - JSONPath expression or key to extract (e.g., "results[0].title", "metadata.author")

## Output Format

Returns a Note containing:
- **With path**: Extracted value as text
- **Without path**: Entire parsed JSON structure as formatted text
- **On failure**: null or error message

## Usage Examples

**Parse entire JSON:**
```json
{"type":"parse-json","target":"$json_note","out":"parsed_data"}
```

**Extract specific value:**
```json
{"type":"parse-json","target":"$json_note","args":{"path":"results[0].name"},"out":"first_name"}
```

**Extract nested field:**
```json
{"type":"parse-json","target":"$api_response","args":{"path":"data.user.email"},"out":"user_email"}
```

## Guidelines

- Returns null if content is not valid JSON
- Empty or missing path returns full parsed structure
- Supports standard JSONPath syntax
- Preserves data types where possible

## Example

**Input Note:**
```json
{"users": [{"name": "Alice", "age": 30}, {"name": "Bob", "age": 25}]}
```

**Path:** "users[0].name"

**Output Note:**
```
Alice
```

