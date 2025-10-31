---
name: save
description: Create a Collection from items and persist it to filesystem
type: plan
examples:
  - '{"type":"save","args":{"items":["$note1","$note2"],"name":"research-findings"},"out":"$saved_collection"}'
---

# Create and Persist

Creates a Collection from provided items and marks it as persistent so it's saved to filesystem.

## Purpose

Convenience tool that combines two common operations:
1. Creating a Collection from items
2. Marking it persistent for filesystem storage

## Parameters

- **items**: List of items (can be $variables, literal values, or Note/Collection IDs)
- **name**: Name for the Collection

## Output

Returns the persisted Collection ID bound to the output variable.

## Usage Example

```json
{
  "type": "create-and-persist",
  "args": {
    "items": ["$note1", "$note2", "$note3"],
    "name": "research-findings"
  },
  "out": "saved_collection"
}
```

## Notes

- Collection is automatically marked persistent
- Will survive system restarts
- Items can be mix of Notes and Collections

