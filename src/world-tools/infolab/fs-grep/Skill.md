---
name: fs-grep
type: python
description: "Search text files under scenarios/<world_name>/fs and return match snippets."
schema_hint: {"path": "string (relative)", "pattern": "string", "recursive": "bool", "max_matches": "int", "context": "int", "case_insensitive": "bool"}
---

# fs-grep

Search text files for a regex pattern and return match snippets as Notes.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (default: root)
- `pattern`: Regex pattern to search for (required)
- `recursive`: Recurse into subdirectories (default: true)
- `max_matches`: Max matches to return (default: 20)
- `context`: Context lines before/after each match (default: 0)
- `case_insensitive`: Case-insensitive regex (default: false)

## Output

Success returns:
- `resource_id`: Collection ID of match Notes
- Match Notes are text with:
  - metadata Note linked by meta Relation (path/line/pattern/context and file metadata)
  - body text snippet for the match

## Examples

```json
{"type":"fs-grep","path":"src","pattern":"TODO","recursive":true,"out":"$todos"}
{"type":"fs-grep","pattern":"error","context":2,"out":"$errors"}
```
