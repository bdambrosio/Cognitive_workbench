---
name: fs-list
type: python
description: "List files and directories under scenarios/<world_name>/fs and return a Collection."
schema_hint: {"path": "string (relative)", "recursive": "bool", "include_files": "bool", "include_dirs": "bool", "max_entries": "int"}
---

# fs-list

List files and directories within the filesystem sandbox for the current world.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (default: root)
- `recursive`: Whether to recurse into subdirectories (default: false)
- `include_files`: Include files (default: true)
- `include_dirs`: Include subdirectories as Collections (default: true)
- `max_entries`: Max entries created across recursion (default: 200)

## Output

Success returns:
- `resource_id`: Collection ID
- Collection items are Notes (files) and Collections (subdirectories)
- File Notes are placeholders with text content:
  - metadata Note linked by meta Relation (path/size/mtime/format)
  - body text `(placeholder)`

## Examples

```json
{"type":"fs-list","path":".","out":"$root"}
{"type":"fs-list","path":"docs","recursive":true,"out":"$docs_tree"}
```
