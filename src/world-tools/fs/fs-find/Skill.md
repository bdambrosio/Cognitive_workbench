---
name: fs-find
type: python
description: "Find files by filename pattern (glob) under scenarios/<world_name>/fs and return a Collection."
schema_hint: {"pattern": "string (glob pattern)", "path": "string (relative)", "recursive": "bool", "max_results": "int"}
---

# fs-find

Find files by filename pattern (glob) within the filesystem sandbox for the current world.

## Input

- `pattern`: Glob pattern for filename matching (required, e.g., "*.txt", "Nan_*", "*Ar*")
- `path`: Directory to search (default: "." root)
- `recursive`: Whether to search subdirectories (default: true)
- `max_results`: Maximum files to return (default: 200)

## Pattern Matching

- Uses shell-style glob patterns (via Python `fnmatch`)
- Matches against **filename only** (not full path)
- Common patterns:
  - `"*.txt"` - all .txt files
  - `"Nan_*"` - files starting with "Nan_"
  - `"*Ar*"` - files containing "Ar"
  - `"Nan_Ar.txt"` - exact filename match

**Note:** Pattern cannot contain path separators (`/` or `\`). Use the `path` parameter to search in specific directories.

## Output

Success returns:
- `resource_id`: Collection ID containing matching file Notes
- Collection items are Notes (files) with same structure as `fs-list` file Notes

## Examples

```json
{"type":"fs-find","pattern":"*.txt","out":"$txt_files"}
{"type":"fs-find","pattern":"Nan_*","path":"bhagavan","recursive":false,"out":"$matches"}
{"type":"fs-find","pattern":"*Ar*","out":"$ar_files"}
```
