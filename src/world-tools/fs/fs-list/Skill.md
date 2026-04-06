---
name: fs-list
type: python
description: "List files and directories under scenarios/<world_name>/fs; returns one Note whose body is a text listing (like ls)."
schema_hint: {"path": "string (relative)", "recursive": "bool", "include_files": "bool", "include_dirs": "bool", "max_entries": "int"}
---

# fs-list

List files and directories within the filesystem sandbox for the current world.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (default: root)
- `recursive`: Whether to recurse into subdirectories (default: false)
- `include_files`: Include files (default: true)
- `include_dirs`: Include subdirectory lines in the listing (default: true)
- `max_entries`: Max lines to emit before truncation (default: 200)

## Output

Success returns:

- `resource_id`: **Note** ID (not a Collection)
- The Note's **text body** is a human-readable directory listing:
  - First line: the listed path (relative)
  - **Directories**: `dirname/` (trailing slash)
  - **Files**: `filename  (size)` — e.g. `readme.md  (2.1K)`, `note.txt  (219B)`
  - Truncation line if `max_entries` reached

**Read the listing with `get_text("$your_out_var")`.** Do **not** use `get_items` — that is only for Collections.

Parse lines in Python: skip empty lines, lines ending in `/` are dirs; for files, strip the `  (size)` suffix or split on `(` and take the first segment for the basename.

## Examples

```json
{"type":"fs-list","path":".","out":"$root"}
{"type":"fs-list","path":"docs","recursive":true,"out":"$docs_tree"}
```

After `out="$root"`, use `text = get_text("$root")` and split into lines.
