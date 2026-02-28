---
name: fs-head
type: python
description: "Read the first N lines of a text file under scenarios/<world_name>/fs."
schema_hint: {"path": "string (relative)", "lines": "int"}
---

# fs-head

Return the first N lines of a text file from the filesystem sandbox.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (required)
- `lines`: Number of lines to return (default: 20)

## Output

Success returns:
- `resource_id`: Note ID containing text content.
- Note content format:
  - metadata Note linked by meta Relation (path/line range/truncation metadata)
  - body text with the first N lines

## Examples

```json
{"type":"fs-head","path":"logs/app.log","lines":50,"out":"$log_head"}
```
