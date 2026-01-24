---
name: fs-stat
type: python
description: "Return metadata for a file or directory under scenarios/<world_name>/fs."
schema_hint: {"path": "string (relative)"}
---

# fs-stat

Return metadata for a file or directory in the filesystem sandbox.

## Input

- `path`: Relative path under `scenarios/<world_name>/fs` (required)

## Output

Success returns:
- `resource_id`: Note ID containing metadata

## Examples

```json
{"type":"fs-stat","path":"data/config.json","out":"$stat"}
```
