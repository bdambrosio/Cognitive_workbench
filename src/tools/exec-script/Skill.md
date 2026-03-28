---
name: exec-script
type: python
description: "Execute an arbitrary bash script in the scenario filesystem, with user permission."
schema_hint:
  target: "string or $variable (bash script text — if $variable, must be a Note, not a Collection)"
  out: "$variable (captured console output)"
---

# exec-script

Execute an arbitrary bash script in the scenario's `fs/` directory after asking the user for permission.

## Input

- `target`: The bash script to execute. Either a string literal or a `$variable` bound to a Note whose content is the script. Binding to a Collection is an error.

## Output

Success (`status: "success"`):
- `value`: Console output (stdout + stderr) produced by the script

Failure (`status: "failed"`):
- `reason`: Error description (empty script, Collection binding, permission denied, execution error)

## Behavior

1. Resolves the script text from `target` (literal string or Note binding; Collection binding fails).
2. Validates that the resolved script is non-empty text.
3. **This tool automatically asks the user for permission before executing**, displaying the full script text. Do NOT use ASK_USER before calling exec-script — the tool handles its own permission prompt.
4. If permission granted, executes the script via `bash -c` in `scenarios/<world_name>/fs/`.
5. Captures and returns stdout + stderr as the `out` value.

## Examples

```json
{"type": "exec-script", "target": "ls -la", "out": "$listing"}
{"type": "exec-script", "target": "$my_script", "out": "$result"}
```
