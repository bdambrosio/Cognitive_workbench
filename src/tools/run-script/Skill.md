---
name: run-script
type: python
description: "Run a shell script from src/scripts/. Looks up {script_name}.sh and runs it under bash. Does not wait for completion."
schema_hint:
  script_name: "string (name without .sh, e.g. daily_post)"
---

# run-script

Run a shell script from `src/scripts/`. Looks up `{script_name}.sh` and submits to bash. Does not wait for completion.

## Input

- `script_name`: Script name without .sh (e.g. `daily_post` → `src/scripts/daily_post.sh`)

## Output

Success: "Script {name} started" (fire-and-forget).

Failure: script not found, or invalid script_name.

## Behavior

- Script must exist at `src/scripts/{script_name}.sh`
- Runs under bash. Does not capture output or return code.
- Fails if script not found.

## Common Workflows

```json
{"type":"run-script","script_name":"daily_post","out":"$result"}
{"type":"say","target":"User","value":"$result"}
```
