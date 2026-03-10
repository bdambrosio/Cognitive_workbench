---
name: run-script
type: python
description: "Submit a shell script from src/scripts/ for execution. Success means the script was submitted successfully."
schema_hint:
  script_name: "string (name without .sh, e.g. daily_post)"
---

# run-script

Submit a shell script from `src/scripts/` for execution. The tool's job is to submit the script — successful submission means success. No further verification is needed.

## Input

- `script_name`: Script name without .sh (e.g. `daily_post` → `src/scripts/daily_post.sh`)

## Output

Success: "Script {name} started" — the goal is achieved.

Failure: script not found, or invalid script_name.

## Behavior

- Script must exist at `src/scripts/{script_name}.sh`
- Submits script to bash for execution.
- Success means the script was submitted. No further polling or log-checking is required.
- Fails if script not found.

## Common Workflows

```json
{"type":"run-script","script_name":"daily_post","out":"$result"}
{"type":"say","target":"User","value":"$result"}
```
