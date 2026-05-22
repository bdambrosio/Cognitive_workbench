---
name: exec-script
description: Run a bash/shell script — for system commands the existing tools don't cover. Trusted by default; treat with the same care as any code-execution action.
args:
  script: required string — the bash script text to execute
---

# exec-script

Run an arbitrary bash script via `bash -c` in the scenario's `fs/` directory. Returns stdout + stderr as a single text observation.

## Behavior

- Resolves the script text from `script` and runs it in a subprocess with cwd = scenario filesystem root.
- Captures stdout and stderr together. Non-zero exit codes are surfaced in the observation but do not by themselves abort the ReAct loop.
- **Trusted by default** — no interactive permission prompt. Treat with the same caution you would treat any code-execution tool.

## Examples

```json
{"thought": "list the working directory", "tool": "exec-script", "script": "ls -la"}
```

```json
{"thought": "count lines in the changelog", "tool": "exec-script", "script": "wc -l CHANGELOG.md"}
```

## Notes

- Use focused single-purpose scripts; avoid pipelines that mix actions you don't both need to see results from.
- Prefer `fetch-text` / `obsidian` / `check-email` for tasks they cover — `exec-script` is the catch-all for what they don't.
