---
name: check-health
type: python
description: "Collect system and cognitive health metrics. Returns a structured health report with status classifications (ok/warning/critical) for hardware, OS, and cognitive subsystems."
schema_hint:
  out: "$variable (optional)"
---

# check-health

Homeostasis monitor. Collects hardware metrics (GPU, CPU, RAM, disk, network, processes) via system-health.sh and cognitive metrics (memory integrity, tool effectiveness, world model health, planning performance) from internal state. Returns a structured health report.

## Input

No required input. Runs all checks automatically.

## Output

Success (`status: "success"`):
- `value`: Human-readable summary string — lists concerns if any, otherwise "All systems nominal."
- The bound Note (`$health`) contains a **JSON object** with these top-level keys:
  - `overall_status`: `"ok"` | `"warning"` | `"critical"`
  - `concerns`: list of human-readable concern strings (empty if all ok)
  - `system`: hardware/OS metrics (cpu, gpu, ram, disk, network, processes)
  - `cognitive`: cognitive metrics (memory, planning, tool_model, world_model, user_interaction)

## Reading the health report in code blocks

```python
health = executor.get_json("$health")
overall = health["overall_status"]       # "ok", "warning", or "critical"
concerns = health["concerns"]            # e.g. ["Plan completion rate low: 45% (last 20 plans)"]
```

**Do NOT** use `health.get("extra", {}).get("overall_status")` — `overall_status` and `concerns`
are at the **top level** of the Note content, not nested under `extra`.

## Behavior

- Calls `src/scripts/system-health.sh` synchronously to collect OS/hardware metrics
- Reads cognitive state from executor, resource_manager, tool_model, and world_model
- Classifies each signal as ok / warning / critical based on thresholds
- Does NOT communicate with the user — returns data for the planner to act on

## Common Workflows

**Scheduled health check (planner decides what to report):**
```json
{"type":"check-health","out":"$health"}
```

The planner reads `$health` via `executor.get_json("$health")` and decides whether to
`say` or `ask` the user based on `overall_status` and `concerns`.
