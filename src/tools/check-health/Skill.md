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
- `value`: Human-readable summary of health status — lists only concerns if any exist, otherwise "All systems nominal."
- `data`: Full structured report dict with per-signal status classifications.
- `extra`: Contains `overall_status` ("ok", "warning", or "critical") and `concerns` list.

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

The planner reads `$health` and decides whether to `say` or `ask` the user based on the report contents.
