# Sensor Spec — Change Order 1: Disposition & Boundary Clarifications

## 1. Add `disposition` to sensor configuration

### SKILL.md frontmatter

Add `disposition` as a supported field, default `"inform"`:

```yaml
---
name: arxiv-monitor
description: ...
type: plan
schedule: "30m"
disposition: "inform"        # NEW — see vocabulary below
gate: null
tools: [...]
parameters: {}
---
```

### scenario.yaml

`disposition` is declarable per-sensor in the character's sensor list, overriding the SKILL.md default (same override pattern as schedule, gate, parameters):

```yaml
sensors:
  - name: arxiv-monitor
    schedule: "30m"
    disposition: "inform"
    parameters: { ... }

  - name: pr-watcher
    schedule: "5m"
    disposition: "trigger:code-review"
    gate: "nonempty:new-prs"

  - name: api-health
    schedule: "1m"
    disposition: "alert"
```

### Disposition vocabulary (initial)

- **`inform`** — Add to agent's awareness context for next OODA cycle. No goal activation or priority change. The agent may or may not act on it. (Default.)
- **`trigger:<goal_name>`** — Activate or reprioritize the named scheduled goal. Loader validates that `<goal_name>` exists in the character's goal schedule at startup.
- **`alert`** — High-priority. Surfaces as an urgent item in the agent's next OODA orient phase. Specific handling TBD as agent-side consumption is built out.

Vocabulary is deliberately minimal. Expect it to need expansion once real sensors are running.

## 2. Stamp disposition into sense_data payload

The SensorRunner includes disposition in the content payload it publishes:

```python
content_payload = {
    'source': f'sensor:{sensor_name}',
    'text': result_content,
    'disposition': disposition_string    # NEW — from merged config
}
```

This travels through the existing sense_data channel unchanged. The OODA loop reads disposition from the parsed content to determine routing. No changes to sense_data_callback itself — it queues as before; disposition is consumed downstream.

## 3. Startup validation for trigger dispositions

In launcher integration (step 3 of Implementation Order), after loading sensors and parsing character configs:

- For any sensor with `disposition: "trigger:<goal_name>"`, validate that `<goal_name>` appears in the character's scheduled goals.
- Log a warning (not a fatal error) if the goal name is not found — the goal may be created dynamically at runtime.

## 4. Clarify sensor/goal boundary

Add to **Constraints and Non-Goals** section:

> **Sensors do not plan.** Sensors execute fixed plans via `execute_plan_sync` or run Python code. They never invoke the incremental planner or generate new plans in response to findings. If a sensor discovers something requiring adaptive reasoning, it pushes the finding to the agent via disposition — the agent decides what to do.
>
> **One-directional flow (for now).** Sensors push to agents. Agents do not manage, reconfigure, or communicate back to sensors. This asymmetry is deliberate and may be relaxed in future iterations if concrete scenarios require bidirectional coordination. The current design should not preclude adding agent-to-sensor influence later (e.g., via Zenoh channels or infospace state that sensors monitor), but this is not in scope.
>
> **Sensors and goals are distinct by degree, not by kind.** Both are scheduled activities that can invoke tools. Sensors are lightweight, fixed-plan, and operate below the agent's cognitive horizon. Goals involve full agent reasoning via the incremental planner. The distinction is operational, not categorical.

## 5. Agent-side consumption (scope note)

The spec currently covers sensor production and delivery. How the OODA loop consumes sensor output — triaging by disposition, integrating `inform` items into context, activating goals for `trigger` items, handling `alert` priority — is a **separate design problem** to be addressed when modifying the OODA loop. The disposition field provides the data contract; the routing logic is not specified here.
