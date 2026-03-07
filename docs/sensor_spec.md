# Sensor Subsystem Specification

## Overview

Sensors are autonomous, asynchronous information-gathering processes associated with an agent. They run independently of the agent's OODA loop on their own schedule, execute a predefined plan, and push results to the agent via the existing `cognitive/{character_name}/sense_data` Zenoh channel. The agent encounters sensor output during its normal observe phase — it does not manage, invoke, or reason about sensors directly.

Sensors are configured per-character in `scenario.yaml` and loaded from `src/sensors/` following the same directory-scanning convention as `src/tools/`.

### Design Principles

- **Push, not poll.** Sensors deliver information to the agent; the agent never polls sensors. This relieves the agent of deciding when to check for updates and enables event-driven reactivity.
- **Below the cognitive horizon.** The agent does not know sensors exist as a concept. It simply sees new information arriving in its sense_data stream. Sensor infrastructure is invisible to the agent.
- **Scenario-configured, not agent-chosen.** The scenario designer decides what sensors a character has. There is no dynamic sensor management at runtime (for now).
- **Minimal machinery.** Sensors reuse `execute_plan_sync` for plan-based execution and the existing Zenoh `sense_data` channel for output. No new communication infrastructure is needed.

## Architecture

### Directory Structure

```
src/
  sensors/
    arxiv-monitor/
      SKILL.md          # Frontmatter metadata + description (same format as tools)
      plan.json          # Optional: static plan for plan-type sensors
      sensor.py          # Optional: Python implementation for code-type sensors
    rss-watcher/
      SKILL.md
      sensor.py
    ...
```

### Sensor Types

**`plan`** — A static plan (list of infospace actions) executed via `execute_plan_sync`. Suitable for sensors that need LLM reasoning within steps (generate-note, refine, etc.) but follow a fixed execution structure. The plan is defined in `plan.json`, identical in format to plan-type tools.

**`code`** — A Python function executed directly. Suitable for deterministic tasks (RSS polling, API checks, file watching) that don't need LLM reasoning. The Python file must expose a `run(context) -> dict` function.

### SKILL.md Frontmatter

```yaml
---
name: arxiv-monitor
description: Monitors arxiv for new papers matching configured search terms
type: plan                    # plan | code
schedule: "30m"               # Execution interval (see Schedule Format below)
# Optional fields:
gate: null                    # Event-driven gating condition (see Gating below)
tools:                        # Tool subset available to plan-type sensors (default: all agent tools)
  - query-web
  - semantic-scholar
  - generate-note
  - refine
parameters: {}                # Static parameters passed to plan/code at each execution
---
```

### Schedule Format

- Interval shorthand: `"30s"`, `"5m"`, `"1h"`, `"2h30m"`
- On-startup flag: `"0s"` or `"startup"` means run once immediately, then stop (one-shot sensor)

Future extension (not for initial implementation): cron expressions.

### Gating (Event-Driven Sensors)

A sensor with a `gate` condition only executes its plan/code when the gate evaluates to true. Between evaluations, the sensor thread sleeps on its normal schedule but skips execution if the gate is not satisfied. This reduces state accumulation — sensors only report when there is something worth reporting.

Gate conditions for initial implementation:

- `null` — No gating; sensor runs every interval (default).
- `"changed:<resource_name>"` — Run only if the named resource has been modified since the last sensor execution.
- `"nonempty:<variable_or_resource>"` — Run only if a code-type sensor's lightweight pre-check returns data.

Gating is evaluated *before* the potentially expensive plan/code execution. For code-type sensors, the `sensor.py` may expose an optional `gate(context) -> bool` function as an alternative to declarative gate expressions.

## Configuration

### scenario.yaml

Sensors are declared per-character under a `sensors` key:

```yaml
characters:
  - name: Researcher
    # ... existing character config ...
    sensors:
      - name: arxiv-monitor
        schedule: "30m"
        parameters:
          search_terms: ["multi-agent systems", "LLM planning"]
          max_results: 5
      - name: rss-watcher
        schedule: "15m"
        parameters:
          feeds:
            - https://news.ycombinator.com/rss
          keywords: ["agent", "LLM"]
      - name: inbox-check
        schedule: "2m"
        gate: "nonempty:inbox"
```

The `name` field must match a directory under `src/sensors/`. Override fields (`schedule`, `parameters`, `gate`) take precedence over the sensor's SKILL.md defaults.

## Execution

### Sensor Lifecycle

1. **Loading.** At launcher startup, after character parsing, call `load_sensors()` (analogous to `load_tools()`) to scan `src/sensors/` and build sensor metadata.

2. **Instantiation.** For each character, read the `sensors` list from character config. For each declared sensor, merge scenario overrides with SKILL.md defaults. Create a `SensorRunner` instance.

3. **Thread launch.** Each `SensorRunner` runs as a daemon thread, started after the agent thread. The launcher's `shutdown_event` is shared with all sensor threads for clean shutdown.

4. **Execution loop.** The `SensorRunner` loop:
   ```
   while not shutdown_event.is_set():
       sleep(interval)
       if gate and not evaluate_gate():
           continue
       result = execute()        # execute_plan_sync or run()
       if result has reportable content:
           push_to_agent(result)
   ```

5. **Shutdown.** On `shutdown_event`, sensor threads exit their sleep/loop cleanly.

### Plan-Type Sensor Execution

Plan-type sensors need access to an `InfospaceExecutor` to run `execute_plan_sync`. However, they should NOT share the agent's executor instance (concurrent mutation of bindings, plan_actions, etc. would cause race conditions).

Each plan-type sensor gets its own lightweight `InfospaceExecutor` instance:
- Shares the agent's `resource_manager` (thread-safe read access to infospace).
- Has its own `plan_bindings_flat` (isolated variable namespace per sensor run).
- Has access to the agent's Zenoh session for publishing.
- Has access to the configured tool subset (loaded tools filtered by the sensor's `tools` list, or all agent tools if not specified).
- Shares the agent's LLM runtime/tokenizer for inference (the SGLang runtime is already thread-safe).

### Code-Type Sensor Execution

Code-type sensors call `sensor.py:run(context)` directly. The context dict provides:

```python
context = {
    'parameters': {},           # From scenario.yaml sensor config
    'resource_manager': rm,     # Read access to infospace resources
    'zenoh_session': session,   # For any needed Zenoh operations
    'last_run': datetime,       # Timestamp of previous execution (None on first run)
    'run_count': int,           # How many times this sensor has executed
}
```

The `run()` function returns a dict:

```python
{
    'status': 'ok' | 'nothing' | 'error',
    'content': str,              # Text content to push to agent (if status == 'ok')
    'metadata': {}               # Optional metadata
}
```

If `status` is `'nothing'`, no message is pushed to the agent (the sensor ran but found nothing reportable). This is the code-level equivalent of gating.

### Output: Push to Agent

Sensor output is delivered via the existing Zenoh sense_data channel, using the same payload format that `sense_data_callback` already parses:

```python
content_payload = {
    'source': f'sensor:{sensor_name}',    # Distinguishable from 'User', agent names, 'console'
    'text': result_content
}
sense_data = {
    'timestamp': datetime.now().isoformat(),
    'sequence_id': 0,
    'mode': 'text',
    'content': json.dumps(content_payload)
}
zenoh_session.put(
    f"cognitive/{character_name}/sense_data",
    json.dumps(sense_data)
)
```

The `source` field uses a `sensor:` prefix so that `sense_data_callback` and downstream processing can identify sensor-originated input. The existing callback will queue this into `text_input_queue` like any other input.

### Heartbeat Convention

Sensors should periodically push a heartbeat even when they have nothing to report, so the agent (or monitoring infrastructure) can distinguish "nothing happening" from "sensor is dead":

```python
content_payload = {
    'source': f'sensor:{sensor_name}',
    'text': '',                           # Empty text = heartbeat, not queued by sense_data_callback
    'heartbeat': True,
    'timestamp': datetime.now().isoformat()
}
```

The existing `sense_data_callback` already filters empty `text_input` — heartbeats will be received but not queued into `text_input_queue`. A future monitoring layer can track heartbeat recency per sensor. For now, heartbeats are logged but not acted upon.

## Sensor Loader

`src/utils/sensor_loader.py` — follows the same pattern as `tool_loader.py`:

- `load_sensors(sensors_dir_path: str) -> Dict[str, Dict]`
- Scans immediate subdirectories of `src/sensors/`
- Parses SKILL.md frontmatter for metadata
- Loads `plan.json` for plan-type sensors
- Validates `sensor.py` existence and `run()` signature for code-type sensors
- Returns dict mapping `sensor_name -> metadata`

## SensorRunner

`src/sensor_runner.py` — the per-sensor execution thread:

```
class SensorRunner:
    def __init__(self,
                 sensor_name: str,
                 sensor_meta: dict,          # From sensor_loader
                 character_name: str,
                 character_config: dict,
                 scenario_overrides: dict,    # From character's sensors[] entry
                 resource_manager,
                 zenoh_session,
                 runtime,                     # Shared SGLang runtime (for plan-type)
                 tokenizer,                   # Shared tokenizer (for plan-type)
                 available_tools: dict,       # Agent's loaded tools (for plan-type)
                 shutdown_event: threading.Event)
```

Key responsibilities:
- Parse and apply schedule interval from config
- Evaluate gate conditions before execution
- For plan-type: create isolated InfospaceExecutor, call execute_plan_sync with the sensor's plan
- For code-type: import and call sensor.py:run()
- Format result and publish to agent's sense_data channel
- Handle errors gracefully (log, continue loop — a crashed sensor should not take down the agent)
- Track last_run timestamp and run_count for context

## Launcher Integration

In `launcher.py`, after agent threads are started:

1. Load all sensors: `all_sensors = load_sensors(src_dir / 'sensors')`
2. For each `(name, config)` in characters:
   - Read `config.get('sensors', [])`
   - For each declared sensor, look up in `all_sensors`, merge overrides
   - Create `SensorRunner` instance
   - Start as daemon thread
3. Sensor threads are tracked for logging/monitoring but do not need explicit join (daemon threads exit with process).

## Constraints and Non-Goals

- **No dynamic sensor management.** Sensors cannot be added, removed, or reconfigured at runtime. This is a scenario design decision.
- **No sensor-to-sensor communication.** Sensors are independent. If coordination is needed, it flows through infospace resources.
- **No dedicated UI.** Sensor activity appears in logs. Sensor-originated messages appear in the agent's conversation/action display like any other sense_data input. Future: a sensor status panel.
- **No shared mutable state between sensor and agent.** The sensor's executor has isolated bindings. Shared access is read-only through resource_manager. Sensor output flows exclusively through sense_data.
- **No cross-agent sensors (yet).** Each sensor is associated with exactly one character. Shared sensors feeding multiple agents are a future extension.

## State Accumulation Mitigation

Sensors that run frequently can flood the agent's input queue and conversation history. Mitigation strategies (to be refined through use):

1. **Gating.** Event-driven gates prevent execution when there's nothing new.
2. **Status: nothing.** Code-type sensors return `'nothing'` to suppress output when a run finds no reportable change.
3. **Sensor-side dedup.** Sensors should track what they've already reported (via `last_run` in context, or by checking infospace for previously created resources) and only push genuinely new information.
4. **Future: summarization.** A periodic "sensor digest" that compresses accumulated sensor notes, analogous to conversation summarization. Not in initial implementation.

## Example Sensors

### arxiv-monitor (plan-type)

```
src/sensors/arxiv-monitor/
  SKILL.md
  plan.json
```

**plan.json:**
```json
{
  "plan": [
    {"type": "semantic-scholar", "value": "$search_terms", "out": "$papers"},
    {"type": "refine", "target": "$papers", "instruction": "Filter to papers published in the last 24 hours. Summarize each in one sentence with title and key finding.", "out": "$recent"},
    {"type": "create-note", "value": "$recent", "name": "arxiv-digest", "out": "$digest"}
  ],
  "out": "$digest"
}
```

The SensorRunner binds `$search_terms` from scenario parameters before executing the plan. The final note content becomes the text pushed to the agent's sense_data.

### rss-watcher (code-type)

```
src/sensors/rss-watcher/
  SKILL.md
  sensor.py
```

**sensor.py:**
```python
import feedparser
from datetime import datetime

_seen_ids = set()

def run(context):
    feeds = context['parameters'].get('feeds', [])
    keywords = context['parameters'].get('keywords', [])
    new_items = []
    
    for feed_url in feeds:
        feed = feedparser.parse(feed_url)
        for entry in feed.entries:
            eid = entry.get('id', entry.get('link', ''))
            if eid in _seen_ids:
                continue
            _seen_ids.add(eid)
            title = entry.get('title', '')
            if keywords and not any(kw.lower() in title.lower() for kw in keywords):
                continue
            new_items.append(f"- {title}: {entry.get('link', '')}")
    
    if not new_items:
        return {'status': 'nothing', 'content': '', 'metadata': {}}
    
    return {
        'status': 'ok',
        'content': f"RSS updates ({len(new_items)} new):\n" + "\n".join(new_items),
        'metadata': {'item_count': len(new_items)}
    }
```

## Implementation Order

1. **sensor_loader.py** — Directory scanner, frontmatter parser, validation. Closely mirrors tool_loader.py.
2. **sensor_runner.py** — SensorRunner class with schedule loop, gate evaluation, plan/code execution, sense_data publishing.
3. **Launcher integration** — Load sensors, instantiate runners per character, start threads.
4. **First code-type sensor** — rss-watcher or similar, to validate the basic loop.
5. **First plan-type sensor** — arxiv-monitor or similar, to validate isolated executor + plan execution.
6. **Gating** — Implement gate evaluation for changed/nonempty conditions.
