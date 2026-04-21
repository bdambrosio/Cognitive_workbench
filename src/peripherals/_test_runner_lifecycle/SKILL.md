---
name: _test_runner_lifecycle
description: Phase 2.2 stub. Exercises PeripheralRunner lifecycle (instantiate, run, stop) with a no-op Peripheral class.
type: peripheral
config:
  tick_period_s: 0.05
registered_tools: []
registered_alerts: []
---

# Test Runner Lifecycle Peripheral

Phase 2.2 stub. Exports a `Peripheral` class whose `run(stop_event)` loops at
`tick_period_s`, incrementing a tick counter, until `stop_event` is set. Used
by `tests/test_peripheral_runner.py` to verify the runner's lifecycle.
