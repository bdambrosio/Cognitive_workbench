---
name: _test_loadable
description: Stub peripheral for Phase 2.1 loader-plumbing validation. No runtime behavior.
type: peripheral
config: {}
registered_tools: []
registered_alerts: []
---

# Test Loadable Peripheral

Phase 2.1 stub. Exists only to validate that a peripheral declaration with
`type: peripheral` is discovered, parsed, and indexed by `peripheral_loader`.
The accompanying `peripheral.py` is intentionally empty.
