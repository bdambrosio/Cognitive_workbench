---
name: scienceworld-reset
type: python
description: "Start a ScienceWorld episode. Reads scenario, difficulty, and seed from scienceworld_config in character YAML. Returns the initial observation plus a session_id for subsequent steps."
schema_hint:
  value: "ignored (scenario comes from scienceworld_config)"
  out: "$variable"
examples:
  - '{"type":"scienceworld-reset","out":"$sw_session"}'
---

# ScienceWorld Reset (Level 4)
## Input
- Scenario, difficulty, and seed are read from `scienceworld_config` in the character YAML file
- `value` parameter is ignored (scenario comes from config)
- No parameters needed - tool reads everything from config

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: initial observation/description
  - `metadata.session_id`: identifier for this session
  - `metadata.scenario`, `metadata.variation_idx`, `metadata.simplification`, `metadata.seed`
  - `metadata.reward` (0 at reset), `metadata.done` (false)

## Requirements
- Python package `scienceworld` available in the environment.
- ScienceWorld assets accessible (installed with the package).

## Configuration
Requires `scienceworld_config` section in character YAML:
```yaml
scienceworld_config:
  scenario: "waterplant"
  difficulty: 0
  seed: 42
```

## Common Workflow
```json
{"type":"scienceworld-reset","out":"$sw"}
{"type":"scienceworld-act","action":"look","out":"$o1"}
```

Note: `scienceworld-act` no longer requires `session_id` - it uses the active session stored in `executive_node.scienceworld_env`.

