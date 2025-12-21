---
name: scienceworld-act
type: python
description: "Take a ScienceWorld action in the active session. Returns observation, reward, done. No session_id needed - uses active session from executive_node."
schema_hint:
  action: "string (text command)"
  out: "$variable"
examples:
  - '{"type":"scienceworld-act","action":"look","out":"$step1"}'
---

# ScienceWorld Act (Level 4)
## Input
- `action`: text command (e.g., "look", "go north", "take shovel")
- No `session_id` needed - uses active session from executive_node.scienceworld_env

## Output
- Note ID (bound to `out` variable) containing:
  - `text`: observation after the action
  - `metadata.reward`, `metadata.done`
  - `metadata.session_id`, `metadata.action`, `metadata.info`

## Workflow
```json
{"type":"scienceworld-reset","out":"$sw"}
{"type":"scienceworld-act","action":"look","out":"$o1"}
{"type":"scienceworld-act","action":"take watering can","out":"$o2"}
```


