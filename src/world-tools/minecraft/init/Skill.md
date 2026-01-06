---
name: init
type: python
description: "Initialize navigation state by normalizing agent pose to block center, cardinal yaw, and pitch zero."
schema_hint:
  value: "ignored"
  out: "$variable"
examples:
  - '{"type":"init","out":"$init_result"}'
  - '{"type":"init"}'
---

# Minecraft Initialization Tool

## Skill: INIT

### Level
Level 1 — Initialization primitive

### Purpose
Initialize navigation state by normalizing the agent's current pose to block center, cardinal yaw (0, 90, 180, or 270 degrees), and pitch zero. This tool is automatically executed during executor initialization if present in the tool catalog.

### Primitives Used
- `mc-status`: Get current position, yaw, and pitch
- `nav-turn`: Normalize orientation to cardinal direction and snap to block center
- `executor.set_world_state`: Update navigation state in world state

### Input
- `value`: ignored
- No parameters required

### Output
- Uniform return format dict:
  - `status`: 'success' or 'failed' (execution status)
  - `value`: Summary text describing the normalized pose
  - `data`: structured data dict containing:
    - `success`: Boolean indicating if initialization succeeded
    - `pose`: Normalized pose dict `{"x": float, "y": float, "z": float, "yaw": float}`
    - `normalized_from`: Original pose before normalization `{"x": float, "y": float, "z": float, "yaw": float, "pitch": float}` (only present if `success=True`)
    - `failure_reason`: One of `"status_failed"`, `"turn_failed"`, `"turn_failed_<reason>"`, `"normalized_status_failed"` (only present if `success=False`)
  - `resource_id`: None (no resource created)

### Success Outcomes

When `success=True`:
- `success`: True
- `pose`: Normalized pose with block-centered position (x, z at block center ±0.5), cardinal yaw (0, 90, 180, or 270), and pitch 0
- `normalized_from`: Original pose before normalization

### Failure Outcomes

When `success=False`:
- `success`: False
- `failure_reason`: Specific reason for failure

Failure Reasons:
- `"status_failed"`: Failed to get initial status from `mc-status`
- `"turn_failed"`: `nav-turn` execution failed
- `"turn_failed_<reason>"`: `nav-turn` failed with specific reason (e.g., `"turn_failed_status_failed"`, `"turn_failed_snapto_failed"`)
- `"normalized_status_failed"`: Failed to get normalized status after `nav-turn`

### Post-Conditions

After successful initialization:
- Agent is positioned at block center (x, z coordinates at block center ±0.5)
- Agent yaw is a cardinal direction (0°, 90°, 180°, or 270°)
- Agent pitch is 0° (horizontal)
- Navigation state is initialized in `world_state.nav` with normalized pose

### Invariants

- Always normalizes pose before initializing state
- Always verifies normalization with second `mc-status` call
- Navigation state structure is consistent (pose, support_here, fell, history)
- Block-centered positioning ensures predictable navigation starting point

### Notes

- This tool is automatically executed during executor initialization if present in the tool catalog
- The tool name must be `init` or `<world_name>-init` (e.g., `minecraft-init`) to be auto-executed
- Normalization ensures consistent starting state for navigation operations
- Support assessment (`support_here`) is left as `"unknown"` and should be updated by first navigation operation
- Navigation history starts empty and is populated by `nav-moveone`, `nav-climb`, and `nav-descend` tools

### Common Workflow

This tool is typically executed automatically during system initialization:

```json
{"type":"init"}
```

- No manual invocation needed if tool is present in `world-tools/<world_name>/init/`
- Executor automatically runs init tool after full tool catalog is loaded
- Higher-level planners can assume navigation state is initialized after executor startup

### Cognitive Contract

- INITIALIZATION-FOCUSED: Establishes consistent starting state for navigation
- NORMALIZATION-BASED: Ensures predictable pose (block-centered, cardinal yaw, zero pitch)
- STATE-MANAGEMENT: Initializes world state structure for navigation tracking
- AUTOMATIC: Executes during executor initialization if present
- VERIFICATION-BASED: Confirms normalization before reporting success

