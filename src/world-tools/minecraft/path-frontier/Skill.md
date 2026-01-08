---
name: path-frontier
type: python
description: "Enumerate locally reachable positions within a bounded number of nav actions. Reasoning tool using conservative BFS simulation."
---

# path-frontier

Reasoning tool that enumerates reachable nearby positions using conservative bounded BFS over nav semantics. Does not execute movement; uses simulation only.

## Input

- `max_actions`: Integer maximum number of nav actions to explore (default: `4`)
- `allow_unknown`: Boolean, if True allows unknown cells in simulation (default: `False`)

## Output

Returns uniform_return format. Access return value via `result["data"]` (not `result["value"]`).

Success (`status: "success"`):
- `data`: Dict containing:
  - `reachable`: List of `{"x": int, "z": int, "path": List[str]}` positions reachable within bounds
    - `path`: Sequence of action names from start to position (e.g., `["nav-turn-right", "nav-move", "nav-climb"]`)
    - path is a simulation estimate, and may fail in execution
  - `count`: Integer count of reachable positions
- `value`: Formatted/truncated version of data (for display)
- `resource_id`: Note ID if result was persisted

Failure (`status: "failed"`):
- `reason`: One of `"executor_not_available"`, `"status_failed"`
- `data`: Reason string (same as `reason`)

## Behavior

- Performs bounded BFS exploration using `nav_simulation.simulate_nav_step`
- Explores `nav-move`, `nav-descend`, `nav-climb`, and `nav-turn` (left/right only)
- Uses spatial map from `world_state("spatial_map")` for cell queries
- Excludes starting position from results
- Conservative: unknown cells treated as failure unless `allow_unknown=True`
- `nav-turn` changes facing direction without moving position (counts as one action)

## Planning Notes

- Use for reasoning about reachability, not for actual movement
- Requires spatial map to be populated (via `mc-map-update`)
- Small `max_actions` (default 4) keeps exploration bounded
- May return 0 positions. This is NOT an error, and does not require recovery
- Results are (x, z) positions only (Y coordinate not included in output)

## Example

```json
{"type":"path-frontier","max_actions":3,"out":"$frontier"}
```
