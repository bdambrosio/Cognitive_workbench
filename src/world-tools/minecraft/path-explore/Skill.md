---
name: path-explore
type: python
description: "Perform one frontier-directed exploration episode. Selects farthest reachable position and attempts to reach it via nav primitives."
---

# path-explore

Autonomous exploration: reasons about reachable frontier, selects farthest target, attempts execution, updates map.

## Input

None. Operates using current agent state.

**Important**: This tool does NOT accept target coordinates as input. It autonomously selects the farthest reachable frontier position from its internal spatial map. The selected target may differ from externally computed targets due to map latency or local navigation constraints.

## Output

Success (`status: "success"`):
- `outcome`: `"no_frontier"` | `"reached"` | `"blocked"` | `"no_progress"` | `"status_failed"`
- `target`: `{"x": int, "z": int}` — autonomously selected target position, present unless `outcome` is `"no_frontier"`

Failure (`status: "failed"`):
- `reason`: `"path_frontier_failed"` | `"path_frontier_failed_allow_unknown"` | `"status_failed"`

## Behavior

1. Calls `path-frontier`; retries with `allow_unknown=True` if no positions found
2. Autonomously selects farthest reachable position (Euclidean distance) from frontier
3. Executes toward autonomously-selected target via `nav-advance` (1 block/step, max 8 steps)
4. Calls `mc-map-update` before returning

Target Selection: The tool selects targets based on its internal spatial map state at execution time. The selected target is reported in the output but cannot be specified by the caller.

Outcome meanings:
- `"reached"`: target position reached
- `"blocked"`: nav-advance failed mid-path
- `"no_progress"`: max steps reached without arriving
- `"status_failed"`: mc-status failed during execution
- `"no_frontier"`: no reachable positions (after retry)

## Planning Notes

- **Autonomous target selection**: Tool selects targets autonomously; cannot specify coordinates
- No guarantees of coverage, optimality, or success
- Path simulation may fail in execution (paths are estimates)
- Use for autonomous exploration, not precise navigation
- Selected target may differ from externally computed targets due to map latency or navigation constraints
- Multiple calls may be needed to cover an area
- `"no_frontier"` with success status is expected when boxed in—not an error
- Internally calls: `path-frontier`, `mc-status`, `nav-advance`, `mc-map-update`

## Example

```json
{"type":"path-explore","out":"$explore_result"}
```
