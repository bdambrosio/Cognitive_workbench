# Infospace Evaluation Framework

This directory contains test files for evaluating incremental planner compliance with infospace type system rules.

## Overview

The evaluation framework tests whether the planner:
1. **Respects type compatibility** - Note-only ops on Notes, Collection-only ops on Collections
2. **Follows tool restrictions** - Only uses allowed tools when specified
3. **Generates valid plans** - Plans execute successfully without type violations

## Architecture

### Components

1. **ComplianceTracker** (`src/infospace_compliance.py`)
   - Tracks type violations during execution
   - Monitors tool allowlist compliance
   - Records compatibility checks

2. **Plan Execution Metrics** (`src/executive_node.py`)
   - Captures compliance data in `_summarize_plan_execution`
   - Stores metrics in `{character}-plans.jsonl`
   - Available in `metrics.infospace_compliance` field

3. **Interactive Test UI** (`src/fastapi_action_display.py`)
   - 🧪 Test button in web UI
   - Test file picker with preview
   - Real-time test execution
   - Visual feedback on violations

### Type Compatibility Rules

```
┌─────────────────────────┬──────┬────────────┐
│ Operation               │ Note │ Collection │
├─────────────────────────┼──────┼────────────┤
│ expand, as-json, refine │  ✓   │     ❌     │
│ summarize, relate       │  ✓   │     ✓      │
│ map, flatten            │  ❌  │     ✓      │
└─────────────────────────┴──────┴────────────┘
```

## Test File Format

```yaml
name: "Test Name"
description: "What this test validates"

# Optional: Restrict which tools planner can use
allowed_tools:
  - create-note
  - summarize
  - display

# Optional: Setup goals to run before main test
setup_goals:
  - "goal: Create initial test data"

# Main test goal
test_goal: "goal: Your test scenario here"

# Expected outcomes (for future automated validation)
expected_metrics:
  type_violations: 0
  tool_misuse: 0
  plan_completed: true
```

## Usage

### Via Web UI (Recommended)

1. Start the system with an infospace character:
   ```bash
   cd src
   python launcher.py ../scenarios/your_infospace_scenario.yaml --ui
   ```

2. Open browser to `http://localhost:3000`

3. Click **🧪 Test** button

4. Select:
   - **Character**: The infospace character to test (e.g., Jill)
   - **Test File**: One of the tests from this directory

5. Click **▶️ Run Test**

6. Watch execution in real-time, see compliance results

### Analyzing Results

After test execution, check the plan log:
```bash
cat data/Jill-plans.jsonl | tail -1 | jq '.metrics.infospace_compliance'
```

Output example:
```json
{
  "type_violations": [
    {
      "type": "note_only_on_collection",
      "operation": "expand",
      "variable": "$my_collection",
      "expected_type": "Note",
      "actual_type": "Collection"
    }
  ],
  "tool_misuse": [],
  "allowed_tools": ["create-note", "summarize", "display"],
  "compatibility_checks": 5,
  "evaluation_mode": true
}
```

## Example Tests

1. **note_collection_compatibility.yaml**
   - Tests Note vs Collection operation rules
   - Verifies expand, map, flatten usage

2. **tool_allowlist.yaml**
   - Tests tool restriction enforcement
   - Ensures disallowed tools aren't used

3. **complex_workflow.yaml**
   - Multi-step workflow with type awareness
   - Tests expand → map → union → summarize chain

## Creating New Tests

1. Create `tests/eval/your_test.yaml`

2. Define test goal that exercises specific behaviors

3. Set `allowed_tools` to restrict planner options (optional)

4. Run via UI to validate

5. Check results in plan log

## Integration with Metrics

Compliance data is automatically captured in:
- `data/{character}-plans.jsonl` - Per-plan compliance metrics
- Future: Aggregate analysis scripts in `src/Metrics/`

## Why This Approach?

**Challenge**: SGLang startup takes ~1 minute, making test-per-process expensive.

**Solution**: Send tests to already-running system via Zenoh messaging. This amortizes startup cost across multiple tests and enables:
- Fast iteration (no restarts)
- Interactive debugging (watch execution in UI)
- Real-time feedback (see violations as they occur)

## Future Enhancements

1. Automated test suite runner (batch mode)
2. Metrics aggregation scripts
3. Historical trend analysis
4. Test result comparison

