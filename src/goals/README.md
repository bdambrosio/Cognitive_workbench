# Goals Directory

This directory contains reusable goal templates that can be executed directly from the UI.

## Purpose

Goals define **what** you want to achieve. They are high-level objectives that the incremental planner will break down into executable plans.

## Directory Structure

```
src/
├── goals/          # Goal templates (this directory)
├── saved_plans/    # Pre-built execution plans
├── tools/          # Individual capabilities
└── ...
```

## Goal File Format

Goals are defined in YAML format:

```yaml
name: "Goal Title"
description: "Brief description of what this goal accomplishes"

test_goal: >
  goal: Your goal text here.
  Additional instructions or context.

expected_metrics:  # Optional - used for testing
  plan_completed: true
  type_violations: 0
```

### Required Fields

- `name`: Display name for the UI
- `description`: Brief explanation
- `test_goal`: The actual goal text to execute (must start with "goal:")

### Optional Fields

- `expected_metrics`: Used if this goal is also used as a test case

## Usage

### From UI

1. Navigate to the **Goals** tab in the character panel
2. Click a goal card to inspect its full content
3. Click **▶️ Execute** to run the goal for the active character
4. The planner will create a plan and execute it

### From Command Line

Goals can also be executed programmatically via the API or by publishing to the character's goal topic.

## Creating New Goals

1. Create a new `.yaml` file in this directory
2. Follow the format above
3. The goal will automatically appear in the Goals tab
4. Test it by executing it for a character

## Examples

- `Weather.yaml` - Get weather forecast for a location
- `JSON_SQL_semantic_scholar.yaml` - Demonstrate JSON SQL operations on academic papers
- `Emergence_research.yaml` - Research a complex topic

## Relationship to Tests

Goals in this directory can also serve as evaluation tests. The `tests/eval/` directory may contain additional test-specific goals with more detailed assertions and metrics. User-facing goals here prioritize clarity and reusability over testing completeness.

