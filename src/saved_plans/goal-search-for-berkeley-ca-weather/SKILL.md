---
name: goal-search-for-berkeley-ca-weather
description: Goal search for Berkeley, CA weather forecast for Nov 21, 2025.  Summarize forecast findings , format, and display.: ; actors: Jill; termination:
type: plan
manual_only: true
parameters:
  - name: query
    description: Parameter for query-web
    required: True
  - name: focus
    description: Parameter for summarize
    required: True
---

# goal-search-for-berkeley-ca-weather

Goal search for Berkeley, CA weather forecast for Nov 21, 2025.  Summarize forecast findings , format, and display.: ; actors: Jill; termination:

## Parameters

- `query`: Parameter for query-web (default: `Berkeley, CA weather forecast Nov 21 2025`)
- `focus`: Parameter for summarize (default: `Berkeley CA weather forecast Nov 21 2025`)

## Usage

```json
{
  "type": "goal-search-for-berkeley-ca-weather",
  "args": {"param1": "value1", ...},
  "out": "$result",
  "expect": "..."
}
```

## Implementation

This is a saved plan generated from an executed goal. The plan is stored in `plan.json` and will be executed directly when this tool is invoked.

Original goal: Goal search for Berkeley, CA weather forecast for Nov 21, 2025.  Summarize forecast findings , format, and display.: ; actors: Jill; termination:

Generated: 2025-11-23T10:34:00.512747
