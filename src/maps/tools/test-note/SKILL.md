---
name: test-note
description: Test Note content against natural language predicate using LLM (returns boolean)
type: python
trusted: true
parameters:
  - name: predicate
    type: string
    description: Natural language question or test condition
---

# Test Note

Universal boolean test tool that evaluates natural language predicates against Note content using LLM.

## Purpose

Provides flexible conditionals for plan control flow. Use when:
- No specialized condition exists
- Complex reasoning over content required
- Ad-hoc validation needed

## Input

- `predicate`: Natural language question (e.g., "contains citations?", "is valid JSON?", "mentions neural networks?")
- `value`: Note content to test

## Output

Returns "true" or "false" as string (for use in tool_condition).

## Usage Examples

Test for citations:
```json
{"type":"if","condition":{"type":"tool_condition","tool":"test-note","target":"$paper","args":{"predicate":"contains academic citations?"}},"then":[...]}
```

Check data validity:
```json
{"type":"if","condition":{"type":"tool_condition","tool":"test-note","target":"$data","args":{"predicate":"is valid JSON with 'results' field?"}},"then":[...]}
```

Content filtering:
```json
{"type":"if","condition":{"type":"tool_condition","tool":"test-note","target":"$text","args":{"predicate":"discusses machine learning or AI?"}},"then":[...]}
```

## Guidelines

- **Be specific:** Clear predicates yield accurate results. "Contains author and year metadata?" > "has metadata?"
- **Binary questions:** Phrase as yes/no questions for clearest results.
- **Prefer specialized conditions:** Use built-in conditions (contains, equals) when possible - they're faster.
- **Cost aware:** Every call hits LLM. Use judiciously in loops.

## Performance

- Fast execution: ~1-2 seconds per test
- Logged for usage analysis

