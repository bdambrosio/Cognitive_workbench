---
name: filter-by-predicate
description: Evaluate complex criteria on each item in a collection and return matching items
type: prompt_augmentation
parameters: predicate (required) - filtering condition to evaluate
examples:
  - '{"type":"filter-by-predicate","target":"$collection","args":{"predicate":"contains technical content"},"out":"$filtered"}'
---

# Filter By Predicate

Apply flexible, natural-language filtering criteria to Collections. Evaluates each item against specified conditions and returns only matching items.

## Purpose

Enable complex filtering that goes beyond simple field comparisons:
- Content-based filtering (semantic, not just keyword)
- Multi-condition logic
- Contextual evaluation
- Subjective/qualitative criteria

## Input Format

Expects a Collection (list) of items. Each item can be:
- Plain text strings
- Structured objects (dicts with fields)
- Mixed types
```json
[
  {"id": "item1", "content": "...", ...},
  {"id": "item2", "content": "...", ...},
  "plain text item",
  {...}
]
```

## Output Format

Returns filtered list containing only items matching the predicate:
```json
[
  {"id": "item1", "content": "..."},
  {"id": "item3", "content": "..."}
]
```

Empty list if no matches: `[]`

## Parameters

Required in `args`:
- `predicate`: String describing filter criteria

Optional in `args`:
- `mode`: 'include' (return matches) or 'exclude' (return non-matches), default: 'include'

## Predicate Language

Write predicates as natural language conditions. The tool evaluates each item's content against the criteria.

### Simple Predicates
- "contains practical examples"
- "discusses security concerns"
- "mentions specific companies"
- "written in technical language"
- "expresses uncertainty"

### Complex Predicates
- "discusses either reinforcement learning OR supervised learning"
- "mentions AI safety AND includes specific proposals"
- "published after 2024 AND cites empirical results"
- "NOT just theoretical discussion"

### Semantic Predicates
- "has a positive tone"
- "is relevant to healthcare applications"
- "contains actionable recommendations"
- "discusses limitations or risks"
- "suitable for non-technical audience"

### Comparative Predicates
- "more detailed than typical overview"
- "focuses on recent developments (last 2 years)"
- "higher quality than blog post level"

## Evaluation Guidelines

### Interpretation
- Apply reasonable semantic interpretation
- Use content context, not just keywords
- Consider implicit meaning when clear

### Edge Cases
- **Ambiguous items**: Exclude unless predicate clearly matches
- **Partial matches**: Include if substantial match (>70% of criteria)
- **Missing fields**: Treat as non-match unless predicate allows null

### Consistency
- Apply same standard across all items
- Similar items should receive similar treatment

## Quality Standards

- **Accuracy**: No false positives/negatives on clear cases
- **Consistency**: Same criteria applied uniformly
- **Completeness**: Evaluate entire item content, not just opening

## Special Handling

**Numeric predicates**: Parse and compare numerically when possible
**Date predicates**: Handle relative dates ("recent", "last month")
**Negation**: "NOT X" excludes items matching X
**Boolean logic**: AND/OR handled with standard precedence (AND before OR)
**Empty/null items**: Skip or treat as non-match based on predicate context

## Examples

### Example 1: Content Filtering

**Input (target):**
```json
[
  {"id": "1", "content": "How to build a neural network from scratch with code examples"},
  {"id": "2", "content": "Overview of deep learning theory and mathematical foundations"},
  {"id": "3", "content": "Implementing transformers in PyTorch with complete tutorial"}
]
```

**Parameters (args):**
```json
{
  "predicate": "contains code or implementation details"
}
```

**Output:**
```json
[
  {"id": "1", "content": "How to build a neural network from scratch with code examples"},
  {"id": "3", "content": "Implementing transformers in PyTorch with complete tutorial"}
]
```

### Example 2: Complex Logic

**Input (target):**
```json
[
  {"title": "AI Safety Research Agenda", "date": "2025-01"},
  {"title": "Historical AI Developments", "date": "2020-06"},
  {"title": "AI Safety Implementation Guide", "date": "2024-12"},
  {"title": "Machine Learning Basics", "date": "2025-02"}
]
```

**Parameters (args):**
```json
{
  "predicate": "mentions safety AND published after 2024"
}
```

**Output:**
```json
[
  {"title": "AI Safety Research Agenda", "date": "2025-01"},
  {"title": "AI Safety Implementation Guide", "date": "2024-12"}
]
```

### Example 3: Exclusion Mode

**Input (target):**
```json
[
  "Theoretical framework for AGI",
  "Practical deployment of ML models in production",
  "Abstract mathematical proofs in learning theory"
]
```

**Parameters (args):**
```json
{
  "predicate": "purely theoretical without practical applications",
  "mode": "exclude"
}
```

**Output:**
```json
[
  "Practical deployment of ML models in production"
]
```

## Error Handling

- **Empty collection**: Return `[]`
- **Malformed predicate**: Return empty list and log error
- **Unparseable items**: Skip and continue processing remaining items
- **No predicate in args**: Return original collection unchanged (no filtering)

## Implementation Notes

Tool should:
1. Iterate through input list
2. Evaluate each item against predicate
3. Collect matches (or non-matches if mode='exclude')
4. Return filtered list in same structure as input
5. Preserve item structure exactly (no modifications to matched items)
