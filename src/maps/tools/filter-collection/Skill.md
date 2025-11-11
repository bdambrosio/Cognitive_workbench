--- 
name: filter-collection
description: Evaluate complex criteria on each item in a Collection and return a new Collection of matching items
type: python
trusted: true
parameters: predicate (required) - filtering condition to evaluate
examples:
  - '{"type":"filter-collection","target":"$collection","args":{"predicate":"contains technical content"},"out":"$filtered","expect":"should return technical papers only"}'
---

# Filter Collection By Predicate

Apply flexible, natural-language filtering criteria to Collections. Evaluates each item against specified conditions and returns a new Collection containing only matching items.

## Purpose

Enable complex filtering that goes beyond simple field comparisons:
- Content-based filtering (semantic, not just keyword)
- Multi-condition logic
- Contextual evaluation
- Subjective/qualitative criteria

## Input Format

Expects a Collection (list of note_ids or sub-collections). Each item is evaluated individually.

```json
["Note_1", "Note_2", "Collection_3"]
```

## Output Format

Returns ID of new Collection containing only items matching the predicate:

```json
"Collection_5"
```

The new Collection will have `item_count` reflecting the number of matches.

Empty result: New empty Collection (`[]`).

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
1. Fetch full content for each note_id/sub-collection in input
2. Evaluate each item against predicate using LLM
3. Collect matching note_ids
4. Create new Collection with filtered list
5. Return the new Collection ID
6. Preserve original item structure and metadata
