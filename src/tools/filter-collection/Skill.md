---
name: filter-collection
type: python
description: "Evaluate complex text criteria on each item in a Collection and return a new Collection of matching items. Use to identify relevant items in a Collection"
---

# Filter Collection Tool

Apply flexible, natural-language filtering criteria to Collections. Evaluates each item against specified conditions and returns a new Collection containing only matching items.

## Purpose

Enable complex filtering that goes beyond simple field comparisons: content-based filtering (semantic, not just keyword), multi-condition logic, contextual evaluation, and subjective/qualitative criteria.

## Input

- `target`: Collection ID or variable (required)
- `predicate`: String describing filter criteria (required)
- `mode`: "include" (return matches) or "exclude" (return non-matches) (optional, default: "include")

## Output

Returns ID of new Collection containing only items matching the predicate. Empty result returns new empty Collection.

## Behavior & Performance

- Evaluates each item individually using LLM
- Supports simple predicates ("contains practical examples"), complex predicates ("discusses either reinforcement learning OR supervised learning"), semantic predicates ("has a positive tone"), and comparative predicates ("more detailed than typical overview")
- Handles numeric predicates (parse and compare numerically), date predicates (handle relative dates), negation ("NOT X"), and boolean logic (AND/OR with standard precedence)

## Guidelines

- Interpretation: Apply reasonable semantic interpretation, use content context not just keywords, consider implicit meaning when clear
- Edge cases: Ambiguous items excluded unless predicate clearly matches, partial matches included if substantial match (>70% of criteria), missing fields treated as non-match unless predicate allows null
- Consistency: Apply same standard across all items, similar items should receive similar treatment
- Quality: No false positives/negatives on clear cases, evaluate entire item content not just opening

## Usage Examples

Content filtering:
```json
{"type":"filter-collection","target":"$collection","predicate":"contains code or implementation details","out":"$filtered"}
```

Complex logic:
```json
{"type":"filter-collection","target":"$collection","predicate":"mentions safety AND published after 2024","out":"$filtered"}
```

Exclusion mode:
```json
{"type":"filter-collection","target":"$collection","predicate":"purely theoretical without practical applications","mode":"exclude","out":"$filtered"}
```
