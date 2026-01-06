---
name: relate
type: python
description: "Compare two Notes or Collections to find similarities, differences, and relationships. Use to identify semantic relationships between documents"
---

# Relate Tool

Perform semantic comparison of two Notes or Collections to identify overlaps, contradictions, unique elements, and conceptual relationships.

## Purpose

Deduplication detection, conflict identification, knowledge consolidation, and relationship discovery between documents.

## Input

- `target`: First Note/Collection to compare
- `other`: Second Note/Collection to compare (required)
- `instruction`: Optional natural language guidance (e.g., "focus on methodology differences", "compare cost implications")

## Output

Returns JSON Note containing comparison analysis:
- `similarity_score`: Numeric similarity (0-1)
- `shared_themes`: List of common themes
- `unique_to_first`: List of elements only in first document
- `unique_to_second`: List of elements only in second document
- `contradictions`: List of conflicts with aspect, first value, second value
- `relationship`: Relationship type (e.g., "first_elaborates_on_second")
- `summary`: Text summary of comparison

## Behavior & Performance

- Tool automatically flattens Collections (including nested Collections) and fetches Note content
- Uses LLM for semantic comparison
- Identifies relationship types: duplicate, elaborates, contradicts, complements, supersedes, synthesizes, exemplifies, critiques

## Guidelines

- Specificity: Cite specific elements, not vague similarities
- Balance: Note both agreements and differences
- Context: Consider source, purpose, audience
- Nuance: Distinguish factual conflicts from perspective differences

## Usage Examples

Basic comparison:
```json
{"type":"relate","target":"$note1","other":"$note2","out":"$comparison"}
```

With instruction:
```json
{"type":"relate","target":"$scaling_laws","other":"$compute_optimal","instruction":"identify how scaling laws inform compute-optimal training strategies","out":"$comparison"}
```
