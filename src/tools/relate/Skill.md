---
name: relate
description: Compare two Notes or Collections to find similarities, differences, and relationships. Use to identify semantic relationships between documents
type: python
schema_hint:
  target: "$variable or Note ID or Collection ID"
  other: "$variable or Note ID or Collection ID"
  instruction: "string (optional, natural language guidance)"
  out: "$variable"
examples:
  - '{"type":"relate","target":"$doc1","other":"$doc2","out":"$comparison"}'
  - '{"type":"relate","target":"$scaling_laws","other":"$compute_optimal","instruction":"identify how scaling laws inform compute-optimal training strategies","out":"$comparison"}'
---

# Relate

Perform semantic comparison of two Notes or Collections to identify overlaps, contradictions, unique elements, and conceptual relationships.

## Purpose

- Deduplication detection
- Conflict identification
- Knowledge consolidation
- Relationship discovery

## Input

- `target`: First Note/Collection to compare
- `other`: Second Note/Collection to compare (REQUIRED)
- `instruction`: Optional natural language guidance (e.g., "focus on methodology differences", "compare cost implications")

Tool automatically flattens Collections (including nested Collections) and fetches Note content.

## Output

Returns JSON comparison analysis:

```json
{
  "similarity_score": 0.75,
  "shared_themes": ["theme1", "theme2"],
  "unique_to_first": ["element1", "element2"],
  "unique_to_second": ["element3"],
  "contradictions": [
    {"aspect": "date", "first": "2024", "second": "2025"}
  ],
  "relationship": "first_elaborates_on_second",
  "summary": "Both discuss X, but first adds Y perspective"
}
```

## Usage

Basic comparison:
```json
{"type":"relate","target":"$note1","other":"$note2","out":"$comparison"}
```

With instruction:
```json
{"type":"relate","target":"$scaling_laws","other":"$compute_optimal","instruction":"identify how scaling laws inform compute-optimal training strategies","out":"$comparison"}
```

## Relationship Types

Common relationships identified:
- **duplicate**: Substantially identical content
- **elaborates**: One expands on the other
- **contradicts**: Factual conflicts
- **complements**: Non-overlapping useful info
- **supersedes**: Newer/better version
- **synthesizes**: Combines multiple sources
- **exemplifies**: Specific instance of general concept
- **critiques**: Critical response to original

## Guidelines

- **Specificity**: Cite specific elements, not vague similarities
- **Balance**: Note both agreements and differences
- **Context**: Consider source, purpose, audience
- **Nuance**: Distinguish factual conflicts from perspective differences
