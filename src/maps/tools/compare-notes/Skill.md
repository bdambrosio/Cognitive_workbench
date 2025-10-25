---
name: compare-notes
description: Find similarities, differences, and relationships between two or more Notes
type: prompt_augmentation
parameters: comparison_mode (optional) - 'similarity'|'contradiction'|'comprehensive'; threshold (optional) - minimum similarity (default 0.3); focus_aspects (optional) - specific dimensions to compare
---

# Compare Notes

Perform semantic comparison of content to identify overlaps, contradictions, unique elements, and conceptual relationships.

## Purpose

Support:
- Deduplication detection
- Conflict identification
- Knowledge consolidation
- Change tracking
- Relationship discovery

## Input Format

Accepts:
- **Single Note** - returns self-analysis
- **Two or more Notes** - via Collection variable (recommended)
- Tool automatically unpacks Collection and compares all member Notes

**Usage Pattern:**
```json
{"type":"create","kind":"Collection","value":["$note1","$note2"],"out":"notes_to_compare"}
{"type":"apply","target":"compare-notes","value":"$notes_to_compare","out":"comparison"}
```

Optional focus argument for targeted comparison:
```json
{"type":"apply","target":"compare-notes","value":"$notes_to_compare","args":{"focus":"methodology differences"},"out":"comparison"}
```

## Output Format

Returns comparison analysis:

**For 2 Notes:**
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

**For multiple Notes:**
```json
{
  "common_themes": ["theme1"],
  "consensus_points": ["point1", "point2"],
  "divergences": [
    {"aspect": "approach", "views": {"note1": "view1", "note2": "view2"}}
  ],
  "outliers": ["note3"],
  "clusters": [
    {"notes": ["note1", "note2"], "theme": "agrees on X"},
    {"notes": ["note4", "note5"], "theme": "disagrees on Y"}
  ],
  "summary": "Three main perspectives emerge..."
}
```

## Comparison Dimensions

### Semantic Similarity
- Content overlap (0.0 to 1.0 score)
- Shared concepts and vocabulary
- Thematic alignment

### Structural Comparison
- Information coverage (what's included/omitted)
- Level of detail
- Organization/framing

### Factual Analysis
- Agreement on facts
- Contradictions or conflicts
- Complementary information

### Temporal/Causal
- Chronological relationships
- Evolution of ideas
- Dependency/prerequisite relationships

## Relationship Types

Common relationships to identify:
- **duplicate**: Substantially identical content
- **elaborates**: One expands on the other
- **contradicts**: Factual conflicts
- **complements**: Non-overlapping useful info
- **supersedes**: Newer/better version
- **synthesizes**: Combines multiple sources
- **exemplifies**: Specific instance of general concept
- **critiques**: Critical response to original

## Quality Guidelines

- **Specificity**: Cite specific elements, not vague similarities
- **Balance**: Note both agreements and differences
- **Context**: Consider source, purpose, audience
- **Nuance**: Distinguish factual conflicts from perspective differences
- **Objectivity**: Describe differences without judgment

## Parameters

Optional args:
- `comparison_mode`: 'similarity' | 'contradiction' | 'comprehensive' (default)
- `threshold`: Minimum similarity to report (default: 0.3)
- `focus_aspects`: List of specific dimensions to compare

## Example

**Input:**
```json
[
  {"id": "note1", "content": "AGI timeline estimates range from 5-50 years"},
  {"id": "note2", "content": "Experts predict AGI within 10-30 years, with high uncertainty"}
]
```

**Output:**
```json
{
  "similarity_score": 0.82,
  "shared_themes": ["AGI timeline prediction", "uncertainty"],
  "unique_to_first": ["50 year upper bound"],
  "unique_to_second": ["expert consensus framing"],
  "contradictions": [],
  "relationship": "complements",
  "summary": "Both address AGI timelines with overlapping ranges; note2 emphasizes expert survey basis while note1 shows wider uncertainty range"
}
```json