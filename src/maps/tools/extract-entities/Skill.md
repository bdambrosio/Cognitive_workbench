---
name: extract-entities
description: Extract named entities, topics, and relationships from text or structured content
type: python
trusted: true
parameters: none
examples:
  - '{"type":"extract-entities","target":"$paper_text","out":"$entities","expect":"should find authors and organizations"}'
---

# Extract Entities

Identify and extract structured information from unstructured text: people, places, organizations, topics, dates, and relationships between entities.

## Purpose

Transform free-form text into structured entity data for:
- Building knowledge graphs
- Indexing and retrieval
- Pattern detection across documents
- Linking related content

## Input Format

Accepts:
- Plain text (paragraphs, documents)
- Structured data with text fields
- Lists of text snippets

## Output Format

Returns JSON structure:
```json
{
  "people": ["Name1", "Name2"],
  "organizations": ["Org1", "Org2"],
  "locations": ["Place1", "Place2"],
  "topics": ["Topic1", "Topic2"],
  "dates": ["2025-01-15", "last week"],
  "key_concepts": ["Concept1", "Concept2"],
  "relationships": [
    {"subject": "Name1", "predicate": "works_at", "object": "Org1"},
    {"subject": "Topic1", "predicate": "relates_to", "object": "Topic2"}
  ]
}
```

## Extraction Guidelines

### Entity Categories

**People**: Full names, roles, pronouns with clear referents
- Include professional titles if mentioned
- Resolve pronouns when unambiguous

**Organizations**: Companies, institutions, projects, teams
- Include both formal and informal names
- Note parent/subsidiary relationships

**Locations**: Cities, countries, venues, virtual spaces
- Be specific when possible (not just "the office")

**Topics**: Domain areas, technologies, methodologies
- Extract at appropriate granularity (not too broad/narrow)
- Include synonyms if multiple terms used

**Dates/Time**: Absolute and relative temporal references
- Normalize when possible (ISO format for absolute dates)
- Preserve relative references ("next week", "recently")

**Key Concepts**: Abstract ideas, themes, goals
- Focus on novel or emphasized concepts
- Distinguish from general background

### Relationships

Extract explicit and strongly implied relationships:
- Employment/affiliation
- Collaboration/partnership
- Causation/dependency
- Temporal ordering
- Hierarchical structure

**Format**: `{subject, predicate, object}` triples

### Quality Standards

- **Precision over recall**: Only extract clear, confident entities
- **Disambiguation**: Use context to resolve ambiguous references
- **Normalization**: Consistent entity naming across text
- **No hallucination**: Never infer entities not present in source

## Special Handling

**Pronouns**: Resolve only when antecedent is clear and recent
**Abbreviations**: Expand on first use, preserve thereafter
**Implicit entities**: Extract only if strongly implied by context
**Conflicting info**: Note conflicts in relationships field

## Parameters

Optional args dict can specify:
- `entity_types`: List of types to extract (default: all)
- `include_confidence`: Boolean, add confidence scores (default: false)
- `max_entities_per_type`: Limit results (default: unlimited)

## Example

**Input:**
```
Sarah joined Anthropic last quarter to work on constitutional AI. 
She previously collaborated with researchers at DeepMind on alignment.
```

**Output:**
```json
{
  "people": ["Sarah"],
  "organizations": ["Anthropic", "DeepMind"],
  "topics": ["constitutional AI", "alignment"],
  "dates": ["last quarter"],
  "key_concepts": ["alignment research"],
  "relationships": [
    {"subject": "Sarah", "predicate": "works_at", "object": "Anthropic"},
    {"subject": "Sarah", "predicate": "previously_at", "object": "DeepMind"},
    {"subject": "Sarah", "predicate": "works_on", "object": "constitutional AI"}
  ]
}
```