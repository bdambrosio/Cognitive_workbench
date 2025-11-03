---
name: refine
description: Transform Note content using natural language instruction via LLM
type: python
trusted: true
flattens_collections: true
parameters:
  - name: instruction
    type: string
    description: Natural language instruction for transformation
examples:
  - '{"type":"refine","target":"$data","args":{"instruction":"extract schema as JSON"},"out":"$schema","expect":"should extract JSON schema"}'
  - '{"type":"refine","target":"$paper","args":{"instruction":"list all citations in JSON format"},"out":"$citations","expect":"should find citation list"}'
---

# Transform Note

Universal transformation tool that applies natural language instructions to Note content using LLM.

## Purpose

Provides flexible, ad-hoc transformations without requiring specialized tools. Use when:
- No specialized tool exists for the transformation
- One-off or exploratory analysis
- Complex reasoning over content needed
- Novel or unexpected data formats

## Input

- `instruction`: Natural language instruction (e.g., "extract schema", "identify citations", "convert to bullet list")
- `target`: Note content to transform

## Output

Returns Note containing transformed content according to instruction.

## Usage Examples

Extract schema:
```json
{"type":"refine","target":"$data","args":{"instruction":"extract schema"},"out":"$schema","expect":"should extract JSON schema"}
```

Extract citations:
```json
{"type":"refine","target":"$paper","args":{"instruction":"list all citations in JSON format"},"out":"$citations","expect":"should find citations"}
```

## Guidelines

- **Prefer specialized tools when available:** Use as-json, extract-entities, etc. when they exist - they're faster and cheaper.
- **Be specific:** Clear instructions yield better results. "Extract all DOIs as JSON array" is better than "get DOIs"
- **One transformation per call:** Break complex multi-step transforms into separate calls.


