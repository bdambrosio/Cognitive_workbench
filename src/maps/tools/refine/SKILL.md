---
name: refine
description: Transform Note content using natural language instruction via LLM
type: python
trusted: true
flattens_collections: true
parameters:
  - name: instruction
    type: string
    description: Natural language instruction for transformation (REQUIRED)
examples:
  - '{"type":"refine","target":"$data","args":{"instruction":"extract schema as JSON"},"out":"$schema","expect":"should extract JSON schema"}'
  - '{"type":"refine","target":"$paper","args":{"instruction":"list all citations in JSON format"},"out":"$citations","expect":"should find citation list"}'
---

# Transform Note

Universal LLM-based transformation tool. Applies natural language instructions to Note content.

## Input
- `target`: Note to transform
- `args.instruction`: Transformation instruction (REQUIRED, e.g., "extract schema", "convert to bullet list")

## Output
Returns Note containing transformed content.

## Usage
```json
{"type":"refine","target":"$data","args":{"instruction":"extract schema as JSON"},"out":"$schema","expect":"should extract JSON schema"}
```

**Note:** Prefer specialized tools (as-json, extract-entities) when available - they're faster and cheaper.


