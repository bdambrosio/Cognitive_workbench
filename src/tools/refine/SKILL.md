---
name: refine
description: Edit existing Note content according to natural language instruction (does not add new content)
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

Universal LLM-based transformation tool. **Edits existing Note content** according to natural language instructions.

## Important: Refine Only Edits, Does Not Add

- ✅ **Edits existing text**: Refine transforms the content of the target Note based on your instruction
- ❌ **Does NOT add new text**: Refine will not add information that isn't already present in the target Note
- ❌ **Does NOT create content**: For creating new content from scratch, use `generate-note` instead

## Input
- `target`: Note to transform (REQUIRED - must exist and contain content)
- `args.instruction`: Transformation instruction (REQUIRED, e.g., "extract schema", "convert to bullet list", "rewrite in formal tone")

## Output
Returns Note containing the edited/transformed content based on the instruction.

## Usage
```json
{"type":"refine","target":"$data","args":{"instruction":"extract schema as JSON"},"out":"$schema","expect":"should extract schema as JSON"}
```

## Examples
- ✅ "Extract the schema from this JSON" - edits to show only schema
- ✅ "Convert this to a bullet list" - edits formatting
- ✅ "Rewrite in formal tone" - edits style
- ❌ "Add a summary paragraph" - will NOT add new content (use generate-note instead)
- ❌ "Create a response to the user" - will NOT create new content (use generate-note instead)

**Note:** Prefer specialized tools (as-json, extract-entities) when available - they're faster and cheaper. For creating new content, use `generate-note`.


