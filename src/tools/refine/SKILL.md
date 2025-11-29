---
name: refine
description: Extract or transform information from unstructured text content using LLM. Use for extracting facts, entities, or specific information from text. Does not add new content.
type: python
trusted: true
flattens_collections: true
examples:
  - '{"type":"refine","target":"$bio","instruction":"extract the nationality","out":"$nationality"}'
  - '{"type":"refine","target":"$article","instruction":"extract the main argument","out":"$argument"}'
  - '{"type":"refine","target":"$data","instruction":"convert to bullet list","out":"$bullets"}'
---

# Transform Note

Universal LLM-based transformation tool. **Edits existing Note content** according to natural language instructions.

## Important: Refine Only Edits, Does Not Add

- ✅ **Edits existing text**: Refine transforms the content of the target Note based on your instruction
- ❌ **Does NOT add new text**: Refine will not add information that isn't already present in the target Note
- ❌ **Does NOT create content**: For creating new content from scratch, use `generate-note` instead

## Input
- `target`: Note to transform (REQUIRED - must exist and contain content)
- `instruction`: Transformation instruction (REQUIRED, e.g., "extract schema", "convert to bullet list", "rewrite in formal tone")

## Output
Returns Note containing the edited/transformed content based on the instruction.

## Usage
```json
{"type":"refine","target":"$data","instruction":"extract schema as JSON","out":"$schema"}
```

## Examples
- ✅ "Extract the schema from this JSON" - edits to show only schema
- ✅ "Convert this to a bullet list" - edits formatting
- ✅ "Rewrite in formal tone" - edits style
- ❌ "Add a summary paragraph" - will NOT add new content (use generate-note instead)
- ❌ "Create a response to the user" - will NOT create new content (use generate-note instead)

**Note:** Prefer specialized tools (as-json, extract-entities) when available - they're faster and cheaper. For creating new content, use `generate-note`.


