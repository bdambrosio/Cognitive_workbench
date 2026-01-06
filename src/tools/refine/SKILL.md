---
name: refine
type: python
flattens_collections: true
description: "Extract or transform information from unstructured text content using LLM. Use for extracting facts, entities, or specific information from text. Does not add new content"
---

# Refine Tool

Universal LLM-based transformation tool. Edits existing Note content according to natural language instructions.

## Purpose

Transform the content of an existing Note based on instruction. Does NOT add new content - only edits/transforms what's already present. For creating new content, use `generate-note` instead.

## Input

- `target`: Note to transform (required - must exist and contain content)
- `instruction`: Transformation instruction (required, e.g., "extract schema", "convert to bullet list", "rewrite in formal tone")

## Output

Returns Note containing the edited/transformed content based on the instruction.

## Behavior & Performance

- Only edits existing text - does not add information that isn't already present
- Does not create content - for creating new content, use `generate-note`
- Prefer specialized tools (as-json, extract-entities) when available - they're faster and cheaper

## Guidelines

- Use for extracting facts, entities, or transforming existing content
- Examples that work: "Extract the schema from this JSON", "Convert this to a bullet list", "Rewrite in formal tone"
- Examples that don't work: "Add a summary paragraph" (will NOT add new content), "Create a response to the user" (will NOT create new content)
- For creating new content, use `generate-note` instead

## Usage Examples

Extract schema:
```json
{"type":"refine","target":"$data","instruction":"extract schema as JSON","out":"$schema"}
```

Transform format:
```json
{"type":"refine","target":"$text","instruction":"convert to bullet list","out":"$bullets"}
```

Extract information:
```json
{"type":"refine","target":"$bio","instruction":"extract the nationality","out":"$nationality"}
```
