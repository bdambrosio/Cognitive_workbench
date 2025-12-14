---
name: generate-note
description: Generate new text or code content from scratch using natural language prompt via LLM
type: python
trusted: true
schema_hint:
  prompt: "string (generation instruction)"
  style: "string (optional, 'code' or 'text', default 'text')"
  context: "$variable or Note ID or Collection ID (optional)"
  out: "$variable"
examples:
  - '{"type":"generate-note","prompt":"Write a Python function to calculate fibonacci numbers","out":"$code_note"}'
  - '{"type":"generate-note","prompt":"Write a summary of quantum computing","style":"text","out":"$summary_note"}'
  - '{"type":"generate-note","prompt":"Refactor this code to use async/await","style":"code","context":"$existing_code","out":"$refactored_code"}'
---

# Generate Note

LLM-based content generation tool. Creates new text or code from scratch using natural language instructions.

## Input
- `prompt`: Generation instruction (REQUIRED, e.g., "Write a tic-tac-toe game", "Explain quantum entanglement")
- `style`: "code" or "text" (optional, default: "text")
- `context`: Optional context - can be:
  - Collection ID (e.g., "$context_collection"): Concatenates all Notes in Collection
  - Note ID (e.g., "$background_note"): Uses Note content as context
  - Plain text string: Used directly as context
  - ❌ **Arrays/Lists are NOT allowed**: Do not pass `["$note1", "$note2"]`. You MUST create a Collection first.

## Output
Returns Note containing generated content (text or code).

## Usage

Generate code:
```json
{"type":"generate-note","prompt":"Write a Python function to calculate fibonacci numbers","style":"code","out":"$fib_code"}
```

Generate text:
```json
{"type":"generate-note","prompt":"Write a summary of quantum computing","out":"$summary"}
```

Generate with context Note:
```json
{"type":"generate-note","prompt":"Refactor this code to use async/await","style":"code","context":"$existing_code","out":"$refactored"}
```

Generate with context Collection:
```json
{"type":"generate-note","prompt":"Write a response based on conversation history","context":"$context_collection","out":"$response"}
```

## Differences from refine
- **generate-note**: Creates NEW content from scratch, following prompt. No target Note required, however, context information can be provided)
- **refine**: Transforms EXISTING Note content (requires target Note)

## Context Assembly Pattern (RAG)
When generating content that needs background (conversation history, TOM model, documents):
1. **Search/load** relevant Notes: `search-notes`, `load`, etc.
2. **Assemble** into Collection: `create-collection` with found Notes
3. **Generate** with context: `generate-note` with `context` pointing to Collection

This makes context assembly explicit and inspectable in the plan.

## Style Options
- `"text"` (default): Generates prose, summaries, explanations (temperature=0.7)
- `"code"`: Generates code with stricter formatting (temperature=0.2)

**Note:** For iterative refinement of generated content, use `refine` tool on the generated Note.

