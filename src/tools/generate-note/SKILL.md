---
name: generate-note
type: python
description: "Generate new text or code content from scratch using natural language prompt via LLM. Use to collect thoughts or generate a coherent statement on a topic"
---

# Generate Note Tool

LLM-based content generation tool. Creates new text or code from scratch using natural language instructions.

## Purpose

Create new content from scratch. For editing existing content, use `refine` instead. For creating content, use `generate-note`.

## Input

- `prompt`: Generation instruction (required, e.g., "Write a tic-tac-toe game", "Explain quantum entanglement")
- `style`: "code" or "text" (optional, default: "text")
- `context`: Optional context - Note ID, Collection ID, or plain text string. Arrays/Lists are NOT allowed - create a Collection first

## Output

Returns Note containing generated content (text or code).

## Behavior & Performance

- Style options:
  - "text" (default): Generates prose, summaries, explanations (temperature=0.7)
  - "code": Generates code with stricter formatting (temperature=0.2)
- Context assembly: When generating content that needs background, search/load relevant Notes, assemble into Collection, then generate with context pointing to Collection

## Guidelines

- Use for creating NEW content from scratch
- Use `refine` for editing/transforming EXISTING Note content
- Context can be Collection ID (concatenates all Notes), Note ID (uses Note content), or plain text string
- Do NOT pass arrays/lists as context - create a Collection first
- For iterative refinement of generated content, use `refine` tool on the generated Note

## Usage Examples

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
