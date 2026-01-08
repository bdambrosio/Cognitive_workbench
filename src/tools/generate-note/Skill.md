---
name: generate-note
type: python
description: "Generate new text or code content from scratch using natural language prompt via LLM. Use to collect thoughts or generate a coherent statement on a topic"
---

# generate-note

LLM-based content generation. Creates new text or code from scratch using natural language instructions.

## Input

- `prompt`: Generation instruction (required)
- `style`: `"code"` or `"text"` (optional, default: `"text"`)
- `context`: Optional context - Note ID, Collection ID, or plain text string (NOT arrays)

## Output

Success (`status: "success"`):
- `value`: Generated content (text or code)

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- **text** (default): Generates prose, summaries, explanations (temperature=0.7)
- **code**: Generates code with stricter formatting (temperature=0.2)
- Context is auto-resolved: Collection IDs concatenate all Notes, Note IDs fetch content

## Planning Notes

- Use for creating NEW content from scratch
- Use `refine` for editing/transforming EXISTING Note content
- Do NOT pass arrays as context - create a Collection first

## Examples

```json
{"type":"generate-note","prompt":"Write a Python fibonacci function","style":"code","out":"$fib"}
{"type":"generate-note","prompt":"Summarize quantum computing basics","out":"$summary"}
{"type":"generate-note","prompt":"Refactor to async/await","style":"code","context":"$existing_code","out":"$refactored"}
```
