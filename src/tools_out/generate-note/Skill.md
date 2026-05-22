---
name: generate-note
type: python
description: "Generate new text or code content from scratch using natural language prompt via LLM. Creates content from the LLM's own knowledge — no source documents."
---

# generate-note

LLM-based content generation. Creates new text or code from scratch using natural language instructions.
No source documents — for generation from source material, use `synthesize` instead.

## Input

- `prompt`: Generation instruction (required)
- `style`: `"code"` or `"text"` (optional, default: `"text"`)
- `target_tokens`: Integer (optional). Target output length in tokens. Overrides default max_tokens when provided via OUTPUT GUIDANCE.

## Output

Success (`status: "success"`):
- `value`: Generated content (text or code)

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- **text** (default): Generates prose, summaries, explanations (temperature=0.7)
- **code**: Generates code with stricter formatting (temperature=0.2)

## Planning Notes

- Use for creating NEW content from scratch (no source material)
- Use `extract` for deriving content from a single Note
- Use `synthesize` for integrating content from multiple documents
- Do NOT pass context — use `synthesize` with source Collections instead

## Examples

```json
{"type":"generate-note","prompt":"Write a Python fibonacci function","style":"code","out":"$fib"}
{"type":"generate-note","prompt":"Explain quantum computing basics","out":"$explanation"}
```
