---
name: transform-note
description: Transform Note content using natural language command via LLM
type: python
trusted: true
parameters:
  - name: command
    type: string
    description: Natural language instruction for transformation
  - name: model
    type: string
    description: Optional LLM model (default uses fast model, can specify 'sonnet' for premium)
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

- `command`: Natural language instruction (e.g., "extract schema", "identify citations", "convert to bullet list")
- `target`: Note content to transform
- `model` (optional): LLM model selection - "fast" (default), "sonnet" (premium)

## Output

Returns Note containing transformed content according to command.

## Usage Examples

Extract schema:
```json
{"type":"transform-note","target":"$data","args":{"command":"extract schema"},"out":"schema"}
```

Extract citations:
```json
{"type":"transform-note","target":"$paper","args":{"command":"list all citations in JSON format"},"out":"citations"}
```

Use premium model:
```json
{"type":"transform-note","target":"$complex_data","args":{"command":"analyze structure","model":"sonnet"},"out":"analysis"}
```

## Guidelines

- **Cost aware:** Uses fast model by default. Reserve "sonnet" for complex reasoning.
- **Prefer specialized tools when available:** Use parse-json, extract-entities, etc. when they exist - they're faster and cheaper.
- **Be specific:** Clear commands yield better results. "Extract all DOIs as JSON array" > "get DOIs"
- **One transformation per call:** Break complex multi-step transforms into separate calls.

## Performance

- Fast model: ~1-2 seconds, low cost
- Sonnet model: ~3-5 seconds, higher cost but better reasoning
- All calls are logged for usage analysis

