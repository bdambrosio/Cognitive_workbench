---
name: refine
type: python
flattens_collections: true
description: "Extract or transform information from unstructured or semi-structured text using an LLM. Edits existing content only; does not add new information."
---

# refine

LLM-based transformation for editing or extracting information from existing text. Operates on a single Note and produces a new Note derived solely from the input.

## Input

- `target`: Note (variable or ID) — MUST be a single Note
- `instruction`: String describing extraction or transformation
- `out`: Variable name for resulting Note

## Output

Success (`status: "success"`):
- `value`: Transformed or extracted content

Failure (`status: "failed"`):
- `reason`: `"instruction parameter required"` | `"input_value parameter required"` | `"llm_generate_failed"`

## Invariants

- Output content is derived **only from existing input text**
- No new information is introduced
- Empty result may indicate: no extractable structure, incorrect input shape, or instruction mismatch

## Planning Notes

**Use `refine` when:**
- Extracting facts or fields from a single document
- Transforming text format (e.g., bullets, JSON)
- Normalizing or rewriting existing content

**Do NOT use `refine` when:**
- Target is a Collection → use `map(refine)` or `flatten` → `refine`
- Combining multiple documents → use `flatten` → `refine`
- Selecting or filtering items → use `filter-collection` / `filter-structured`
- Extracting structured fields → use `project` / `pluck`
- Creating new content → use `generate-note`

## Anti-Patterns

- ❌ `refine(target=$collection)`
- ❌ `refine(target=$note, instruction="add summary")` — adds new info
- ❌ Treating empty result as "data missing"

## Example

```json
{"type":"refine","target":"$data","instruction":"extract schema as JSON","out":"$schema"}
```
