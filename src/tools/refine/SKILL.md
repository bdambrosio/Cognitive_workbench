---
name: refine
type: python
flattens_collections: true
description: "Extract or transform information from unstructured or semi-structured text using an LLM. Edits existing content only; does not add new information."
---

# Refine Tool

Universal LLM-based transformation tool for **editing or extracting information from existing text**.

`refine` operates on a **single Note** and produces a **new Note** whose content is derived solely from the input. It does **not** create new facts or synthesize information.

---

## INPUT CONTRACT

- `target`: **Note** (variable or ID)  
  - MUST be a single Note (not a Collection)
- `instruction`: **String** describing extraction or transformation
- `out`: Variable name for resulting Note

### REQUIREMENTS
- Target content SHOULD be unstructured or semi-structured text
- Instruction MUST specify *what to extract* or *how to transform*

### NOT SUPPORTED
- ❌ `refine(Collection)`
  - Use `map(refine)` if each Note is independent
  - Or `flatten` → `refine` if structure is distributed
- ❌ Notes containing multiple independent documents  
  - Use `split` first

---

## OUTPUT

- Returns a **Note** containing transformed or extracted content
- Output content is derived **only from existing input text**
- No new information is introduced

---

## FAILURE SEMANTICS

An **empty result** may indicate:
- No extractable structure matching the instruction
- Incorrect input shape (Collection instead of Note)
- Instruction too specific or incompatible with content

**Empty result does NOT imply information is absent globally.**  
It often indicates a contract or instruction mismatch.

Actual failures include:
- Invalid target type
- Missing instruction
- LLM execution error

---

## USAGE GUIDANCE

### Use `refine` when:
- Extracting facts or fields from a single document
- Transforming text format (e.g., bullets, JSON)
- Normalizing or rewriting existing content

### Do NOT use `refine` when:
- Target is a Collection → use `map(refine)` or `flatten` → `refine`
- Combining multiple documents → use `flatten` → `refine`
- Selecting or filtering items → use `filter-collection` / `filter-structured`
- Extracting structured fields → use `project` / `pluck`
- Creating new content → use `generate-note`

---

## REPRESENTATION INVARIANTS

- Collection ≠ Note
- Use `map(refine)` when each Note is independently meaningful
- Use `flatten` → `refine` when structure is distributed across text
- Prefer specialized tools (`as-json`, `extract-entities`) when available

---

## ANTI-PATTERNS

- ❌ `refine(target=$collection)`
- ❌ `refine(target=$note, instruction="add summary")`
- ❌ Treating empty result as “data missing”
- ❌ Using `refine` for relational or field-based selection

---

## EXAMPLES

Extract schema:
```json
{"type":"refine","target":"$data","instruction":"extract schema as JSON","out":"$schema"}
