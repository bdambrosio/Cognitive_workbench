---
name: extract
type: python
flattens_collections: false
description: "Derive content from a single Note via LLM-guided extraction, compression, or transformation. Output is grounded entirely in the input — no new information is introduced."
schema_hint:
  target: "$variable (single Note, required)"
  instruction: "string (what to extract or how to transform, required)"
  out: "$variable (optional)"
---

# extract

Derive content from a single Note's text via LLM-guided extraction or transformation.
Output is grounded entirely in the input — no new information is introduced, no
cross-document synthesis.

## Input

- `target`: Note (variable, ID, or name) — MUST be a single Note
- `instruction`: String describing what to extract or how to transform (required)
- `target_tokens`: Integer (optional). Target output length in tokens. Overrides default max_tokens when provided via OUTPUT GUIDANCE.
- `out`: Variable name for resulting Note

## Output

Success (`status: "success"`):
- `value`: Extracted or transformed content as a new Note

Failure (`status: "failed"`):
- `reason`: `"instruction parameter required"` | `"target parameter required"` |
  `"target is empty"` | `"target must be a single Note, not a Collection; use map(extract) for Collections"` | `"llm_generate_failed"`

## Invariants

- Output content is derived **only from existing input text**
- No new information is introduced
- Does NOT operate on Collections — use `map(extract)` for per-item extraction

## Planning Notes

For **small, predictable structure** embedded in Note text (e.g. YAML frontmatter between `---` lines, key: value blocks with a fixed layout), **prefer handling it in Python** in the planner code step before calling **`extract`**. Use **`extract`** when the layout is unknown, messy, or you need semantic judgment over free-form prose.

**Use `extract` when:**
- Pulling specific facts or fields from a document
  ("extract the key architectural innovation as one sentence")
- Reshaping text format
  ("convert this abstract to bullet points", "rewrite as JSON with fields: method, result")
- Compressing a single document
  ("summarize this paper in 3 sentences", "extract only the methodology section")
- Formatting and reporting from a single source document
  ("format this weather data as a concise report", "summarize these findings and present as a briefing")
- Normalizing or cleaning text
  ("remove citation markers", "standardize author name format")

**Prefer `extract` over `synthesize` when:**
- There is only ONE source document — even if the goal says "summarize" or "report"
- Grounding is critical — extract never introduces new information
- The task is reformatting, not integrating across multiple documents

**Do NOT use `extract` when:**
- Extracting bibliography/references from a paper → use `extract-references` (GROBID-based, structured, deterministic). It accepts a Note ID from `semantic-scholar` and returns structured citation data (title, authors, year, venue). Far more reliable than LLM extraction for reference lists.
- Integrating across multiple documents → use `synthesize`
- Creating content from scratch → use `generate-note`
- Filtering items in a Collection → use `filter-structured` or `filter-semantic`
- Accessing structured metadata fields → use `project` or `pluck`

## Anti-Patterns

- ❌ `extract(target=$collection)` — Must be a single Note. Use `map(extract)` for Collections.
- ❌ `extract(target=$paper, instruction="extract citations/references")` — Use `extract-references` instead. It uses GROBID for structural parsing and returns structured data per citation.
- ❌ `extract(target=$note, instruction="add a conclusion")` — Adds new content. Use `generate-note`.
- ❌ `extract(target=$note, instruction="compare with other papers")` — Cross-document. Use `synthesize`.

## Examples

```json
{"type":"extract","target":"$paper","instruction":"Extract the key architectural innovation as one sentence.","out":"$innovation"}
{"type":"extract","target":"$abstract","instruction":"Compress to 2-3 sentences retaining methodology and results.","out":"$compressed"}
{"type":"extract","target":"$paper","instruction":"Extract as JSON: {\"method\": ..., \"result\": ..., \"limitation\": ...}","out":"$structured"}
{"type":"map","target":"$papers","operation":"extract","instruction":"State the main contribution in one sentence.","out":"$contributions"}
```
