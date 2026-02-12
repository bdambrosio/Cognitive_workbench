# Semantic Operators: `extract` and `synthesize`

## Design Rationale

The infospace primitive catalog has clean set semantics at the structural level:
`union`, `intersection`, `difference`, `filter-structured`, `sort`, `project`, `pluck`.
Each does one thing, on one type, with predictable output.

The semantic operators (`refine`, `summarize`, `relate`) blur two orthogonal axes:
scope (Note vs Collection) and operation (extract vs compress vs compare vs synthesize).
This causes tool selection confusion: the model reaches for `relate` when it needs
`summarize`, uses `refine` when it needs `generate-note`, etc.

Two operators, cleanly separated by the boundary they respect:

- **`extract`**: Note → Note. Derives content from a single document. Never crosses
  the document boundary.
- **`synthesize`**: Collection → Note (or Note + Note → Note). Integrates across
  documents to produce new understanding. Always crosses the document boundary.

Everything the old tools did maps to one of these two.


## Tool Replacement Map

| Old tool | New tool | Notes |
|----------|----------|-------|
| `refine` | `extract` | Rename. Tighten contract. |
| `summarize` on single Note | `extract` | Compression is extraction (selecting what matters) |
| `summarize` on Collection | `synthesize` | Cross-document integration |
| `relate` | `synthesize` with `format="comparison"` | Comparison is synthesis of two inputs |
| `generate-note` with `context` | `synthesize` | Synthesis from source material |
| `generate-note` without `context` | `generate-note` (unchanged) | Creation from nothing is a different category |
| `map(refine)` | `map(extract)` | Unchanged pattern, new name |


---


# extract

Derive content from a single Note's text via LLM-guided extraction or transformation.
Output is grounded entirely in the input — no new information is introduced, no
cross-document synthesis.

## Input

- `target`: Note (variable, ID, or name) — MUST be a single Note
- `instruction`: String describing what to extract or how to transform (required)
- `out`: Variable name for resulting Note

## Output

Success (`status: "success"`):
- `value`: Extracted or transformed content as a new Note

Failure (`status: "failed"`):
- `reason`: `"instruction parameter required"` | `"target parameter required"` |
  `"target is empty"` | `"llm_generate_failed"`

## Behavior

- Takes the Note's text content and the instruction, calls the LLM, returns derived text
- Output is constrained to information present in the input
- Does NOT add facts, context, or knowledge from outside the input
- Does NOT operate on Collections — use `map(extract)` for per-item extraction
- Handles long inputs via chunking if needed (same as current refine)

## What `extract` Is For

- Pulling specific facts or fields from a document
  ("extract the key architectural innovation as one sentence")
- Reshaping text format
  ("convert this abstract to bullet points", "rewrite as JSON with fields: method, result")
- Compressing a single document
  ("summarize this paper in 3 sentences", "extract only the methodology section")
- Normalizing or cleaning text
  ("remove citation markers", "standardize author name format")

## What `extract` Is NOT For

- Integrating across multiple documents → use `synthesize`
- Creating content from scratch → use `generate-note`
- Filtering items in a Collection → use `filter-structured` or `filter-semantic`
- Accessing structured metadata fields → use `project` or `pluck`

## Anti-Patterns

❌ `extract(target=$collection)` → Must be a single Note. Use `map(extract)` for Collections.
❌ `extract(target=$note, instruction="add a conclusion")` → Adds new content. Use `generate-note`.
❌ `extract(target=$note, instruction="compare with other papers")` → Cross-document. Use `synthesize`.
❌ Using extract to produce content not derivable from the input text.

## Planning Notes

- The most common pattern for analyzing a Collection is `map(extract)` to pull
  per-item facts, then `synthesize` to integrate across items.
- For compression of a single document, `extract` with a compression-oriented
  instruction replaces the old `summarize` on a single Note.
- If you need structured output (JSON), say so in the instruction AND verify
  the output — LLMs don't always produce valid JSON.

## Examples

```json
{"type": "extract", "target": "$paper", "instruction": "Extract the key architectural innovation described in this paper as one sentence.", "out": "$innovation"}

{"type": "extract", "target": "$abstract", "instruction": "Compress to 2-3 sentences retaining methodology and results.", "out": "$compressed"}

{"type": "extract", "target": "$paper", "instruction": "Extract as JSON: {\"method\": ..., \"result\": ..., \"limitation\": ...}", "out": "$structured"}

{"type": "map", "target": "$papers", "operation": "extract", "instruction": "State the main contribution in one sentence.", "out": "$contributions"}
```


---


# synthesize

Integrate content across multiple documents (or between two documents) to produce
new understanding. Always crosses the document boundary — this is the tool for
combining, comparing, and generating insight from a Collection.

## Input

- `target`: Collection (variable or ID) — the primary input. May also be a single Note
  when `other` is provided for two-input comparison.
- `other`: Optional second Note or Collection for explicit comparison. When provided,
  the operation compares target against other.
- `focus`: Optional string guiding what to attend to
  ("architectural improvements", "methodology differences", "emerging trends")
- `format`: Output format (optional, default: `"narrative"`):
  - `"narrative"`: Prose synthesis (replaces old `summarize` on Collection)
  - `"comparison"`: Structured JSON with similarity_score, shared_themes,
    unique_to_first, unique_to_second, contradictions (replaces old `relate`)
  - `"executive"`: High-level overview, 300-500 words
  - `"technical"`: Balanced detail with compression
  - `"comprehensive"`: Low compression, preserves nuance
- `compression_ratio`: Optional float (default: 3.0). Controls output length relative
  to input. Only meaningful for narrative/technical/comprehensive formats.
- `instruction`: Optional free-form instruction for specialized synthesis tasks.
  Overrides format-specific defaults when provided.
- `out`: Variable name for resulting Note

## Output

Success (`status: "success"`):
- `value`: Synthesized content as a new Note.
  - For `format="narrative"` / `"executive"` / `"technical"` / `"comprehensive"`: prose text
  - For `format="comparison"`: JSON string with structure:
    ```json
    {
      "similarity_score": 0.75,
      "shared_themes": ["theme1", "theme2"],
      "unique_to_first": ["aspect1"],
      "unique_to_second": ["aspect2"],
      "contradictions": [{"aspect": "X", "first": "claim A", "second": "claim B"}],
      "relationship": "complements",
      "summary": "Narrative comparison summary..."
    }
    ```

Failure (`status: "failed"`):
- `reason`: `"target parameter required"` | `"target is empty"` |
  `"llm_generate_failed"` | `"other required for comparison format"`

## Behavior

- Flattens Collection items, applies focus filtering if `focus` provided
- Uses hierarchical map-reduce for long inputs (auto-chunking at ~16k chars)
- Focus filtering applies relevance threshold (40%) — chunks below threshold excluded
- When `other` is provided: both inputs are processed, then compared/integrated
- When `format="comparison"` and `other` is NOT provided: fails with error
  (comparison requires two distinct inputs)
- Output may include observations, patterns, and integrative conclusions that are
  not present in any single input document — this is by design. Synthesis creates
  new understanding from the combination.

## What `synthesize` Is For

- Identifying themes and trends across a Collection of papers
  ("what are the dominant architectural directions?")
- Comparing two documents or Collections
  ("how do these two approaches differ?")
- Producing a report from multiple sources
  ("write a technical summary of these findings")
- Aggregating per-item extractions into a coherent narrative
  (after `map(extract)` produces per-paper innovations, `synthesize` finds patterns)

## What `synthesize` Is NOT For

- Extracting content from a single document → use `extract`
- Creating content with no source material → use `generate-note`
- Filtering or selecting items → use `filter-structured` or `filter-semantic`
- Structural operations on Collections → use `project`, `sort`, `head`, etc.

## Anti-Patterns

❌ `synthesize(target=$note)` without `other` → Single-Note operations use `extract`.
   Exception: a Note containing a long document that needs focus-aware compression
   may use `synthesize`, but `extract` is preferred.
❌ `synthesize(target=$X, other=$X)` → Comparing something to itself is meaningless.
   Target and other MUST be distinct.
❌ `synthesize(target=$papers, format="comparison")` without `other` → Comparison
   requires two inputs. Use `format="narrative"` for single-Collection synthesis.
❌ Using synthesize on a single item when extract would suffice — synthesize is
   heavier (flattening, chunking, map-reduce) and should be reserved for multi-item work.

## Planning Notes

- The standard analytical pipeline is:
  1. `map(extract)` — per-item fact extraction
  2. `synthesize` — cross-item integration
  This two-phase pattern replaces most uses of the old `summarize` + `relate` combination.

- For simple Collection summarization, `synthesize` alone (without prior `map(extract)`)
  is often sufficient. Use the two-phase pattern when you need per-item structure first.

- `focus` dramatically improves signal-to-noise. Prefer specific focus strings
  ("efficiency improvements in attention mechanisms") over vague ones ("improvements").

- `format="comparison"` with `other` replaces the old `relate` tool entirely.
  The structured output (similarity_score, shared_themes, etc.) is identical.

- When synthesizing after `map(extract)`, the intermediate Collection should contain
  the extracted content, not raw paper text. This keeps the synthesis focused on
  what was extracted rather than re-reading all source material.

## Examples

```json
{"type": "synthesize", "target": "$papers", "focus": "significant architectural improvements and emerging directions", "format": "technical", "out": "$report"}

{"type": "synthesize", "target": "$paper_a", "other": "$paper_b", "format": "comparison", "instruction": "focus on methodology differences", "out": "$comparison"}

{"type": "synthesize", "target": "$innovations", "focus": "dominant trends", "format": "executive", "out": "$executive_summary"}

{"type": "synthesize", "target": "$extracted_methods", "focus": "how attention mechanisms have evolved", "format": "narrative", "compression_ratio": 2.0, "out": "$attention_report"}
```


---


# Catalog Entry Updates

## Old entries to remove:

```
- summarize, relate: Note, Collection;  Generate new content
- refine: Note;  Transform Note content  (under "as-json, refine, coerce")
```

## New entries:

```
- extract: Note;  Derive content from a single Note (extraction, compression, reshaping)
- synthesize: Collection;  Integrate across documents (themes, comparison, reporting)
```

## Updated compatibility table:

```
Operation_name: applicable to;  Purpose
 - extract: Note;  LLM-guided extraction, compression, or transformation of single-Note content
 - synthesize: Collection;  Cross-document integration, comparison, and reporting
```

## Updated efficiency heuristics:

```
- Use extract directly on Notes for single-item work
- Use map(extract) for per-item extraction across a Collection
- Use synthesize for cross-item integration, comparison, or reporting
- Two-phase pattern: map(extract) → synthesize (extract per item, then integrate)
- Single-phase: synthesize with focus (when per-item extraction isn't needed)
```

## Updated critical workflows:

```
- semantic-scholar → synthesize (with focus) — direct Collection analysis
- semantic-scholar → map(extract) → synthesize — two-phase with per-item extraction
- semantic-scholar → filter-structured → synthesize — filtered then analyzed
- For comparison: synthesize with format="comparison" and other= (requires two inputs)
```


---


# Implementation Notes

## `extract` implementation

Rename `refine.py` → `extract.py`. Minimal code changes:
- Update tool name registration
- Tighten input validation: reject Collection targets with clear error message
  ("extract operates on single Notes; use map(extract) for Collections")
- Update docstring/contract
- Absorb single-Note summarize case: if the old `summarize` was called on a
  single Note, the instruction becomes the `focus` parameter mapped to an
  extraction instruction like "Compress with focus on: {focus}"

## `synthesize` implementation

New file, or merge `summarize.py` + `relate.py`:
- Input handling: flatten Collection, apply focus filter (from current summarize)
- When `other` provided: flatten both, pass to comparison template (from current relate)
- When `format="comparison"`: require `other`, use structured output template
- When `format="comparison"` without `other`: fail with clear error
- When `target == other`: fail with clear error ("comparison requires distinct inputs")
- Map-reduce for long inputs (from current summarize)
- Output formatting based on `format` parameter

## `generate-note` changes

Remove `context` parameter. If the user wants to generate from source material,
they should use `synthesize`. `generate-note` becomes purely "create from scratch
using the LLM's own knowledge" — no source documents.

This cleanly separates: `extract` (from one doc), `synthesize` (from many docs),
`generate-note` (from nothing).

## Migration

- Existing plans using `refine` → alias to `extract` for backward compat
- Existing plans using `summarize` → route to `extract` (single Note) or
  `synthesize` (Collection) based on target type
- Existing plans using `relate` → route to `synthesize` with `format="comparison"`
- Deprecation warnings for old names, remove after validation
