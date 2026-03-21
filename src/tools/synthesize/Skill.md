---
name: synthesize
type: python
flattens_collections: true
description: "Integrate content across multiple documents to produce new understanding. Use for cross-document synthesis, comparison, and reporting from Collections."
schema_hint:
  target: "$variable (Collection or Note, required)"
  other: "$variable (second input for comparison, optional)"
  focus: "string (what to attend to, optional)"
  format: "string (narrative|comparison|executive|technical|comprehensive, default: narrative)"
  instruction: "string (free-form override, optional)"
  out: "$variable (optional)"
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
  - `"narrative"`: Prose synthesis
  - `"comparison"`: Structured JSON with similarity_score, shared_themes,
    unique_to_first, unique_to_second, contradictions
  - `"executive"`: High-level overview, 300-500 words
  - `"technical"`: Balanced detail with compression
  - `"comprehensive"`: Low compression, preserves nuance
- `compression_ratio`: Optional float (default: 3.0). Controls output length relative
  to input. Only meaningful for narrative/technical/comprehensive formats.
- `instruction`: Optional free-form instruction for specialized synthesis tasks.
  Overrides format-specific defaults when provided.
- `target_tokens`: Integer (optional). Target output length in tokens. Overrides
  computed target length when provided via OUTPUT GUIDANCE.
- `out`: Variable name for resulting Note

## Output

Success (`status: "success"`):
- `value`: Synthesized content as a new Note.
  - For `format="narrative"` / `"executive"` / `"technical"` / `"comprehensive"`: prose text
  - For `format="comparison"`: JSON string with structure:
    `{"similarity_score": 0.75, "shared_themes": [...], "unique_to_first": [...], "unique_to_second": [...], "contradictions": [...], "relationship": "...", "summary": "..."}`

Failure (`status: "failed"`):
- `reason`: `"target parameter required"` | `"target is empty"` |
  `"llm_generate_failed"` | `"comparison format requires 'other' parameter"`

## Behavior

- Flattens Collection items, applies focus filtering if `focus` provided
- Uses hierarchical map-reduce for long inputs (auto-chunking at ~16k chars)
- Focus filtering applies relevance threshold — chunks below threshold excluded
- When `other` is provided: both inputs are processed, then compared/integrated
- When `format="comparison"` and `other` is NOT provided: fails with error
- Output may include observations, patterns, and integrative conclusions not
  present in any single input document — this is by design

## Planning Notes

**Use `synthesize` when:**
- Identifying themes and trends across a Collection of papers
- Comparing two documents or Collections
- Producing a report from multiple sources
- Aggregating per-item extractions into a coherent narrative

**Do NOT use `synthesize` when:**
- Extracting, formatting, or reporting from a single document → use `extract`
  (even if the goal says "summarize", "report", or "present" — if there is only ONE source, use `extract`)
- Creating content with no source material → use `generate-note`
- Filtering or selecting items → use `filter-structured` or `filter-semantic`
- Structural operations on Collections → use `project`, `sort`, `head`, etc.

**Standard analytical pipeline:**
1. `map(extract)` — per-item fact extraction
2. `synthesize` — cross-item integration

**For comparison:** use `format="comparison"` with `other=` (requires two inputs)

## Examples

```json
{"type":"synthesize","target":"$papers","focus":"significant architectural improvements","format":"technical","out":"$report"}
{"type":"synthesize","target":"$paper_a","other":"$paper_b","format":"comparison","instruction":"focus on methodology differences","out":"$comparison"}
{"type":"synthesize","target":"$innovations","focus":"dominant trends","format":"executive","out":"$executive_summary"}
{"type":"synthesize","target":"$extracted_methods","focus":"how attention mechanisms have evolved","format":"narrative","compression_ratio":2.0,"out":"$attention_report"}
```
