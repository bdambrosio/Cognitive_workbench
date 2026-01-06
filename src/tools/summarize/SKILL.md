---
name: summarize
type: python
flattens_collections: true
description: "Compress information with focus-aware adaptive compression and styling. Use to extract and prepare content in presentable form"
---

# Summarize Tool

Extract essential information with focus-aware compression and adaptive styling. Automatically filters content by relevance and scales output appropriately.

## Purpose

Transform verbose content into concise summaries using information-theoretic compression. Compression ratio applies to relevant content (after focus filtering), not raw input size.

## Input

- `target`: Note ID, Collection ID, or variable containing content to summarize
- `focus`: Optional topic to guide summarization (string)
- `style`: Optional output style - "executive", "technical" (default), or "comprehensive" (string)
- `compression_ratio`: Optional compression factor (float, default: 3.0)

## Output

Returns Note containing summarized content. Output length varies by style and compression ratio.

## Behavior & Performance

- Auto-Chunking: Texts >16k characters are split into boundary-aware chunks
- Focus Filtering: When focus provided, applies leaky relevance filter (40% threshold - lenient). Compression ratio applies to focused content only
- Hierarchical Summarization: Map-reduce pattern for long documents
- Style options:
  - "executive": High-level overview, 300-500 words max, focuses on key findings
  - "technical" (default): Balanced detail preservation, uses compression_ratio directly
  - "comprehensive": Low compression (2:1), preserves nuance and technical details
- Compression ratio: Applied to effective content size (after focus filtering if focus provided). 3.0 = reduce to 33% of input

## Guidelines

- Focus dramatically improves signal-to-noise ratio in output
- Predicate phrasing: Since tool checks chunks in isolation, phrase predicates to detect presence rather than global summary
- Cost: Every chunk requires an LLM call
- Fallbacks: Returns content with warning if focus yields 0 chunks
- Monitor inclusion_pct: if consistently >75%, focus may be too broad

## Usage Examples

Default (technical, 3:1 compression):
```json
{"type":"summarize","target":"$papers","out":"$summary"}
```

Executive brief with focus:
```json
{"type":"summarize","target":"$reports","focus":"cost analysis","style":"executive","out":"$brief"}
```

Detailed focused analysis:
```json
{"type":"summarize","target":"$papers","focus":"transformer attention","compression_ratio":2.5,"out":"$detailed"}
```

Comprehensive review:
```json
{"type":"summarize","target":"$document","style":"comprehensive","out":"$full_analysis"}
```
