---
name: summarize
type: python
flattens_collections: true
description: "Compress information with focus-aware adaptive compression and styling. Use to extract and prepare content in presentable form"
---

# summarize

Extract essential information with focus-aware compression and adaptive styling.

## Input

- `target`: Note ID, Collection ID, or variable containing content
- `focus`: Optional topic to guide summarization (string)
- `style`: `"executive"` | `"technical"` (default) | `"comprehensive"` (string)
- `compression_ratio`: Optional compression factor (float, default: 3.0)

## Output

Success (`status: "success"`):
- `value`: Summarized content

## Behavior

- **Auto-Chunking**: Texts >16k chars are split into boundary-aware chunks
- **Focus Filtering**: When focus provided, applies relevance filter (40% threshold)
- **Hierarchical Summarization**: Map-reduce for long documents
- **Styles**:
  - `executive`: High-level overview, 300-500 words max
  - `technical` (default): Balanced detail, uses compression_ratio directly
  - `comprehensive`: Low compression (2:1), preserves nuance
- Compression ratio applies to effective content size (after focus filtering)

## Planning Notes

- Focus dramatically improves signal-to-noise ratio
- Phrase focus to detect presence (chunks evaluated in isolation)
- If inclusion_pct consistently >75%, focus may be too broad

## Examples

```json
{"type":"summarize","target":"$papers","out":"$summary"}
{"type":"summarize","target":"$reports","focus":"cost analysis","style":"executive","out":"$brief"}
{"type":"summarize","target":"$papers","focus":"transformer attention","compression_ratio":2.5,"out":"$detailed"}
```
