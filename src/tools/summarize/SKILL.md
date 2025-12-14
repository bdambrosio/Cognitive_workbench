---
name: summarize
description: Compress information with focus-aware adaptive compression and styling
type: python
trusted: true
flattens_collections: true
schema_hint:
  target: "$variable or Note ID or Collection ID"
  focus: "string (optional, topic to guide summarization)"
  style: "string (optional, 'executive' or 'technical' or 'comprehensive', default 'technical')"
  compression_ratio: "float (optional, default 3.0)"
  out: "$variable"
examples:
  - '{"type":"summarize","target":"$doc","out":"$summary"}'
  - '{"type":"summarize","target":"$papers","focus":"attention mechanisms","style":"technical","out":"$summary"}'
  - '{"type":"summarize","target":"$reports","style":"executive","out":"$brief"}'
  - '{"type":"summarize","target":"$data","focus":"cost analysis","compression_ratio":2.0,"out":"$detailed"}'
---

# Summarize Content

Extract essential information with focus-aware compression and adaptive styling. Automatically filters content by relevance and scales output appropriately.

## Purpose

Transform verbose content into concise summaries using information-theoretic compression. Compression ratio applies to *relevant* content (after focus filtering), not raw input size.

## Input Format

Accepts:
- Plain text (single document or passage)
- Structured data (JSON/dict with fields to summarize)
- Collections (flattened and processed as unified content)

## Optional Parameters (All Optional)

- **focus** (string) - Topic to guide summarization. When provided:
  - Applies leaky relevance filter to content (40% threshold - lenient)
  - Compression ratio applies to focused content only
  - Example: "attention mechanisms", "deployment strategies"

- **style** (string) - Output style, default: "technical"
  - **"executive"**: High-level overview, 300-500 words max, focuses on key findings
  - **"technical"**: Balanced detail preservation (DEFAULT), uses compression_ratio directly
  - **"comprehensive"**: Low compression (2:1), preserves nuance and technical details

- **compression_ratio** (float) - Compression factor, default: 3.0
  - Applied to effective content size (after focus filtering if focus provided)
  - 3.0 = reduce to 33% of input (~1500 words → ~500 words)
  - Lower values (2.0) = more detail retained
  - Higher values (5.0) = more aggressive compression
  - Overrides style-based compression defaults

## How It Works

1. **Content Measurement**: Estimates input tokens (~4 chars/token)
2. **Focus Filtering** (if focus provided): 
   - Rates each chunk for relevance (0-10 scale)
   - Keeps chunks scoring ≥4 (leaky threshold - permissive)
   - Recomputes effective input size from filtered chunks
3. **Target Calculation**: Applies compression ratio to effective input
4. **Hierarchical Summarization**: Map-reduce pattern for long documents
5. **Length Control**: LLM prompted with target token count

## Output Characteristics by Style

### Technical (default)
- Preserves methodology, technical details, caveats
- Uses compression_ratio directly (default 3:1)
- Best for: Research, analysis, documentation
- Example output: ~33% of input length

### Executive
- High-level findings and implications only
- Fixed cap: 300-500 words regardless of input size
- Best for: Decision-makers, presentations, briefs
- Example: 8 papers → 1 page summary

### Comprehensive
- Low compression (2:1), preserves nuance
- Detailed technical content and supporting evidence
- Best for: Deep analysis, technical reviews
- Example output: ~50% of input length

## Leaky Focus Filter Details

When focus is provided, content filtering is intentionally **lenient**:

**Inclusion criteria (score ≥4/10):**
- Explicitly mentions focus topic (score 7-10)
- Provides context for understanding focus (score 5-6)
- Discusses related concepts (score 4-6)
- Contains cross-references to focus topic (score 4-5)

**Why leaky is important:**
- Preserves surrounding context for comprehension
- Avoids losing important related information
- Handles ambiguous relevance gracefully
- Better to include than risk false negatives

**Typical inclusion rates:** 30-50% of chunks (not 80-90%)

## Example Usage Patterns

**Default (technical, 3:1 compression):**
```json
{"type":"summarize","target":"$papers","out":"$summary"}
```

**Executive brief with focus:**
```json
{"type":"summarize","target":"$reports","focus":"cost analysis","style":"executive","out":"$brief"}
```

**Detailed focused analysis:**
```json
{"type":"summarize","target":"$papers","focus":"transformer attention","compression_ratio":2.5,"out":"$detailed"}
```

**Comprehensive review:**
```json
{"type":"summarize","target":"$document","style":"comprehensive","out":"$full_analysis"}
```

## Information-Theoretic Design

**Key insight:** Compression ratio applies to *relevant* information, not raw input.

**Without focus:**
```
8 papers (100K tokens) → 3:1 compression → 33K token summary
(Proportional coverage of all 8 papers)
```

**With focus="attention mechanisms":**
```
8 papers (100K tokens) 
→ Leaky filter keeps 3 papers + relevant sections (30K tokens)
→ 3:1 compression on 30K → 10K token summary
(Focused, detailed coverage of attention topics)
```

**Result:** Focus dramatically improves signal-to-noise ratio in output.

## Observability

Each summarization logs:
```
summarize: input=45000t, focus=yes, filtered=18000t (40%), 
target=6000t, style=technical, ratio=3.0, output=6200t
```

Monitor inclusion_pct: if consistently >75%, focus may be too broad.

## Edge Cases

- **Focus yields 0 chunks**: Falls back to full content with warning
- **Very short input (<4K tokens)**: Minimal compression applied
- **Filter error**: Includes chunk by default (fail-safe)
- **Target <300 tokens**: Floor applied for readability