---
name: summarize
description: Compress information while preserving key points and structure
type: python
trusted: true
parameters:
  - name: focus
    type: string
    description: Optional topic to guide summarization
examples:
  - '{"type":"summarize","target":"$doc","args":{"focus":"key points"},"out":"$summary","expect":"should capture main findings"}'
  - '{"type":"summarize","target":"$results","out":"$summary","expect":"should condense results"}'
---

# Summarize Content

Extract essential information from text, documents, or structured data while maintaining accuracy and context.

## Purpose

Transform verbose or detailed content into concise summaries appropriate for the target use case. Preserves factual accuracy while reducing cognitive load.

## Input Format

Accepts:
- Plain text (single document or passage)
- Structured data (JSON/dict with fields to summarize)
- Lists of items (creates consolidated summary)

## Optional Parameters

- **focus** - A topic or query to guide summarization. When provided, emphasize information relevant to this focus while still maintaining accuracy.

## Output Format

Returns a summary with:
- **Key points** - Most important information (2-5 bullets)
- **Context** - Brief background if relevant
- **Actionable items** - Decisions, next steps, or implications (if present)

## Summarization Guidelines

### Length Scaling
- **Brief** (default): 3-5 sentences, highlights only
- **Standard**: 1-2 short paragraphs, includes context
- **Detailed**: Multiple paragraphs, preserves nuance

### Content Priorities
1. **Focus relevance** - if 'focus' parameter provided, prioritize information related to that topic
2. **Facts over opinions** - unless opinions are the subject
3. **Novel information** - what's new or unexpected
4. **Actionable insights** - what can be done with this
5. **Critical context** - minimum background needed to understand

### Quality Checks
- Accuracy: No hallucination or inference beyond source
- Completeness: Captures all major themes
- Clarity: Self-contained without source
- Brevity: Removes redundancy and filler

## Example Usage

**Input (verbose):**
```
The research paper discusses three approaches to neural scaling. 
The first approach focuses on parameter count... [3 more paragraphs]
```

**Output (brief):**
Neural scaling research identifies three approaches: parameter scaling (most common, diminishing returns above 100B), data scaling (underexplored, limited by quality), and compute-optimal scaling (Chinchilla findings suggest balanced approach). Key finding: current models may be over-parameterized and under-trained.

**Input with focus:**
```
Input: "Google released Gemini 2.0 with improved multimodal capabilities..."
Focus: "new capabilities"
```

**Output (brief, focused):**
Gemini 2.0 introduces native image and video understanding, real-time audio processing, and enhanced code generation with 50% faster inference compared to 1.5 Pro.

## Special Handling

- **Multiple sources**: Synthesize across sources, note agreements/conflicts
- **Technical content**: Preserve technical accuracy, define jargon if needed
- **Temporal data**: Maintain chronological clarity
- **Structured data**: Preserve hierarchical relationships in flattened form

## Error Handling

If input is:
- Already concise: Return original with note
- Empty/null: Return "No content to summarize"
- Malformed: Attempt best-effort summary, flag issues