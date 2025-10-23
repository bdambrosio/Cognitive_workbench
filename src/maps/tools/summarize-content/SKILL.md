---
name: summarize-content
description: Compress information while preserving key points and structure
type: prompt_augmentation
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
1. **Facts over opinions** - unless opinions are the subject
2. **Novel information** - what's new or unexpected
3. **Actionable insights** - what can be done with this
4. **Critical context** - minimum background needed to understand

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