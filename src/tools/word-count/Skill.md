---
name: word-count
type: python
description: "Count words in text. Use to determine length of a document, e.g. to determine if it needs to be summarized before further use"
---

# Word Count Tool

Counts the number of words in input text.

## Purpose

Simple deterministic word counting for text analysis. Use to determine document length, e.g., to determine if it needs to be summarized before further use.

## Input

- `target`: Note ID or variable containing text to count

## Output

Returns string with word count (e.g., "Word count: 6").

## Behavior & Performance

- Simple deterministic counting
- Fast and reliable
- Works on any text content

## Guidelines

- Use to check document length before processing
- Useful for determining if summarization is needed
- Returns formatted string, not numeric value

## Usage Examples

Count words in text:
```json
{"type":"word-count","target":"$text","out":"$count"}
```