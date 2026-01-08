---
name: word-count
type: python
description: "Count words in text. Use to determine length of a document, e.g. to determine if it needs to be summarized before further use"
---

# word-count

Simple deterministic word counting for text analysis.

## Input

- `target`: Note ID or variable containing text to count

## Output

Success (`status: "success"`):
- `value`: String with word count (e.g., "Word count: 6")
- `extra.count`: Integer count

## Behavior

- Simple whitespace-based counting
- Fast and reliable
- Works on any text content

## Planning Notes

- Use to check document length before processing
- Useful for determining if summarization is needed

## Example

```json
{"type":"word-count","target":"$text","out":"$count"}
```
