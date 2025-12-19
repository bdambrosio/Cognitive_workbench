---
name: word-count
description: Count words in text. Use to determine length of a document, e.g. to determine if it needs to be summarized before further use.
type: python
schema_hint:
  target: "$variable or Note ID"
  out: "$variable"
parameters: none
examples:
  - '{"type":"word-count","target":"$text","out":"$count"}'
---

# Word Count

Counts the number of words in input text.

## Purpose

Simple deterministic word counting for text analysis.

## Input Format

Accepts:
- Plain text string

## Output Format

Returns string with word count.

## Example

Input: "Hello world this is a test"
Output: "Word count: 6"

