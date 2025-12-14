---
name: is-question
description: Check if text contains a question
type: prompt_augmentation
parameters: none
examples:
  - '{"type":"if","condition":{"type":"tool_condition","tool":"is-question","target":"$input"},"then":[...]}'
---

# Is Question

Determines if the input text is a question or contains a question.

## Purpose

Boolean condition for identifying interrogative content in control flow.

## Input Format

Accepts:
- Plain text string

## Output Format

**CRITICAL**: Return ONLY the word "true" or "false", nothing else.

- Return "true" if the text is a question or contains a question
- Return "false" if the text is not a question

Do NOT include explanations, punctuation, or any other text. Only output "true" or "false".

## Examples

Input: "What is the weather like today?"
Output: true

Input: "The weather is nice today."
Output: false

Input: "I wonder if it will rain. Should I bring an umbrella?"
Output: true

