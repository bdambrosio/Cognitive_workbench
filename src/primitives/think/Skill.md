---
name: think
type: primitive
description: Generate an internal reflection on the state of the plan
---

# Think

## INPUT CONTRACT

- `value` (required): Literal string or `$variable` referencing thought prompt

**REQUIREMENTS:**
- Value must be provided (string content for the thought)

## BEHAVIOR

Generates an internal thought that enriches the planning context for subsequent tool selections. The thought is appended to the internal conversation state, allowing the LLM to reference it in later reasoning.

Thoughts are:
- **NOT** persisted as Notes
- **NOT** published to the UI
- **NOT** communicated to the user

Use this for tracking reasoning steps, making observations, or noting intermediate conclusions that inform later decisions.

## EXAMPLES

```json
{"type": "think", "value": "The user wants a summary, but how long should it be?"}
```
```json
{"type": "think", "value": "Do I already have the information I need to answer the question?"}
```
```json
{"type": "think", "value": "$observation"}
```

## OUTPUT

Returns `success`. The thought is injected into the planning context but produces no external side effects.

## ANTI-PATTERNS

- Using `think` to communicate with the user — use `say` instead
- Using `think` to store persistent data — use `create-note` instead
