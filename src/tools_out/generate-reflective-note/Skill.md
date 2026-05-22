---
name: generate-reflective-note
type: python
description: "Generate content with awareness of the agent's current reasoning state — goal analysis, situation context, character orientation. Like generate-note but informed by internal planner state."
---

# generate-reflective-note

LLM-based content generation with access to the agent's internal reasoning state.
Like `generate-note`, but the LLM prompt includes the planner's Stage 1 context
(character orientation, goal analysis, tool selection, situation awareness) as
background, enabling state-aware or reflective generation.

## Input

- `prompt`: Generation instruction (required)
- `style`: `"code"` or `"text"` (optional, default: `"text"`)
- `target_tokens`: Integer (optional). Target output length in tokens. Overrides default max_tokens when provided via OUTPUT GUIDANCE.

## Output

Success (`status: "success"`):
- `value`: Generated content informed by agent state

Failure (`status: "failed"`):
- `reason`: Error description

## Behavior

- Automatically retrieves the planner's reasoning snapshot (if available)
- Includes it as `## Internal Reflective State` context in the LLM prompt
- The prompt is the actual query; the reflective state is background
- Falls back to regular generation if no planner state is available

## Planning Notes

- Use when the agent needs to generate content that reflects its current understanding,
  orientation, or reasoning about the goal and situation
- Use `generate-note` for plain generation without agent state awareness
- Typical uses: self-assessments, progress reflections, situation reports,
  agent-perspective summaries, epistemic status notes

## Examples

```json
{"type":"generate-reflective-note","prompt":"Assess your current understanding of the user's goals and any gaps","out":"$assessment"}
{"type":"generate-reflective-note","prompt":"What aspects of this task are you most uncertain about?","out":"$uncertainty"}
{"type":"generate-reflective-note","prompt":"Summarize what you know about the current situation from the agent's perspective","out":"$sitrep"}
```
