---
name: filter-semantic
description: Filter a list of items to those matching a natural-language condition (e.g. "only the items that mention X"). NOT YET WIRED in chat-mode — present in src/tools/ for future enablement.
args:
  source: required string — items to filter (concrete shape TBD pending wiring)
  predicate: required string — natural-language condition, e.g. "mentions safety AND published after 2024"
  mode: optional string (default "include") — "include" returns matches, "exclude" returns non-matches
---

# filter-semantic

Apply a natural-language predicate to each item in a list and return the items that match (or, with `mode: "exclude"`, the items that don't).

**Status:** the tool exists in `src/tools/` but is not yet exposed via the chat ReAct catalog. The list-shape input convention hasn't been settled (chat ReAct passes strings as `$stepN` bindings; the OODA executor passed Collection IDs). When wired, the args contract above will be revised to match the chosen shape.

## Predicate guidance

- Semantic: `"has a positive tone"`, `"contains code or implementation details"`
- Boolean: `"mentions safety AND published after 2024"`
- Numeric / date: `"price under $50"`, `"earlier than 2020"`

Phrase predicates to describe what to **match**. Use `mode: "exclude"` instead of writing negative predicates ("does NOT contain X").

## Why this is different from `assess`

- `assess` operates on **one** text, returns one bool.
- `filter-semantic` operates on **many** texts, returns the subset that match.

One tool call vs N tool calls for the same job.
