---
name: discover-notes
type: primitive
description: Global embedding-based search across all Notes; returns a Collection of matches
---

# Discover-Notes

## INPUT CONTRACT

- `query` (required): Natural-language search string
- `out` (required): `$variable` for the resulting Collection
- `limit` (optional, default `5`): Max number of Notes returned (top-K by similarity)
- `threshold` (optional, default `0.3`): Minimum cosine similarity for a match

**REQUIREMENTS:**
- `query` and `out` must be provided

## BEHAVIOR

Runs an embedding-based search over every Note in the current infospace and returns the top-`limit` Notes above `threshold` as a Collection. Each hit is wrapped in a structured search-result Note (same shape as `search-web` / `semantic-scholar` results). An empty-case (no matches) still returns `success` with an empty Collection.

**`limit` is a hard top-K cap.** There is no way to express "all matches above threshold" — if you need higher recall (e.g. the goal says "all existing Notes that X"), pass an explicit larger `limit` (50+ is reasonable for a few-hundred-Note infospace). With the default of 5 the planner cannot distinguish "exhaustive" from "undersampled".

## EXAMPLES

```json
{"type": "discover-notes", "query": "transformer architectures or attention mechanisms", "out": "$matches"}
```
```json
{"type": "discover-notes", "query": "meeting notes about the Q2 release", "limit": 50, "threshold": 0.25, "out": "$hits"}
```

## OUTPUT

Returns `success` with the Collection bound to `out`. Use `get_items("$var")` to read the list of result Note IDs.

## FAILURE SEMANTICS

- Missing `query` or `out`
- Underlying search failed (embedding index unavailable)

## ANTI-PATTERNS

- Using the default `limit=5` for "find all" goals — the result is silently top-5, not exhaustive. Always pass an explicit higher `limit` when the goal wording signals exhaustiveness.
- Treating an empty-Collection result as `failed` — an empty Collection is the legitimate no-match outcome. Branch on `len(get_items("$var"))`, not on the tool's status.
- Calling `discover-notes` when the goal names a specific resource (`_rss_pending_titles`, etc.) — use `load` for known names, `discover-notes` for semantic discovery.
