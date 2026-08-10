# Project Rules

House rules a capable model wouldn't reach on its own. No restating of
ordinary good practice, and no architecture — that lives in
`docs/STATUS.md`, which gets reviewed. Rules naming modules rot silently;
behavioural rules don't. For trivial tasks, use judgment.

## Stance

Be concise, direct, and candid. Challenge weak assumptions. Keep verified
facts distinct from inference and from guesses, and say which you are
offering — "I checked and it says Y" and "it probably does Y" must not
read alike.

Ask only when a decision is materially ambiguous, risky, or needs
approval; otherwise choose, say so, continue. Exception: when a request
has several real readings that lead to different work, present them
instead of silently picking one.

## No keyword matching

Never use keyword or string matching for classification, routing,
affordance detection, or intent understanding — use an LLM call with a
natural-language description, embedding similarity, or another
meaning-based method. Keyword lists always miss edge cases. Only with
explicit permission.

## Reuse over re-implementation

Before writing logic for a common task (fence stripping, JSON repair,
retry/backoff, path resolution, env reading), look for an existing
utility. If several sites need it and none exists, add one helper in
`utils/`. When a duplicate has crept in, consolidate rather than add a
third.

## KISS — and let the user carry a little

Review every plan for unnecessary complexity before proposing it. The
non-obvious half: prefer a significantly simpler design even when it
shifts a *minor* burden onto the user. The tell is machinery whose whole
purpose is to spare them one small explicit action — inference,
heuristics, disambiguation, and the guards those need. Name the trade
and let them choose; ask when unsure which way it falls.

Distinct from feature creep (that's about scope — do less). This is
about where the work sits when the feature is agreed.

## Surgical changes

Touch only what the request requires; match surrounding style even where
you'd choose differently. Clean up orphans *your* edit created; leave
pre-existing dead code alone and mention it instead. Nothing speculative —
no unasked features, no abstraction for one call site, no error handling
for impossible states.

## Verify against the real thing

Validate user-facing work in the real interface: run it, look at it,
screenshot it. Passing tests routinely coexist with a broken screen. When
you claim something works, say how you know — and if a check was skipped
or a test failed, say that plainly instead of reporting around it.
