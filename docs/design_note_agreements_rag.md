# Design Note — Agreements as RAG-Push State

*Status: design discussion, not implementation. Recording the architectural direction that emerged from running the discourse-reflect bench (bench/discourse_reflect/v0.1) and observing how the production discourse pass scales.*

## Origin

The discourse-reflect bench surfaced two findings that reshape how we should think about the discourse-state architecture, not just its prompt quality:

1. **Production reflection is conservative to a fault.** Per-update scoring against frontier-labelled gold:
   - add_recall = 0.33 (misses ~2/3 of agreements the dialog established)
   - update_correctness = 0.15 (almost never refines stale items when the dialog supports it)
   - carryover_preservation = 0.87 (strong)
   - add_precision = 0.94 (strong — what's added is grounded)
   - wording = 0.85 (acceptable)

   The optimization target localizes sharply to add_recall and update_correctness. The conservative-and-stale failure is what the bench is meant to drive.

2. **Improving add_recall in isolation creates a downstream problem.** If a more aggressive prompt pushes per-update content from ~1.3 agreements/segment to ~3/segment, the active state hits ~250 agreements at the same conversation depth where it currently has ~60. None of the bench axes catches this. add_recall *improves*, but the resulting state becomes unusable: it appears in Jill's system prompt every turn, taking attention budget proportionate to its size.

So write-time accuracy and read-time economy are tied. Optimizing one without considering the other relocates the failure rather than removing it.

## The proposal: RAG-push for agreements

Currently the entire CURRENT AGREEMENTS section is dumped into the system prompt every turn. This is the "push" model — all state, every turn, full footprint. It works while state is small; it breaks as state grows.

The proposal: **agreements stay in a Collection with embeddings; the prompt receives only the top-K relevant to the current turn**, gated by cosine similarity to the turn's embedding. Same architectural pattern as `_render_active_threads_block` — activation-gated rendering with a small primary/secondary cap.

Write side:
- Discourse pass produces full new_state (no behavioral change).
- Parse new_state and prior_state into individual agreement items (line-per-bullet under the new prompt format).
- Compute the diff:
  - new items → embed, add to `agreements` Collection
  - removed items → delete embedding
  - reworded items → re-embed and replace
  - unchanged → no-op
- The bge-small embeddings already used by threads carry over. No new model.

Read side:
- Per-turn embedding of the user input (already computed for thread activation; can be reused).
- Cosine similarity against the agreements Collection.
- Render top-K above threshold, with the same primary/secondary cap pattern threads uses.

The detail-vs-pruning tension at the discourse-pass level is significantly relaxed by this. Size of the active state stops being a per-turn prompt-cost problem. The discourse pass can afford to extract more aggressively because the consumption side no longer pays for it linearly.

## What this is not

**Not** a switch to fully pull-based agreements. The render-time top-K *is* a push — agreements still appear in the prompt unsolicited. The change is from "all of them, always" to "the relevant ones, this turn." Jill doesn't have to know to ask; the activation gate handles it.

**Not** a justification for letting the discourse pass be sloppy. add_recall and update_correctness still need to improve — the agreement has to *exist correctly* to be retrievable. The bench's primary axes remain the right targets. RAG-push just makes their improvement non-self-defeating.

**Not** a structural reorganization of agreements vs. decisions vs. commitments. They have different semantics, but for retrieval purposes the distinction does little useful work. Treating them as a single typed bag and letting semantic relevance rank within is fine. Splitting the store buys complexity without obvious payoff. KISS.

## Redundancy observation

The push side currently double- and triple-stores some agreements:

- "Treat user as competent adult" — in Jill persona (`character` field), in companion model's "HOW TO BE USEFUL RIGHT NOW", AND in CURRENT AGREEMENTS.
- "Reuters is high-trust" — in companion observed-defaults (implicitly via "fact-checking style" descriptions), in CURRENT AGREEMENTS.
- Several other meta-stance items — operationally enforced by persona, restated in agreements.

The agreements that are *truly* load-bearing for every turn are mostly already represented elsewhere. Most of CURRENT AGREEMENTS is background context that only matters when revisited — exactly the shape that RAG-push handles well. So removing those from the prompt baseline doesn't lose them; it just eliminates the redundant render.

This isn't a separate optimization — it's a corollary that says "the cost of moving to RAG-push is lower than it looks, because the always-relevant items survive via other channels."

## Scale parsimony — why semantic search isn't needed for retrieval

A natural follow-on question: should the remember subagent get a `semantic_search` primitive so it can directly query the agreements Collection?

Probably not, at least not yet. Even at hundreds of agreements at ~100 chars each, the active-agreements file is ~10–20KB — well within the subagent's existing whole-file read budget (the prompt already routes "agreements / decisions / commitments" questions to `discourse_state.txt` as a whole-file read). Grep + read with line ranges remains tractable for queries of the form "did we agree X?". Adding semantic search to the subagent is a real complexity increment (a new primitive, embedding storage, plumbing) that's hard to justify when the file-grep path works for the foreseeable size range.

If state ever grows to thousands of agreements, the calculus shifts. Until then, the render-time RAG handles the prompt-economy problem; the subagent's existing primitives handle on-demand retrieval. Two separate concerns, two separate solutions, neither dragged into doing more than it needs.

## Remember-subagent semantics — keep the existing dispatch

The current remember prompt has 3-way semantic routing (user-state → companion file, agreements → discourse file, why-Jill → reasoning_trace). It already conflates agreements / decisions / commitments under one bucket and routes them to a single file. That's fine. The semantics differ but the access pattern is the same: read the file, find the relevant entries, cite. There's no operational benefit to making the subagent type-aware beyond what it already is.

The architectural change is at write-time and render-time (new state diff feeds an embedding store; render uses RAG); the subagent stays as it is.

## What gets simpler under RAG-push

- **Discourse-pass prompt design.** Less pressure on consolidation/pruning to keep state lean. The "do more of (a) and (b) and (c) in one prompt" pressure decomposes — extraction can be the dominant focus.
- **Render-time prompt economy.** Bounded by K, not by accumulated state size. Predictable.
- **Bench scoring axes.** add_recall and update_correctness remain primary. The "state size" tension we'd otherwise need to instrument as a 7th axis becomes much less acute — size growth doesn't translate to per-turn cost, so the bench can stay focused on extraction quality.

## What stays unchanged

- The discourse pass itself (DiscourseTracker.analyze_segment) — same prompt, same output format.
- The persistence model (discourse_state.txt as the canonical text snapshot, kept for the subagent's grep/read access).
- The reasoning trace, conversation log, companion model, persona — all unchanged.

The RAG-push layer is additive: parse + embed + diff at write-time, gated render at read-time. The existing artifacts continue to exist.

## What's not yet decided

- **K and threshold values for render.** Threads use primary≥0.30, secondary≥0.20, secondary_cap=2. Agreements may need different values — they're more numerous than threads and may benefit from a larger K with a tighter threshold. To be tuned empirically once the basic plumbing exists.
- **Whether to preserve the "always-on" subset.** A handful of agreements may genuinely warrant unconditional rendering (e.g., active monitoring procedures, explicit user directives). Could be implemented as a `pinned` boolean on each agreement. Defer until evidence demands it; a clean K-only render may be sufficient.
- **Diff granularity and parsing robustness.** Line-per-bullet under the new prompt format is straightforward to diff. Reworded items are hardest — naive diff sees them as remove+add. May want a fuzzy-match step (cosine sim on item embeddings between consecutive states) before classifying as modify vs. remove+add. Defer to first implementation pass.
- **Bench coverage of the new path.** Once RAG-push is implemented, the existing per-pair extraction bench remains valid but a complementary read-time bench (memory_recall-shaped probes against the agreements Collection) becomes useful. Build only after the architecture lands.

## Bottom line

The discourse pass needs to extract more aggressively (add_recall and update_correctness are the load-bearing failure axes per the bench). Doing that without an architectural change relocates the failure into prompt bloat. RAG-push for agreements removes the relocation problem and reuses the threads infrastructure already in place. The remember-subagent stays as-is; the agreements file stays as-is; the new piece is a small diff/embed loop on the write side and a top-K render on the read side. KISS through the whole change.
