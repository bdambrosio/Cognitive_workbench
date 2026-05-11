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

The proposal: **the system prompt receives only the top-K agreements relevant to the current turn**, gated by cosine similarity to the turn's embedding. Same architectural pattern as `_render_active_threads_block` — activation-gated rendering with a small primary/secondary cap.

No new persistent store. The canonical agreements state remains the linefeed-delimited list of agreement lines under CURRENT AGREEMENTS in discourse_state.txt. Embeddings are derived from the current lines on demand, using the bge-small model already used by threads. They can be cached against line content for efficiency, but there is no insert/update/delete protocol — when the file changes, the lookup follows.

Read side:
- Per-turn embedding of the user input (already computed for thread activation; can be reused).
- Cosine similarity against the current agreement lines (embedded on demand or via content-keyed cache).
- Render top-K above threshold, with the same primary/secondary cap pattern threads uses.

The detail-vs-pruning tension at the discourse-pass level is significantly relaxed *on the consumption side*. Size of the active state stops being a per-turn prompt-cost problem. The write side has its own scaling problem that this doesn't fix — addressed in the next section.

## Write-side: triage before CRUD

RAG-push fixes consumption cost. It does *not* fix the write-side cost: `analyze_segment` still receives the full prior_state on every call, and is asked in one shot to carry forward, update, and prune every existing item while also extracting new ones. That combined call is what produces the bench's worst score — update_correctness = 0.15 — and the per-item reasoning load grows with state size. So a more aggressive extraction pass actively makes the *next* segment's update reasoning harder. RAG-push without a write-side change would relocate the failure rather than remove it.

The fix mirrors the read-side change in spirit but uses semantic reasoning rather than embedding similarity:

**Triage pass.** Input: the segment plus the full agreements list, indexed. Output: just the indices of items the segment plausibly modifies, invalidates, or supersedes — no text reproduced. Output volume is bounded regardless of state size. The cognitive task is recognition over a list, which local LLMs handle far better than full-state rewriting.

**CRUD pass.** Input: the segment plus only the triaged items, plus a slot for newly extracted items. Output: updates / removals for the flagged items and any new items found. Working set is small enough that per-item attention is feasible.

**Assembly.** Final state = (untouched items, byte-identical) ∪ (CRUD outputs). Carryover items aren't regenerated, so they pass through identical to their prior text. carryover_preservation becomes an architectural property *for items triage leaves unflagged* — the realized floor is set by triage selectivity and CRUD restraint on flagged items, not by LLM memory. (v0.1 bench: 0.881 with deliberately false-positive-biased triage; tightening CRUD's leave-alone bias lifts it above 0.95 but at unfavorable cost to update_correctness.)

### Why LLM triage rather than embedding gating

The cheaper alternative — top-K by cosine similarity to segment — classifies by topical proximity, which is a noisy proxy for "this item is affected by what was said." It misses:

- Retractions whose language is topically distant from the agreement being retracted.
- Scope changes that invalidate without restating the topic.
- Indirect invalidations ("we no longer need to navigate northwest because we found the trail" invalidates a navigation plan via a non-navigation signal).

These are exactly the cases update_correctness currently fails on. Embedding gating wouldn't catch them either. LLM triage can.

This also aligns with the project's standing rule against similarity-as-classification (CLAUDE.md: no keyword matching — use semantic approaches). Embedding similarity sits closer to keyword matching than to semantic reasoning; the triage approach is the genuinely semantic one.

### What this costs

- Two sequential LLM calls per segment instead of one. Triage output is tiny, so the bottleneck is its input — same token cost as the combined call's prior_state. CRUD operates over a small subset, so it's faster than the current combined call. Net wall-clock should be comparable or better, but worth measuring rather than assuming.
- Triage miss is a real failure mode: an item that should be updated but isn't flagged silently persists stale. The bench can't currently distinguish triage-miss from reasoning-miss; instrument before relying on the new scores.
- Cardinality calibration. Too-few flags reproduces the current failure mode. Too-many flags gives the CRUD pass back a working set as large as the combined call, losing the benefit. Triage should bias toward false positives — the CRUD pass can choose to leave a flagged item unchanged, so over-flagging is recoverable while under-flagging is not.
- State-size ceiling is raised, not removed. Triage still reads N items as input. It degrades more slowly than rewriting, but at thousands of items would need its own follow-on architectural step. Not a near-term concern.

### Relationship to RAG-push

Independent and complementary. Write-side triage handles per-segment update accuracy; read-side RAG handles per-turn prompt economy. Both operate over the same agreement lines in discourse_state.txt — no separate store. Shipping triage without RAG-push is coherent; shipping RAG-push without triage leaves update_correctness broken and worsens it as state grows; doing both is the design.

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

A natural follow-on question: should the remember subagent get a `semantic_search` primitive so it can directly query the agreements by embedding similarity?

Probably not, at least not yet. Even at hundreds of agreements at ~100 chars each, the active-agreements file is ~10–20KB — well within the subagent's existing whole-file read budget (the prompt already routes "agreements / decisions / commitments" questions to `discourse_state.txt` as a whole-file read). Grep + read with line ranges remains tractable for queries of the form "did we agree X?". Adding semantic search to the subagent is a real complexity increment (a new primitive, embedding storage, plumbing) that's hard to justify when the file-grep path works for the foreseeable size range.

If state ever grows to thousands of agreements, the calculus shifts. Until then, the render-time RAG handles the prompt-economy problem; the subagent's existing primitives handle on-demand retrieval. Two separate concerns, two separate solutions, neither dragged into doing more than it needs.

## Remember-subagent semantics — keep the existing dispatch

The current remember prompt has 3-way semantic routing (user-state → companion file, agreements → discourse file, why-Jill → reasoning_trace). It already conflates agreements / decisions / commitments under one bucket and routes them to a single file. That's fine. The semantics differ but the access pattern is the same: read the file, find the relevant entries, cite. There's no operational benefit to making the subagent type-aware beyond what it already is.

The architectural change is at write-time (triage + CRUD over the existing agreement lines) and render-time (top-K render gated by similarity); the subagent stays as it is.

## What gets simpler under the combined change

- **Discourse-pass prompt design.** Splits cleanly into triage and CRUD. The "do (a) and (b) and (c) in one prompt" pressure decomposes — each pass has one cognitive load. Carryover stops being LLM-mediated.
- **Render-time prompt economy.** Bounded by render-time K, not by accumulated state size. Predictable.
- **Bench scoring axes.** add_recall and update_correctness remain primary. carryover_preservation becomes an architectural floor rather than a bench-measured outcome. The "state size" tension stays bounded on both sides — size growth doesn't translate to per-turn render cost, and write-side reasoning load no longer scales linearly with the full state.

## What stays unchanged

- The discourse state's **external** format (section headers, bullet style, content style) — same. `analyze_segment`'s inputs and final output shape are unchanged; only its internal call structure splits into triage + CRUD.
- The persistence model (discourse_state.txt as the canonical text snapshot, kept for the subagent's grep/read access).
- The reasoning trace, conversation log, companion model, persona — all unchanged.

Both layers are additive at the architectural boundary: the existing analyze_segment contract stays the same, but its internals decompose into triage + CRUD; render gets a new top-K gate. The existing artifacts continue to exist.

## Decisions locked (2026-05-11)

Resolved deliberately in favor of KISS — default to the existing artifact and ephemeral derivation rather than new persistent infrastructure.

- **Item identity is per-call, not persistent.** Triage numbers the current agreement lines sequentially (1..N) inside a single triage→CRUD cycle. The numbers exist only so triage can point at items for CRUD; they're discarded once the new state is written back. No UUIDs, no stable IDs.
- **No new schema, no new store.** Agreements remain a linefeed-delimited list of texts in discourse_state.txt under CURRENT AGREEMENTS. No Collection object, no per-item metadata, no migration. Embeddings are derived from current lines on demand (cache against content for efficiency; not a first-class store).
- **CRUD output is line-prefixed, tolerant.** Format on the order of `UPDATE n: <text>` / `REMOVE n` / `ADD: <text>`. No strict JSON — local LLMs are weak at it, and tolerant parsing is cheap.
- **No format change to discourse_state.txt.** It stays as the canonical text snapshot. Triage and CRUD round-trip through the same file format.
- **No pinned subset.** Operationally critical items are already redundantly represented via persona and companion model (see "Redundancy observation"). Render-side RAG can stand on top-K alone.
- **KEY DECISIONS MADE folds into CRUD.** Same prompt emits `UPDATE n` / `REMOVE n` / `ADD` for agreements and parallel directives for decisions. One LLM call covers both sections; avoids a third pass for a thin, structurally-similar section.
- **Agreement provenance tags pass through verbatim.** The parsed line is the full bullet content including the `(this segment)` / `(established earlier)` tag. Carryover items remain byte-identical. CRUD's UPDATE/ADD lines must include a tag; the prompt directs it on which one.
- **No triviality threshold on segments.** CRUD always runs (with an empty flagged set if triage flagged nothing), so the new-item slot still gets a chance. No "skip on short segment" heuristic; no threshold to calibrate.

## What's not yet decided

- **K and threshold values for render.** Threads use primary≥0.30, secondary≥0.20, secondary_cap=2. Agreements may need different values — they're more numerous than threads and may benefit from a larger K with a tighter threshold. Tune empirically once the basic plumbing exists.
- **Triage prompt design.** False-positive vs false-negative tradeoff (bias toward false-positive per the cost analysis); how to format the indexed list for cheap recognition; whether triage emits a brief reason per flag (could help CRUD focus, costs output tokens). Calibrate empirically.
- **New-item extraction: bundled into CRUD, or separate third pass.** Bundling avoids a third LLM call and lets CRUD see new + updated items together (useful for consolidation). Separating avoids cognitive interference between extraction and update. Default to bundling; revisit if extraction quality drops measurably.
- **Combined-call fallback at small state size.** Below some N (≈20 agreements?) the combined call may be cheaper and at least as accurate. Could keep both paths or commit to triage+CRUD for simplicity. Defer until plumbing exists.
- **Bench instrumentation to separate triage-miss from reasoning-miss.** update_correctness currently aggregates both failure modes. Once triage exists, an honest evaluation needs to attribute failures to the right stage — otherwise we can't tell whether to tune the triage prompt or the CRUD prompt.
- **Bench coverage of the read path.** Once RAG-push is implemented, the existing per-pair extraction bench remains valid but a complementary read-time bench (memory_recall-shaped probes against the agreement lines) becomes useful. Build only after the architecture lands.

## Bottom line

The discourse pass needs to extract more aggressively (add_recall and update_correctness are the load-bearing failure axes per the bench). Doing that without architectural change relocates the failure in two ways: more bloat in the system prompt, and worse per-item reasoning over a growing list inside the next reflect call. Two complementary changes address both:

- **Read-side RAG-push** — top-K from the current agreement lines in the system prompt — handles consumption cost.
- **Write-side triage + CRUD** — LLM triage flags affected items, CRUD operates only on those, carryover items pass through byte-identical — handles update accuracy and makes carryover an architectural property.

The remember-subagent stays as-is; the agreements file stays as-is; analyze_segment's external contract stays as-is. The new pieces are a triage→CRUD decomposition on the write side and a top-K render on the read side, both operating over the same agreement lines with embeddings derived on demand. KISS through the whole change.
