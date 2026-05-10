# Design Note — Threads as Graded-Relationship Anchors

*Status: design discussion, not implementation. Recording current understanding from a sequence of architectural reframings prompted by the four-tier awareness benchmarks. Not yet a spec.*

## Origin

The four-tier operational-self-awareness benchmark suite (introspective fidelity, behavioral consistency, perturbation detection, counterfactual self-prediction) surfaced a consistent gap: when a user installs a session-level rule ("for this session, be a strict skeptic" / "respond only in continuous prose"), CW has no architectural surface that captures it as a tracked, introspectable, perturbable element of state. The rule lives in conversation history, mixed with everything else, and falls out of attention as the session grows.

Bench evidence:
- **Tier 3 PROBE-01 / PROBE-02** (perturbation detection on dropped prefix-turn rules): both Jill and the capability-described baseline failed to detect a missing rule. Neither has a representation of "what rules should be in effect right now."
- **Tier 4 PAIR-08** (skeptic-rule perturbation): the perturbation didn't shift behavior because the rule was never installed as a tracked directive in the first place — Jill's training prior on the topic was doing the work the rule was supposed to do.
- **Sharpest moment** (baseline PROBE-02 perturbed cell): the agent's *behavior* used lists when the format rule was dropped (the rule's absence was reflected in output formatting), but the agent's introspection said nothing was missing. Behavior-yes, introspection-no, on the same turn.

The initial diagnosis was: **CW needs a "session contract" surface — a tracked list of user-installed rules with a `provenance` field, exposed in orientation each turn.** That framing turned out to be a symptom-level fix, not the right diagnosis.

## First refinement: not session, thread

Sessions aren't the right scope. Real conversation interleaves multiple activities — a user might work on a code project Monday, ask unrelated questions Tuesday, return to the code project Wednesday. Those are two threads sharing two sessions, not a single session-state. Cross-session continuity matters; intra-session topical interleaving matters; rules belong to *activities*, not to time-bounded sessions.

Threads are the right scope: a thread is a coherent activity that may span multiple sessions and may interleave with other threads within a session. Rules attach to the activity, not to wall-clock time.

This shifts the construct from "list of currently-installed rules" to "named coherent contexts, each carrying its own rules-in-effect, scoped concerns, topical summary, and history pointers."

## Second refinement: turn-to-thread membership is graded

A turn doesn't belong to one thread. It can participate to varying degrees in multiple threads simultaneously. A turn that compares speech-draft tone to financial-report tone participates partially in both threads. A turn that brings up a related side-topic has weak membership in the original thread plus partial membership in the side-topic.

Forcing binary "this turn is in thread A" assignment destroys information. Norvig's observation about poetry — that ambiguity is sometimes the point, not noise to resolve — applies here. Conversation regularly carries content that's intentionally or unintentionally ambiguous between interpretations, and the ambiguity may be the load-bearing piece.

So: turn-to-thread membership is a distribution, not a category.

## Third refinement: threads themselves are not crisp

Even if turn-to-thread is graded over a fixed thread inventory, the inventory itself isn't clean. Threads overlap (the speech-draft thread overlaps with the career-thinking thread because the speech is about the user's career). Threads emerge and reinterpret retroactively (what looked like one thread becomes two; two later merge). They exist at multiple resolutions simultaneously (a "career" macro-thread contains many micro-threads at finer grain). They're observer-dependent (the user's notion of where one thread ends may differ from the agent's).

Pulled this far, the recursion suggests no level cleanly bottoms out into discrete categories that aren't themselves abstractions over a continuous structure. Conversation doesn't have a categorical basis; it has a continuous semantic structure with useful but lossy clusters.

## Resolution: anchors stay crisp, relationships become graded

The recursion is real but doesn't argue against having any structure at all. Architecture is necessarily lossy; the question is *what to discretize* and *what to leave fluid*.

The position that survives the recursion:

**Crisp where anchors serve human and system reference. Graded where relationships connect anchors.**

- **Anchors stay crisp.** A thread either exists in the registry or it doesn't. It has a stable ID, a human-readable name, lifecycle states (active / paused / completed / archived), and the user can refer to it (`/thread resume speech-draft`) or modify it (`/thread merge A B`). Same for concerns, rules, recall hits — they're crisp pegs to hang things on.
- **Relationships between anchors become first-class and graded.** Currently CW represents most state as set membership: a concern is in the concerns collection (binary), a recall hit either passes the relevance threshold (binary include/exclude), a rule is installed (binary). The thread construct introduces graded relationships:
  - turn ↔ thread: a turn has weighted membership across active threads
  - rule ↔ thread: a rule applies with varying strength to its host thread, possibly with leakage into adjacent threads
  - concern ↔ thread: a concern can be partially scoped to one thread, partially shared across several
  - thread ↔ thread: threads can have graded similarity / overlap / sub-thread relationships among themselves

The architectural shift is from *set membership* to *weighted association*. The discretization happens at the entity level (anchors are named, listable, modifiable); the gradient lives in the links between entities.

Anchors aren't claiming to model conversational reality faithfully. They're handles for reference. The faithful-to-reality work — graded interpretation of how any given turn relates to any given anchor — stays in the LLM's attention over assembled prompts. That's already where the LLM's gradient capability lives, and we shouldn't try to take it from there.

## Parsimony observation: graded relationships reduce category proliferation

A consequence of graded relationships that's worth recording explicitly: **with graded membership, you need fewer named threads, not more.**

Under crisp set membership, every meaningful combination of activities needs its own named compound thread. "Code project work" and "career thinking" as separate threads can't represent a turn that's about both — you'd need a third compound thread "code project work that's also career thinking." With many basic activities, the combinatorial explosion of compounds is severe.

Under graded membership, the combinatorics collapse. A turn that's about both gets `{code_project: 0.5, career_thinking: 0.5}`. The compound thread isn't needed — the combination is expressible as a weighted distribution over the basic primitives. N basic threads cover an exponential number of effective combinations.

The same parsimony argument applies to other graded-relationship pairs. A concern doesn't need to be duplicated across multiple threads to capture cross-thread relevance — it can have graded membership in several. A rule doesn't need scoped variants for "this thread strict" vs. "that thread loose" — it can have a single canonical form with graded application strength per thread.

The architectural payoff: **fewer entities, simpler registries, but richer expressed structure.** This is the pattern that distributed representations have over atomistic symbolic ones — you trade categorical clarity for representational efficiency, and the trade is usually favorable when the underlying domain is genuinely continuous.

There's a constraint: the basic primitives have to be well-chosen. If you pick the wrong basis threads, no amount of graded membership recovers structure that's missing from the basis. But this argues for letting threads emerge from use rather than being predeclared in a taxonomy — the basis adapts to the activities the user actually engages in.

## Open operational questions

Conceptually clean; operationally underspecified. Before any code:

- **How is turn-to-thread membership computed?** Per-turn LLM classification call? Embedding similarity to thread centroids? Hybrid with user declarations?
- **How often are memberships updated?** Once per turn at write time, or recomputed on retrieval?
- **How are relationships stored cheaply?** A dense matrix of (turn × thread) is fine at small N but doesn't scale. Sparse representation with a relevance threshold? Lazy computation?
- **Thread detection — explicit, inferred, or hybrid?** User-declared threads are predictable but high-friction. Agent-inferred is smooth but error-prone. Hybrid (agent infers, surfaces inference, user can override) probably right but adds turns.
- **Lifecycle management.** Who decides a thread is paused vs. completed? Time-out? User declaration? Agent inference?
- **Cross-thread state.** Some user dispositions ("I trust AP") are global, not thread-scoped. Two-tier model with user-level vs. thread-level state?
- **Recall scoping.** Should recall default to active-thread or be cross-thread? How does graded thread activation weight recall queries?
- **Migration of existing surfaces.** Concerns, discourse_state, companion_model — what happens to them? Add a thread_id field? Re-architect as per-thread sub-objects? Index-only layer over flat surfaces?

Each of these has multiple defensible answers. The conceptual framing here doesn't decide them.

## Relationship to existing CW state

Existing surfaces (concerns, recall, discourse_state, companion_model, reasoning_history) remain. None are deleted. What changes is that:

1. Thread anchors are added as a new top-level surface in the prompt assembly pipeline.
2. Existing surfaces gain graded thread-affiliation as a relationship attribute (not a hard scope).
3. Orientation builder consumes the active-thread distribution to weight which surfaces to surface in the current prompt.
4. Recall queries become thread-aware (default scope to active thread, with explicit cross-thread retrieval available).

The thread layer doesn't replace existing structure; it adds a layer of organization that makes the existing structure more navigable.

## Status and next step

This note records the current understanding. It is **not** a spec. Before committing to a build:

1. Identify a concrete user use case that justifies the complexity. Without a clear pattern where multi-thread / cross-session continuity would noticeably matter, the existing flat-surface architecture is sufficient and threads are over-engineering.
2. Decide the open operational questions above, at least the load-bearing ones (explicit vs. inferred detection; persistent vs. ephemeral; how memberships are computed).
3. Pilot the graded-relationship pattern in one place (probably turn ↔ thread as the easiest entry point) and let the design teach what generalizes vs. what's domain-specific.

The thread construct is a candidate first step into a broader research direction: replacing crisp set-membership with weighted association across CW's state surfaces, where the underlying conversational structure is genuinely graded. Threads are the most visible place where the current crispness mismatches reality. They're not the only place.
