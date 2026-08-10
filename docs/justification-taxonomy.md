# Justification taxonomy — leaf and inference types

Closed vocabulary for annotating justification graphs (claims → premises →
evidence), serving three uses:

1. **Review keys for `justify`** — each entry carries a review question the
   audit pass applies when the pattern is present (generalizing the current
   single model_prior audit note).
2. **Graph annotation** — dimension/value tags on leaves and type tags on
   inference edges, storable as Relations with an ordinal judgment property.
3. **Deterministic reduction** — grades propagate AND=min, OR=max, then
   caps apply; output is an overall grade plus the min-path (the weakest
   link, identified mechanically).

Admission rule: an entry earns its place only if it moves a grade or
redirects a min-path. Classification into the vocabulary is semantic
(LLM, validated against the closed sets below) — never keyword matching.
Reduction over the tags is plain code, no LLM.

## Ordinal scale

    sourced > probable > unverified > suspect > refuted

- **sourced** — mechanically checked this session: verbatim quote matched
  against the persisted observation, deterministic recomputation, or a
  primary-source retrieval performed now. Named `sourced` rather than
  `verified` deliberately: the audit notes instruct the agent to *verify*
  a weak claim with a tool, and a grade whose name is the natural English
  description of that act gets claimed as an outcome of it (turn 2403 —
  a correct verification reported as "upgraded to verified", a grade
  change no code emitted). Only the reducer assigns grades; no name
  reachable by performing a check can stay honest.
- **probable** — supported, no flagged weakness. **The default**, including
  legacy Notes/Collections with no annotations (`provenance-missing`).
- **unverified** — rests on unchecked prior, testimony, or paraphrase for
  an externally checkable fact.
- **suspect** — a flagged weakness combination applies (see caps).
- **refuted** — verification was performed and contradicts the claim.
- **conflict** — off-scale terminal: independent evidence disagrees.
  Halts reduction for that subtree; demands resolution or dual reporting.

Reduction: AND node = min(children); OR node = max(*independent*
children); caps apply after propagation. `conflict` propagates as itself.

## Leaf dimensions (premises / evidence nodes)

Compose with the existing grounding types (retrieved / memory /
user_asserted / context / inferred / model_prior); they do not replace
them. Each dimension has a closed value set; unannotated = the neutral
value.

### volatility: `volatile` | `stable`
Whether the fact is of a kind that changes: org status, prices, versions,
roles, availability, schedules (volatile) vs definitions, mathematics,
settled history (stable).
- **Review key (volatile):** could this have changed since the evidence
  date — or, for model_prior, since training cutoff? If load-bearing,
  verify now. *(The SpaceX key.)*
- **Effect:** model_prior × volatile → cap **suspect**. Aged evidence ×
  volatile → cap **unverified**. Stable exempts prior-grounded leaves from
  both caps.

### source-grade: `primary` | `unknown` | `unreliable`
Authority of the evidence source *for this claim*: the subject's own
filings/IR/official docs/repo of record (primary); ungraded outlet
(unknown — the default web grade); on the user-curated unreliable ledger.
- **Review key (unknown):** is a primary source for this claim cheaply
  reachable? Prefer it when the claim is load-bearing.
- **Effect:** quote-matched × primary → **sourced**. unknown → cap
  **probable**. unreliable → cap **suspect**.

### mediation: `direct` | `mediated`
Whether this harness retrieved the evidence or is reading a model's
account of it. Derived from the tool, not tagged by the attributor:
fetched bytes, API rows, file text and deterministic renders are
`direct`; anything an LLM wrote is `mediated`, including
local-ground-truth subagents (`inspect`, `security`, `recall`) whose
underlying reads were real but whose *observation* is a synthesis of
them. Orthogonal to source-grade: that axis asks how authoritative a
source is, this one asks whether we actually read it.
- **Review key (mediated):** a verbatim quote against a synthesis proves
  the synthesis said it, not that the source did. If the claim is
  load-bearing, fetch the named source and check it.
- **Effect:** currently surfaced in the render, not a cap — with
  source-grade unimplemented every recorded-evidence path already
  returns **probable**, so a cap would be inert. When source-grade
  lands, `mediated` must cap at **probable** so a synthesis cannot be
  promoted to **sourced** on the strength of a verbatim match against
  itself.

### quote: `matched` | `absent`
Whether a verbatim span, machine-matched against the persisted
observation, anchors the leaf (the citation machinery).
- **Review key (matched):** does the quoted span *entail* the claim —
  support, not co-occurrence?
- **Review key (absent, on retrieved):** why is there no quotable span?
  A retrieved claim with no span is the paraphrase-drift signature.
- **Effect:** matched → floor **probable**; absent on retrieved → cap
  **probable**.

### polarity: `presence` | `absence`
Whether the evidence is a finding or a lack of findings ("search returned
no mention of X").
- **Review key (absence):** absence evidence is only as good as the probe —
  see the `negation-from-absence` edge, whose query-adequacy modifier
  does the real work.
- **Effect:** absence → never above **probable** on its own.

### age: `fresh` | `aged`
Evidence date relative to the claim's volatility class — a memory Note
inherits the date of its *originating turn's* evidence (walk the Level-1
chain), not the date it was recalled.
- **Review key (aged):** for volatile claims, re-verify rather than
  re-cite. *(The stale-memory key — volatility one hop removed.)*
- **Effect:** aged × volatile → cap **unverified**.

### testimony (grounding-refinement on user_asserted / third-party)
Authoritative for the speaker's own states, preferences, and intentions;
testimony for world-facts.
- **Review key:** is the claim about the speaker (authoritative) or the
  world (verify if load-bearing)? A user premise that *contradicts* your
  prior on a checkable fact is a verification trigger, not an error to
  correct. *(The SpaceX lesson, user-side.)*
- **Assistant-side:** the assistant is likewise authoritative for its own
  current state. A claim restating operational state visible in the turn's
  records (the active-concerns block) grounds `context`, never
  `model_prior`; self-state the records do NOT show stays `model_prior`
  and earns its background check. *(The turn-2316 lesson: a concern-list
  reply graded 8× suspect and spawned a verification of the assistant's
  own self-description.)*

## Inference edge types

One type per inferred claim (edge from premises to conclusion). AND over
its premises unless noted.

### `entailment`
Conclusion is contained in / directly restated by the cited evidence.
- **Review:** does the span actually contain it?
- **Effect:** min(premises), no penalty. The only edge that can carry
  **sourced** upward.

### `deduction`
Follows logically from jointly held premises.
- **Review:** are *all* premises cited? A hidden premise must be added as
  a leaf — usually model_prior, which is where chains rot silently.
- **Effect:** min(premises).

### `calculation`
Arithmetic / unit / date transform.
- **Review:** recompute with the calculate tool.
- **Effect:** **sourced** if recomputed, else min(premises).

### `generalization`
Instances → rule.
- **Review:** enough instances? Representative? Counter-instances sought?
- **Effect:** cap **probable**.

### `extrapolation`
Projecting a trend or pattern forward.
- **Review:** is the pattern's driver still in force over the projected
  span?
- **Effect:** cap **probable**; volatile domain → cap **unverified**.

### `analogy`
Mapping from a similar case.
- **Review:** do the load-bearing features actually transfer?
- **Effect:** cap **unverified**.

### `causal-attribution`
Cause inferred from sequence or correlation.
- **Review:** alternative causes considered? Confound ruled out?
- **Effect:** cap **unverified**.

### `negation-from-absence`
Didn't find it → it doesn't exist. Modifier: **query-adequacy**
(`adequate` | `inadequate`).
- **Review:** was the probe *designed* to find the thing if it existed —
  right venue, right phrasing, existence-shaped rather than
  comparison-shaped? One venue or several? Right timeframe?
- **Effect:** adequate → ceiling **probable**; inadequate → **suspect**.
  *(The Qwen key.)*

### `evidence-repurposing`
Evidence gathered for question A pressed into service for claim B.
- **Review:** would this evidence plausibly look different if the claim
  were false? If not, it isn't evidence for the claim.
- **Effect:** cap **unverified**.

### `circular-support`
The conclusion (or a restatement) appears among its own premises —
including "no need to verify because <conclusion>".
- **Review:** strike the conclusion from the premise set; does anything
  remain?
- **Effect:** **suspect** regardless of leaf grades. *(The SpaceX
  worst-sentence key.)*

### `paraphrase`
Restating a source without a verbatim anchor.
- **Review:** if load-bearing, fetch the original and quote it. A
  remembered paraphrase of a justification is not a justification.
  *(The laundering key.)*
- **Effect:** cap **probable**.

### `corroboration` (OR-join)
Multiple sources supporting the same claim.
- **Review:** are they *independent* — or do they share a root (wire
  copy, common upstream, one repo mirrored)? Non-independent disjuncts
  collapse to a single source.
- **Effect:** max(independent disjuncts).

## Structural conditions (graph-level)

### `conflict-unresolved`
Independent evidence disagrees and no resolution was performed.
- **Review:** resolve by source-grade and freshness, or report both
  positions explicitly. Never silently pick one.
- **Effect:** subtree grade = **conflict** (off-scale, halts).

### `single-source`
A load-bearing claim whose OR-join has one disjunct.
- **Review:** is a second independent source cheap? If yes, get it.
- **Effect:** cap **probable**.

### `provenance-missing`
Legacy Note / Collection with no recorded trail.
- **Review:** none at rest. Verify only when it becomes load-bearing for
  a volatile claim.
- **Effect:** default **probable** (the graceful-degradation rule).

### `premise-contradicts-user`
The reply denies the user's checkable premise on non-verified grounds.
- **Review:** verify before contradicting.
- **Effect:** **suspect** + verify-now trigger.

## Canonical caps (worked combinations)

| Pattern | Grade | Incident |
|---|---|---|
| model_prior × volatile, unverified | suspect | SpaceX "private company" |
| negation-from-absence × inadequate query | suspect | Qwen3.8-27B "unavailable" |
| circular-support | suspect | "tools inapplicable because private" |
| paraphrase of a prior justification | cap probable | justify-via-recall laundering |
| quote-matched × source-primary | sourced | ir.spacex.com correction |
| aged memory × volatile | unverified | (anticipated: stale Note re-cited) |

## Storage sketch (Relations)

Edge Relations typed by inference type; leaf tags and the ordinal
judgment stored as properties; judgment recomputed (not hand-edited) by
the reducer; unannotated legacy leaves read as `probable`. Original-query
grade and post-audit grade are separate judgments on the same node — the
delta is the retraction signal, and over time the (grade, outcome) pairs
are the calibration data that would justify any future numeric layer.
