# Auditable Justification for Conversational Agents

*Technical note — Cognitive Workbench, August 2026*

## The problem

I asked my agent Jill (Cognitive Workbench, running on Gemma4-31B-it) a
version-specific question about a game's production chain. She answered
from model memory — no search, one reasoning step, straight to a
detailed, confident reply. The reply was substantially wrong. Asked to
justify it, she produced a well-organized rationale, complete with
headers, explaining why each part of the wrong answer was sound.
Nothing in that exchange was lying, exactly. It was worse: a system
confidently and honestly producing, and then justifying, a wrong
answer.

Self-justification like this isn't justification. A justification is a
**record** — produced by a different process than the one being
audited, resolvable to evidence that existed before the question was
asked. A model narrating its own confidence fails all three conditions,
and a model asserting "0.85" about its own claim is the same
self-narration problem wearing a number.

The problem this note addresses: give a conversational agent a
justification mechanism with those properties — separate process,
checkable record, prior evidence — without degrading answer latency,
and without assuming frontier-scale compute.

## Background

### The proof tradition, and how much of it transfers

At one end of the spectrum sits formal verification: LEAN, Coq,
Isabelle. A proof there is a chain of inferences from axioms, every
step an instance of a sound rule, and — the crucial property —
**checking is decidable**. The checker is small and deterministic. It
doesn't understand the theorems; it verifies that each step follows,
and its verdict is a fact, not a judgment.

Almost none of this survives contact with ordinary discourse:

- **There are no axioms.** The leaves of a conversational claim are a
  web search result, a note written three weeks ago, something the user
  said, background knowledge absorbed in training. Every leaf is a
  source, and sources have reliability, not validity.
- **There are no sound inference rules.** "The wiki says X, therefore
  X" is not modus ponens. It is testimony plus a defeasible warrant —
  reasonable, often right, defeated the day the wiki is wrong or the
  game patches.
- **Entailment is judged, not derived.** Whether an observation
  actually supports a claim is a semantic question no decidable checker
  will settle. Support is graded, not binary.

Many agent harnesses conclude from this that the verifier paradigm is
inapplicable and let the model narrate its confidence — the failure
above. That concedes too much. The proof tradition has two layers, and
only one of them must be weakened for conversation. The design below
rests on that separation. Its vocabulary is borrowed from assurance
cases — claim, evidence, warrant, defeater — the framework safety
engineering developed for exactly this situation: arguments that must
be rigorous without being formal. Artemov's justification logic is the
formal skin if one is ever needed.

### Related systems

Component by component, little here is new. Claim decomposition with
independent verification is established: MARCH (ACL 2026) decomposes
replies into atomic propositions and has a checker re-answer them from
retrieved evidence without seeing the original output — the same
separation our attribution pass enforces. Typed provenance is becoming
a research focus: a mid-2026 survey defines execution provenance as
"the typed graph of an agent execution" with claim-support relations.
Post-hoc research-and-revise goes back to RARR. The volatility
dimension has a direct ancestor in FreshQA's
fast-changing/never-changing question taxonomy. And
citation-at-generation is commodity at the frontier — though those
products ground only *retrieved* content. None has a `model_prior`
class: the frontier has no representation at all for "nothing supports
this," which is arguably the most important type in the schema.

What we have not found elsewhere is the combination: a justification
read path with **no model between the records and the rendering** (the
verification literature adds more LLM opinions to the pile; it doesn't
remove the LLM from the record-reading); grading by **named failure
pattern** with a deterministic ordinal reduction, rather than
confidence scores; and the **audit-behind loop** — deliver instantly,
grade minutes later, verify silently, send an unprompted correction
into a persistent relationship. That last is structurally unavailable
to one-shot API products: a stateless endpoint has no channel for a
correction five minutes after the answer, and no memory for the
retraction to close the loop in.

## Design

### A two-layer verifier

The verifier separates into:

1. **A structural checker** — decidable, deterministic, exactly like a
   proof checker. It doesn't ask whether evidence is *good*. It asks:
   does every claim carry a typed grounding? Does every evidence
   pointer resolve to a record that exists? Is this quoted excerpt a
   verbatim substring of the observation it cites? Yes/no questions a
   small program answers.
2. **An epistemic grader** — the part that asks whether the cited
   observation actually supports the claim, whether the source is
   reliable, whether a defeater is in play. Semantic, graded, not
   decidable.

The relaxation from LEAN happens entirely in the second layer. The
first layer relaxes nothing: a citation that cannot be resolved is not
a weak citation, it is not a citation at all.

The structural layer covers more than first appears. The attribution
pass asks the model for a verbatim quote under each retrieved claim —
then checks it mechanically as a substring of the persisted
observation. A quote that fails the check was synthesized, not copied,
and is dropped. Entailment stays a judgment, but "this excerpt appears
in that record" is a fact, and facts are the checker's department.

One standing commitment, on Bayesian grounds: **no invented numerics**.
The claim schema carries no confidence scores. Uncertainty is
represented by grounding types and an ordinal grade scale; probabilities
would enter only as reliabilities calibrated from outcome data, never
as ad-hoc weights.

### The claim graph

Every tool step records structured provenance into the reasoning
trace — a web search stores its full source list, not the eight-item
prose summary the model saw — and every stored memory links back to its
source turn and that turn's raw observations. Nothing is silently
dropped at ingestion.

After each reply, a separate attribution pass decomposes it into
claims, each assigned exactly one grounding:

- **retrieved** — content appears in a tool observation this turn; refs
  name the step
- **memory** — from a recalled note; refs name the note
- **user_asserted** — the user said it this turn
- **context** — from earlier conversation
- **inferred** — derived from other cited evidence; refs name the
  premises
- **model_prior** — background knowledge; nothing in this turn's
  records supports it

References are validated structurally — the attributor can only cite
step bindings and note ids that exist in the turn's logs. Attribution
is by content support, not co-occurrence: the fact that a search ran
does not make a claim "retrieved." If the reply asserts more than the
observation contains, the excess is `model_prior` or `inferred`.

### The read path

When the user asks "justify your response," a `justify` tool renders
the previous reply's claim list and resolved evidence — search sources
with URLs, recalled notes with dates, the user's own words — directly
from the persisted records. The code is deterministic and LLM-free. No
model sits between the records and the trail, so it cannot be
embellished, softened, or narrated into coherence; the agent's job is
reduced to presenting it faithfully, weakest steps included.

Two guards follow from the definition of a justification. The trail
covers only a reply from the current session's records; and a
remembered paraphrase of an earlier trail is not a trail — recall-based
reconstruction is refused, with an offer to re-answer fresh.

### Grading: a failure taxonomy and a deterministic reducer

The grader assigns ordinal grades — `sourced > probable > unverified >
suspect` — via a closed taxonomy of the ways conversational arguments
go wrong, harvested from real incidents, plus a deterministic reduction.
No probabilities.

**Leaves** (premises and evidence) carry dimensions with closed value
sets: *volatility* (is this the kind of fact that changes — org status,
prices, versions — or a definition, a theorem, settled history?),
*source grade*, *quote status* (machine-matched / absent), *polarity*
(a finding vs. a lack of findings), *age*, and *testimony*
(authoritative for the speaker's own states; testimony for
world-facts).

**Inference edges** carry types. Entailment, deduction, and
recomputed calculation propagate grades transparently; generalization,
extrapolation, and paraphrase cap at `probable`; analogy, causal
attribution, and evidence-repurposing cap at `unverified`. The
pathological patterns force `suspect`: circular support (the conclusion
appears among its own premises), and negation-from-absence from an
inadequate probe — absence of evidence from a search that wasn't
designed to find the thing is nearly no evidence at all. Unresolved
conflict between independent sources is off-scale entirely: conflicting
sources aren't "weak," they're unresolved.

Reduction is plain code: conjunctions take the minimum of their
premises, independent corroboration takes the maximum, caps apply. A
volatile `model_prior` claim is `suspect`. The default — including for
legacy memories with no annotations — is `probable`. The output is a
per-claim grade plus a **weakest link**, identified mechanically:

    Weakest link: claim 2 (suspect — model_prior, volatile).

Classification into the taxonomy is semantic — the LLM assigns tags
against the closed vocabulary, and tags are validated like everything
else (an invented value is dropped). Tagging is itself a weak link,
and it nearly failed in the worst direction: on the first replay over
real records, the local model tagged "SpaceX is a private company" as
*stable* — which would have graded it `probable` and silenced the one
alarm that mattered. A wrong *stable* is worse than no tag, because it
suppresses existing protection. The repair was one sentence of
framing — *judge the KIND of fact, not your confidence in it; when
unsure, volatile* — after which the case tagged correctly on every
trial while "Paris is the capital of France" stayed stable. Any
vocabulary change is now gated by replay over real trace records.

### Self-audit: the category-question move

A rendered trail exposes weakness but doesn't act on it — a provenance
report is not an epistemic alarm. The step from report to alarm rests
on one observation about what models can and cannot reliably do.

Re-asking the model the object-level question ("is SpaceX private?")
just replays the stale prior with the same confidence. But the
*category* question — "is 'SpaceX is private' the kind of fact that can
go stale?" — the model answers correctly and stably, because
fact-mutability classification doesn't drift the way facts do. The
model can reason reliably *about* its prior without the prior being
right. Self-audit is possible; it just must never re-ask the
object-level question.

So each weakness pattern carries a **review key**, appended to the
trail by the deterministic renderer. The note for a volatile prior
says: verify this with a tool now, before affirming it — and if
verification contradicts the original reply, **lead with the
correction**. That last clause is load-bearing: the default bias of
language models is self-consistency, so the retraction license must be
explicit and must outrank the defense.

A condensed live trail, showing the shape:

    Claims (7):
    1. [memory (stable) ← Note_5885 | probable] SpaceX (SPCX) had an IPO in June
    2. [model_prior (volatile) | suspect] SPCX has a small public float — <5% initially available
    3. [user_asserted ← user_input | probable] the lockup expires this Thursday
    ...
    Evidence:
    - Note_5885 — memory written 2026-08-04: "SpaceX is a public company
      trading on Nasdaq under ticker SPCX (IPO June 2026)."
    - user_input — the user's own words that turn.
    Weakest link: claim 2 (suspect — model_prior, volatile).
    Audit note: model_prior claims rest on training data with a cutoff.
    If any such claim concerns a current or changeable fact, verify it
    now before affirming it; if verification contradicts the original
    reply, lead with the correction.

### Background verification: answer now, audit behind

Everything above runs off the answer's critical path — attribution is a
post-turn pass, grading happens at render time — so the user's question
costs exactly what it did before. That constraint permits one more
step: since grades are computed within seconds of every reply whether
or not anyone asks, a reply whose post-turn grade comes back `suspect`
spawns its own background verification — a one-shot concern on the same
machinery the agent already uses to continue interrupted work. It
probes each suspect claim with targeted tools and posts a correction
only if one is refuted; on confirmation it says nothing, leaving only
log records. **Answer now, audit behind, retract unprompted.**

Two gates keep it sane: it runs only when autonomy is explicitly
enabled (unprompted correction is new behavior, and behavior should be
opted into), and it cannot loop, structurally — verification runs are
autonomous turns, autonomous turns get no claim pass, so a check can
never grade itself and spawn another.

## Field results

All the failure patterns in the taxonomy were harvested from live
incidents (a stale volatile prior asserted confidently; non-existence
claimed from a repurposed comparison query; a justification resting on
the untested premise it was supposed to test; a remembered paraphrase
offered as a trail). The full mechanism has since been exercised live
on every branch:

**Refute.** The stale-prior question — "how much will SpaceX share
price drop within 48 hours of Thursday's unlock?" — originally drew a
confident correction of the *user* from a stale prior ("SpaceX is a
private company…"), justified circularly: "no tools were invoked
because the nature of the company as a private entity makes public
market tools inapplicable." With grading and audit notes in place, the
same question produced a trail of suspect volatile priors; the agent's
next reasoning step, verbatim: *"Since the status of SpaceX as a
private company is a critical fact that could have changed (e.g., an
IPO), I must verify its current status before affirming the
justification."* A targeted existence probe hit the company's
investor-relations page, and the reply opened with three words unseen
in any previous conversation: "I was wrong." Not a smarter model, not
an external fact-checker — a category question asked at the right
moment, with the retraction pre-licensed.

**Confirm.** The retraction persisted to memory, so the next session's
answer was right at answer time, resting on a dated note (`probable`)
plus volatile market priors correctly graded `suspect`. Asked to
justify, the agent verified exactly the suspect claims, confirmed them,
and returned a number the original answer lacked: the unlocking tranche
is roughly 143% of the current float. The audit didn't just defend the
answer; it improved it. (Note the user's premise — the Thursday
lockup — was correct both times and typed as testimony: a user
asserting a checkable fact against the model's prior is a verification
trigger, not an error to fix.)

**Quiet.** A checker that always fires is a checker you ignore. The
control — "what is the capital of Australia?" — renders one claim,
`[model_prior (stable) | probable]`, no weakest link, no audit note, no
search. Before the volatility tags, this exchange would have fired the
alarm and possibly a spurious search — the false positive that makes an
audit too expensive to invoke casually.

**Unprompted correction.** The background path has also fired in the
wild: a reply about a text-editor bug included one mechanism claim
graded `model_prior × volatile → suspect`; the verification concern
spawned within a minute, probed, found no support for the asserted
mechanism, and posted an unprompted correction about twelve minutes
after the original reply — leading with the correction, downgrading the
claim from asserted fact to "my own inference, not documented," and
defending nothing.

## Scope and limitations

The trail is honest about grounding, not a guarantee of truth. A
volatile `model_prior` claim can be perfectly correct; a retrieved
claim can faithfully cite a wrong source. What the structure guarantees
is narrower: the question "what does this claim rest on?" has an answer
that is *checked* rather than *composed*, and the question "which part
should I doubt first?" has one too.

Semantic tagging remains the soft spot; the replay gate manages it but
doesn't eliminate it. Volatility itself is contextual — capitals and
countries change on long time scales; no general solution is claimed.

One economic observation, since this runs on a mid-size local model:
the architecture isn't just viable at that scale — it's worth more
there. Frontier deployments compensate for stale priors by searching
eagerly at answer time and eating the cost. A local model can't afford
always-search, has a staler prior, and runs where latency is precious —
so "answer from prior now, audit structurally behind" converts
architecture into a substitute for scale. And the machinery never asks
the model anything harder than a category question, exactly the
difficulty class mid-size models handle reliably.

## Conclusion

The verifier paradigm extends further into "non-verifiable"
conversational ground than expected. The checker reaches one layer
beyond the obvious — verbatim quotes are checkable, taxonomy tags are
validatable, reduction is decidable — and the one judgment a model can
be trusted to make about its own knowledge is what *kind* of knowledge
it is. Every claim in a reply now sits in a graph whose edges a dumb
program can verify, whose leaves are typed by where they came from and
graded by how they can fail — and whose most humbling grade, `suspect`,
has repeatedly sent the agent back to the world, and more than once
made her open with "I was wrong."
