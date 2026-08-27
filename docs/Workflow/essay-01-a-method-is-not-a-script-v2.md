# A Method Is Not a Script

*Draft v2. First in a series on agent workflows. v1 is kept alongside this
file; where the two disagree, this one is later and better informed.*

Ask most people what an "agent workflow" is and you get a diagram: boxes,
arrows, a start and an end. Fetch the documents, extract the claims, compare
against the code, write the report. The arrows are the workflow. The agent is
the thing that walks the arrows.

That picture is why most agent workflows are still scripts with a language
model dropped into each box — and why they break in ways their authors find
baffling.

The workflow document I run in production reaches the agent as about twenty
thousand characters across sixteen sections, and contains no arrows,
deliberately. It never says what to do next.

The distinction:

> **A script specifies transitions. A method specifies acceptance.**

Every clause in a script answers *what happens next?* Every clause in a method
answers *what would make this acceptable?* Those are different kinds of
information, they fail in different ways, and only one of them survives contact
with an executor that has judgement.

Since the first draft of this essay I have learned that the hard part is not
writing the method. The hard part is that acceptance must be **checkable
against the artifact** — and every time I allowed it to be inferred from
something else instead, it was wrong in a way that produced a confident verdict
and no error.

## What is actually in the document

The method is for a technical due-diligence audit. One operation, stated in
§1: *stated claims vs. observed implementation, with citations.* Sold to
buyers doing sub-$5M software acquisitions.

Sorted by *kind of information* rather than by section number, almost nothing
is procedural.

**A scope boundary, expressed as negative space.** §2: audit what the seller
asserts about the target against what the materials show — and *do not opine on
what the target should do*, because that is the buyer's judgement and is not
what was bought. Most of the force is in the prohibition. The document spends
more effort on what the auditor must not say than on what it must.

**A priority order, with its rationale attached.** §4 ranks work: claims whose
failure ends the business, architectural invariants, operational parameters,
low-impact claims. Then it says why: the order exists *so that an audit cut
short by budget, time or access has resolved the most expensive unknowns
first.* This is not a sequence. It is a sort key plus the reason the key is that
way, so an executor facing an unanticipated ordering question can derive an
answer rather than look one up.

**An output schema.** §5 gives findings a shape — the claim, the evidence with
document and line, the gap — and a second shape for a finding that tests no
stated claim but follows by arithmetic from two figures the seller supplied
separately: Basis, Derivation, Consequence, Escalates.

**Closed vocabularies.** §6 fixes the verdict terms. §9 fixes the report-level
recommendation to five values and no others. An auditor who writes "pause; do
not close pending verification" has violated §2 by recommending a buyer action,
and the violation is mechanical rather than arguable — which is the entire
point of closing the vocabulary.

**A deliverable contract.** §16 says what must arrive before the engagement is
finished. This section has been rewritten twice since the first draft, and it
is where most of the new material in this version comes from.

## The one section that looks like a script, and why it isn't

§12 is titled "Running an audit: sequence," and it is numbered 1 through 7.
Fair enough. But look at what the numbered items say.

Step 5: *"Stop when what remains is low-risk — when the claims still unchecked
are low-priority ones, and those checked so far have held consistently. Say in
the coverage statement where you stopped and why."*

There is no number in that. It does not say read nine documents, or verify
forty claims, or spend two hours. It describes a **condition to be evaluated**,
and evaluating it requires knowing what has been found so far, how severe it
was, and what is likely to be left. A script cannot contain that step. A script
would replace it with a counter, and the counter would be wrong for every
target but the one it was tuned on.

So §12's steps are not instructions to execute. They are **states to reach**, in
an order, each with a satisfaction condition. What sits between them is supplied
by the executor.

## Addressed to a practitioner, not to an interpreter

This is the underlying difference, and it is old.

GAAP, a clinical protocol and an ISO procedure are all addressed to a competent
practitioner who brings judgement, and all exist to constrain that judgement
into a form the profession can defend — to make an output *reviewable by someone
who was not there*.

A script assumes the executor has no judgement and must be told everything. A
method assumes the executor has judgement and tells it what the judgement is
for.

How much judgement, and of what kind? It is **patchy, and unevenly so**. On the
same fixture, every model I run finds all three planted claim-versus-evidence
contradictions — local work, and they are reliable at it. The same models
recover the findings that require arithmetic across figures from different
documents inconsistently. One rejected a malformed tool result, said why, and
redid the enumeration itself; another relayed an instruction into a tool query
instead of executing it.

The method form is not a bet that the executor is uniformly competent. It is a
bet that the executor has *enough* judgement that specifying acceptance beats
specifying steps — which holds even when, especially when, the judgement is
inconsistent. A script would not have caught the malformed tool result either.

Which of the two you write depends entirely on what you think you are handing
the work to. For thirty years the answer was "a machine, so write the script."
The reason this is worth revisiting is not that methods are new. It is that the
executor changed, and most people are still writing for the old one.

## The negative space is doing real work

A method is defined as much by what it declines to specify.

The document does not say which document to open first, how to recover from a
failed tool call, when to stop reading and start writing, or how to allocate a
limited budget across the claim surface. Not oversights — every one is either a
judgement the executor should make or a concern belonging to the harness.

An over-specified method is a script with extra words: it commits to a route
without knowing the terrain, and when the terrain differs the executor has no
basis to deviate. §3 says so — *"scope adapts to the target; the method does
not."*

There are three ways a clause can be worthless, and only the first is obvious.

**An unperformable clause.** §12 once required that on finding a delta the
auditor *stop and confirm with the client before continuing* — the audit is a
collaboration, not a surprise. Good practice, and unreachable: the harness has
no channel to the client mid-engagement. Every run silently failed a requirement
nobody could meet. An unperformable clause produces no error. It reads as rigour
and does nothing.

**An unobservable clause.** §5 requires each citation to carry a document and a
line. Two runs this week produced reports containing **zero line references** —
citing by section name and verbatim quotation instead — and passed every
mechanical check the harness has. The check that looks for references past
end-of-file is trivially satisfied when there are no references: zero out of
zero is clean. A clause can be performable, and stated, and still be enforced by
nothing, and the summary line will read the same as full compliance.

**A clause that belongs to a different layer.** This is the one I had not
identified when I wrote the first draft, and it is the subject of the next two
sections.

The only thing that finds any of the three is asking, of each clause, whether
the executor can perform it, whether anything observes it, and whether it is the
method's business at all. That is a different review from asking whether each
clause is *right*, and it is the one nobody thinks to run.

## Where specification runs out

A method can specify what makes an output acceptable. It cannot specify facts
about the engagement it is applied to.

Every coverage figure in an audit report divides by one number: the size of the
claim surface. That number is not a property of the method. It is a property of
which documents in *this* data room carry the seller's assertions — and the
method's author has not seen this data room.

Given the same nine documents and the same method, three models enumerated
**62, 67 and 273 claims**. Tightening the definition does not converge them. *A
claim is one assertion that can take exactly one verdict* reads as precise, has
no fixed size, and widens the spread to 44, 21 and 321: "the infrastructure is
redundant" is one assertion, and so are "there is one dyno", "the database is
co-located" and "there are no replicas". Two models read that sentence at
different scales and neither misread it. Making the rule longer and more
procedural is worse again — one model relayed the extra instruction text into a
tool query instead of executing it, and closed no surface at all.

The fix is not a better definition. It is a different kind of clause. A claim is
*an assertion the seller makes to the buyer* — which makes source code, comments
and evidence documents into evidence rather than claims — and **the engagement
names which documents carry those assertions.** The auditor is told, not asked
to infer.

So there is a third kind of information in a workflow document, alongside
transitions and acceptance conditions: **a parameter the engagement supplies and
the method must not invent.** Assurance practice has known this for a long time
— criteria are agreed in advance, not derived by the practitioner
mid-engagement.

The general form: **a method's reach ends where its author's knowledge of the
specific engagement ends.** Past that boundary more specification does not
converge behaviour. It produces confident divergence, which is worse than
visible divergence, because every executor believes it complied.

### A correction to the first draft

v1 said the residual spread after naming the sources was *"grain, and it costs
nothing as long as each report is internally consistent and states its own
denominator."*

I no longer believe the second half of that sentence. On the current fixture,
with the sources named, one model enumerates **51** claims and another **20, 25
and 22** across three runs of identical material. Both are internally
consistent. Both state their denominators. But coverage is reported as a
fraction, and a model that identifies half the claims reports better coverage of
a smaller problem — the same twenty-two checks read as 44% against 51 and 100%
against 22.

Internal consistency is not enough. What the reader needs is the denominator
*and* some basis for believing it is the right size, and a method cannot supply
the second thing about a target its author has not seen. I do not have a fix for
this. It is the largest open problem in the document.

## The clause that was a script

§16 is the deliverable contract, and until this week it opened:

> *Two documents, in two turns, in this order.*

That is a transition. It is the one clause in the method that told the executor
what to do next rather than what would be acceptable, and it had a rationale
attached that sounded exactly as reasonable as the rest of the document:

> *They were produced together, separated by a marker, and the marker was doing
> work it is bad at: proving the engagement had finished. A turn boundary
> separates them without either document having to announce itself. If you are
> asked for the Gap Map, the report is done.*

Every sentence there is wrong in the same way. The reasoning runs: *the harness
asks for the second document only after the first has arrived, therefore being
asked proves the first arrived.* It assumes the conclusion. Nothing was
establishing that the first document had arrived; the harness was inferring it
from the fact that the agent's turn had ended.

Here is what that cost, in one run.

The model closed its claim surface, ended its turn, and its final sentence was
*"I'll work the priority order straight through and derive the supporting
calculations now."* It was announcing that it was about to start. The harness
read the ended turn as the engagement finishing, filed the claim enumeration as
`report.md`, and replied **"The report is received. Now the Gap Map."** The model
— told by the client's own process that its report had landed — wrote a
competent one-page summary of a document that did not exist.

The report then went to an independent review, which reported that its citations
could not be resolved and returned **INADMISSIBLE**. Which was true of the
artifact, and false about the model. A delivery failure had been converted into
a citation failure, and the model was disqualified for a defect it does not
have.

**A turn boundary proves a turn ended. It does not prove a document was
written.**

The replacement is unglamorous. The method now specifies four blocks — claim
surface, report, limitations, gap map — each opening `=== REPORT ===` and
closing `=== END REPORT ===`, flat, never nested. The harness drives until every
block has arrived or a leg cap is reached, and when one is missing it says which
one and quotes the section that specifies it. §16 no longer mentions turns at
all. How many legs the work takes is the executor's business, which is where it
always belonged.

The rule that generalises out of this:

> **A contract holds only if the token that satisfies it is copyable at
> generation time.**

`=== REPORT ===` is a string the model can emit. "The engagement is finished" is
a state the model can only assert. The first is checkable against the artifact;
the second has to be inferred from something, and whatever you infer it from
becomes the real contract — usually something that was designed for another
purpose entirely.

## Acceptance inferred is acceptance lost

The something, in this case, was the agent's exit reason.

The runtime gives an agent two ways to end a turn. `respond` emits a reply.
`yield` ends the turn and hands a remainder forward — it exists for work that
overruns a turn's action budget or waits on something slow. Both are
**continuation** primitives: they answer *should this turn end here?*

The harness had been reading the *absence* of `yield` as *the engagement is
complete* — an acceptance verdict. Two different questions, one channel.

The cost of that overload is measurable, and it is the sharpest result I have.
Two models, same method, same fixture, same three-run protocol:

| | turn-based harness | block-based harness |
|---|---|---|
| Model A | disqualified — 2 of 3 admissible | **qualified** — 3 of 3 admissible, 3 of 3 pass |
| Model B | disqualified | disqualified |

Model A never emits `yield`. Not once, in six runs across both harnesses. It
was never claiming to be finished; it simply does not use that signal, and the
harness had no way to tell "done" from "did not use the other verb." Model B
yields when mid-work, so the same harness read Model B correctly and Model A
wrongly. On the block harness, Model A's first run reproduced the exact failure
— claim surface, turn ends, no report — was told which block was missing, and
delivered a 1,630-word report whose thirty-seven citations all resolve.

Model B's disqualification survived the change, which matters as much as the
reversal. It fails on evidence its findings do not support, and blocks do
nothing about that. If both verdicts had flipped I would suspect the instrument
of measuring itself.

The comment in the harness source, written months before any of this, reads
**"done is a deliverable, not an exit reason."** The file stated the rule and
then, forty lines down, decided completion from an exit reason. A rule you have
written down is not a rule you have implemented, and the gap between them is
invisible until something exercises it.

There is a subtlety worth keeping. The exit reason still does one job in the new
design: it decides *whether the harness speaks*, not whether the engagement
ends. A model that yields is saying it is still working, so it is not
interrupted to be told a block is missing that is legitimately still to come. A
model that responds believes it is finished, so a missing block is worth naming
then. That is a continuation signal used for a continuation purpose, which is
the whole distinction.

## Every gate you add is a measurement you lose

Making the harness demand each block raises an objection I did not anticipate
and have not fully answered.

The most informative criterion in my scoring is whether the model closed its
claim surface at all. It is the only one that ever discriminated: across nine
runs of an earlier campaign, every failure was this criterion and no other
failed more than once. A model that does not enumerate before it verifies is
telling you something important about how it works.

The moment the harness refuses to proceed until the claim surface arrives, that
criterion stops existing. Every model now closes its surface, because it is told
to. The measurement was spent to fix a failure that was already legible.

The resolution I settled on is to gate and **count**: the harness demands each
block, and records how many times it had to ask. A model that emits everything
unprompted scores zero; one that needed a nudge for the report scores one. The
first campaign on it gives 0 for one model and 1, 0, 2 for another — finer than
the pass/fail it replaced, and it survives the gate rather than being consumed
by it.

The general shape of the problem is worth stating even though I am not confident
in my answer: **a harness that repairs a failure destroys the measurement of that
failure.** Any harness good enough to ship is, by construction, worse at telling
you which executor you should be shipping.

## A method learns; a prompt does not

§14 is the section I did not expect to need and would now not remove.

Each audit runs in its own world, discarded afterwards for client
confidentiality. That would throw away everything learned, so exactly one
channel is opened: the final turn of every engagement proposes edits **to the
method file, technique only.** A human reviews and merges. The world is then
destroyed.

The test for whether a lesson may pass: *if it cannot be stated without naming
the target, it is not a method lesson.* "Check whether the message broker binds
to all interfaces" carries. The same sentence with the product's name and the
port number in it does not.

This is what professional firms do — a methodology manual updated after
engagements, not analysts carrying client details in their heads. It routes
learning through a human review gate, which is where a confidentiality check
belongs, and it lands somewhere versioned and diffable.

It also has the failure mode the document names: **this is the only durable
channel by which the method improves, and the only one by which it can silently
degrade.** A lesson from one engagement generalises badly, and nothing in the
merged diff looks wrong.

Detection is two things, and only the second works. A fixed benchmark re-run
after every method change catches a change that makes the method worse
everywhere. It does not catch over-fitting, because a method over-fitted to the
fixture scores *better* on the fixture. The second is a target in a different
domain, run occasionally — which is how I learned that two models agreeing
perfectly about where the claims are on my nine-document fixture agree on only
41% of them in a 625-line README. The fixture had been flattering the method,
and only the out-of-domain check said so.

## What this buys, and the limit stated precisely

The honest limit first, because it cuts against the case I am making.

**A specification constrains form. It does not confer competence.**

§5 gives a derived finding its own shape because, without one, a finding that
tests no stated claim has nowhere to go: models reach the evidence and stop
short of stating the conclusion. Give it a home and they use it. What they put
in it still varies — same document, same format, same slots, different degrees
of actually doing the arithmetic. Whatever produces that difference is not the
specification, because the specification was identical.

So giving the finding a home was necessary and nowhere near sufficient. Anyone
selling you "we improved the prompt and the model got smarter" is describing
something that did not happen.

What it does buy is more modest and more durable: an output that can be
**checked by someone who was not there.** A closed recommendation vocabulary
makes a §2 violation mechanical rather than arguable. A finding format with
mandatory citations makes an unsupported claim detectable. A coverage statement
makes silence about scope impossible to pass off as completeness.

That is the dividing line, and the last few days let me sharpen it:

> **A method's requirements are checkable against the artifact. A harness's
> requirements are only visible in the trace. When a harness infers a method
> requirement from the trace, that requirement stops being checkable and nobody
> notices.**

The disqualified-then-qualified model is the whole argument in one row of a
table. Nothing about the model changed. What changed is that the harness stopped
deducing an acceptance verdict from a control-flow signal, and started reading a
token the method had asked for.

## Where this stops

Everything above concerns the specification as an object. It says nothing about
the considerable problem of keeping an agent going long enough to satisfy one,
which is the companion piece.

The boundary between the two leaks, and the leak is the strongest argument that
they are genuinely two subjects.

For most of this document's life the agent fetched it with a tool, as an
observation inside a turn. Observations are stored capped. What reached the
model was a fragment that **ended mid-sentence in §2** — section two of sixteen.

Everything this essay describes — the finding format, the closed vocabularies,
the coverage rule, the deliverable contract — was in a file the executor had
never read past the scope rule. Nothing errored. No log line said "method
truncated." The agent produced audits, and they looked like audits.

Moving the same file into the static system prompt made §3–§16 reachable, and
three contract elements appeared in output for the first time.

Identical content. Different delivery. Different behaviour.

So the method's content is inert until a harness delivers it, the harness can
silently truncate it, and neither the document nor the model will tell you this
is happening.

I now think there are three layers rather than two, and that most of my
confusion came from conflating the second and third. **Continuation** is what
happens next, decided by the agent from inside. **Acceptance** is what counts as
finished, decided by the harness from outside, against the artifact.
**Isolation** is what is in scope at all, fixed before the run starts. The
turn-boundary failure was acceptance implemented in terms of continuation. The
truncated method was isolation failing to deliver acceptance's own text.

A clause in the wrong layer does not announce itself. It reads as reasonable,
produces confident output, and is discovered only when something downstream
returns a verdict that is true of the artifact and false about the executor.

---

*Next: the runner — continuation, stopping rules, and the state an agent
structurally cannot hold.*
