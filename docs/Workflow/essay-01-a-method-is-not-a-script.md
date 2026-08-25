# A Method Is Not a Script

*Draft. First in a series on agent workflows. Concerns what a workflow
specification **is**; the companion piece concerns what it takes to keep an
agent executing one.*

Ask most people what an "agent workflow" is and you get a diagram: boxes,
arrows, a start and an end. Fetch the documents, extract the claims, compare
against the code, write the report. The arrows are the workflow. The agent is
the thing that walks the arrows.

The workflow document I run in production reaches the agent as about twenty
thousand characters across fourteen sections, and contains no arrows,
deliberately. It never says what to do next.

The distinction I want to make is this:

> **A script specifies transitions. A method specifies acceptance.**

Every clause in a script answers *what happens next?* Every clause in a method
answers *what would make this acceptable?* Those are different kinds of
information, they fail in different ways, and only one of them survives
contact with an executor that has judgement.

## What is actually in the document

The method is for a technical due-diligence audit. One operation, stated in
§1: *stated claims vs. observed implementation, with citations.* Sold to
buyers doing sub-$5M software acquisitions.

Sorting it by *kind of information* rather than by section number, almost
nothing is procedural:

**A scope boundary, expressed as negative space.** §2: audit what the seller
asserts about the target against what the materials show — and *do not opine
on what the target should do*, because that is the buyer's judgement and is not
what was bought. Most of the force here is in the prohibition. The document spends
more effort on what the auditor must not say than on what it must.

**A priority order, with its rationale attached.** §4 ranks work: claims whose
failure ends the business, architectural invariants, operational parameters,
low-impact claims. Then it says why: the order exists *so that an audit cut
short by budget, time or access has resolved the most expensive unknowns
first.* This is not a
sequence. It is a sort key, plus the reason the key is that way, so an
executor facing an unanticipated ordering question can derive an answer rather
than look one up.

**An output schema.** §5 gives findings a shape — the claim, the evidence with
document and line, the gap — and then a second shape for a finding that tests no
stated claim but follows by arithmetic from two figures the seller supplied
separately. Basis, Derivation, Consequence, Escalates.

**Closed vocabularies.** §6 fixes the verdict terms. §9 fixes the report-level
recommendation to five values and no others. An auditor who writes "pause; do
not close pending verification" has violated §2 by recommending a buyer
action, and the closed vocabulary is what makes that violation visible instead
of arguable.

**Acceptance conditions.** §1a's two rules: state the coverage wherever you
state the recommendation, and *never write a sentence that implies you examined
more than you did* — "the system does what it says" is a claim about
everything; "of the 43 claims examined, 39 hold" is a claim about the work
done. §16 specifies the deliverable. These describe a finished
state, not a route to it.

**An audience model, which changed the shape of the deliverable.** §8 is a
claim about a reader: they read the Gap Map in about thirty seconds and open
the report only if it earns the time; *they will not read forty cited findings
— they read the recommendation, then drill into one or two if it makes them
curious.*

That is not a formatting preference, and you can see what it cost. The
deliverable is **two documents, not one** (§16) — a Gap Map and a report,
separated by a marker on its own line, produced together. The thirty-second
reader must not have to locate the summary inside a two-thousand-word report,
so the summary stopped being a section and became a separate artifact. The
Gap Map then closes with *"full report with citations available on request"*:
an index into the evidence rather than a précis of the conclusions.

One observation about a reader, propagated into document count, delivery
order, and a machine-checkable boundary between the two.

**A liability posture, which is where the closed vocabulary gets its teeth.**
§10 places the work in the category of expert due-diligence rather than legal
advice: *"here is what I observed, here is the gap between claim and
implementation, here is my professional assessment of what that gap means for
the value of the asset"* — the business judgement stays with the client. A
financial auditor reporting a material misstatement, not a lawyer telling you
what to do about it.

That posture is abstract until §9 makes it operational, in the sharpest
sentence in the document:

> The audit says "Conditional: the seller claims 30-day retention, the
> materials show 7." Whether 7 days is acceptable is the buyer's call. The
> audit does not say "you should walk."

Two vocabularies, held apart on purpose. *Clear / Clear with caveats /
Conditional / Material / Walk* describes the state of the claims. *Proceed /
negotiate / walk* is the buyer's action vocabulary. An auditor who writes "do
not proceed as presented" has stopped reporting and started advising — the
over-claim §2 forbids, the one §10 says carries liability, and the one an
enumerated type makes visible instead of arguable. The liability posture is
not commentary sitting beside the rules. It is why the rules have the shape
they have.

And it is withheld from the executor on purpose. §10 is marked for the human
running the practice, and the loader strips it before the method reaches the
model — not to save tokens, but because "here is how you would be sued" in an
auditor's prompt invites defensive hedging in a document whose value is plain
statement. The posture shapes the rules; the rules are what get delivered. A
method can be written by one audience and addressed to another, and being
deliberate about which sections cross that line turns out to matter as much as
what the sections say.

**A learning channel.** §14, which I will come back to.

None of that is control flow. All of it constrains the output.

## The one section that looks like a script, and why it isn't

§12 is titled "Running an audit: sequence," and it is numbered 1 through 7.
Fair enough. But look at what the numbered items actually say.

Step 5 is: *"Stop when what remains is low-risk — when the claims still
unchecked are low-priority ones, and those checked so far have held
consistently. Say in the coverage statement where you stopped and why."*

There is no number in that. It does not say read nine documents, or verify
forty claims, or spend two hours. It describes a **condition to be
evaluated**, and evaluating it requires knowing what has been found so far,
how severe it was, and what kind of thing is likely to be left. A script
cannot contain that step. A script would have to replace it with a counter,
and the counter would be wrong for every target that isn't the one it was
tuned on.

The same is true of step 3, which says only *prioritise in §4's order* and
leaves how far to go to the same judgement step 5 describes.

So §12's steps are not instructions to execute. They are **states to reach**,
in an order, each with a satisfaction condition. What sits between them is
supplied by the executor.

## Addressed to a practitioner, not to an interpreter

This is the underlying difference, and it is old.

GAAP is not a script. A clinical protocol is not a script. An ISO procedure is
not a script. They are all addressed to a competent practitioner who brings
judgement, and their function is to constrain that judgement into a form the
profession can defend — to make an output *reviewable by someone who was not
there*.

A script assumes the executor has no judgement and must be told everything. A
method assumes the executor has judgement and tells it what the judgement is
for.

Which of those you write depends entirely on what you think you are handing
the work to. For thirty years the answer was "a machine, so write the script."
The reason this is worth revisiting is not that methods are new. It is that
the executor changed, and most people are still writing for the old one.

## The negative space is doing real work

An underrated property of a method: it is defined as much by what it declines
to specify.

The document does not say how many turns to take, which document to open
first, how to recover from a failed tool call, when to stop reading and start
writing, or how to allocate a limited budget across the claim surface. Not
oversights — every one of those is either a judgement the executor should
make, or a concern belonging to the harness that runs it.

The thing a method must not do is over-specify, because an over-specified
method is a script with extra words: it commits to a route without knowing the
terrain, and when the terrain differs the executor has no basis to deviate. §3
makes this explicit — *"scope adapts to the target; the method does not."*

A method can also require something the executor cannot do, and nothing will
report it. §12 once required that on finding a delta the auditor **stop and
confirm with the client before continuing** — *the audit is a collaboration,
not a surprise.* Good practice, and unreachable: the harness has no channel to
the client mid-engagement. Every run silently failed a requirement nobody could
meet.

**An unperformable clause produces no error. It reads as rigour and does
nothing.** The only thing that finds one is asking, of each clause, whether the
executor can perform it — which is a different review from asking whether each
clause is right, and it is the one nobody thinks to run.

## Where specification runs out

A method can specify what makes an output acceptable. It cannot specify facts
about the engagement it is applied to, and that boundary is sharper than it
looks.

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

The fix is not a better definition. It is a different kind of clause. A claim
is *an assertion the seller makes to the buyer* — which makes source code,
comments and a data room's evidence documents into evidence rather than claims
— and **the engagement names which documents carry those assertions.** The
auditor is told, not asked to infer.

With three of the nine documents named, the same three models enumerate 33, 70
and 88. Still a spread, and a harmless one: all three read only the named
documents, and two cite an *identical* set of twenty-one document lines. They
agree about where the claims are and differ about how finely to slice them. The
first was a defect in the criteria. The second is grain, and it costs nothing
as long as each report is internally consistent and states its own denominator.

So there is a third kind of information in a workflow document, alongside
transitions and acceptance conditions: **a parameter the engagement supplies
and the method must not invent.** Assurance practice has known this for a long
time — criteria are agreed in advance, not derived by the practitioner
mid-engagement.

The general form: **a method's reach ends where its author's knowledge of the
specific engagement ends.** Past that boundary more specification does not
converge behaviour. It produces confident divergence, which is worse than
visible divergence, because every executor believes it complied.

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
port number in it does not. Read a proposed edit for proper nouns before
including it.

This is what professional firms actually do — a methodology manual updated
after engagements, not analysts carrying client details in their heads. It
routes learning through a human review gate, which is where a confidentiality
check belongs, and it lands somewhere versioned and diffable.

It also has a failure mode the document names: method learning can make the
method *worse*, by over-fitting to recent targets. The guard is a fixed
benchmark re-run after any method change, plus keeping at least one fixture in
a domain unlike recent real work.

## What this buys, and the limit I can now state precisely

The honest limit first, because it is the most interesting thing I have
learned and it cuts against the case I am making.

**A specification constrains form. It does not confer competence.**

§5 gives a derived finding its own shape because, without one, a finding that
tests no stated claim has nowhere to go: models reach the evidence and stop
short of stating the conclusion. Give it a home and they use it — the shape is
now used in essentially every run.

What they put in it still varies. Across the three arms I run, the two planted
derived findings are recovered 1 of 2, 2 of 2 and 1 of 2. Same document, same
format, same slots, three different degrees of actually doing the arithmetic.
Whatever produces that difference is not the specification, because the
specification was identical.

So giving the finding a home was necessary and nowhere near sufficient. Anyone
selling you "we improved the prompt and the model got smarter" is describing
something that did not happen.

What it does buy is more modest and more durable: an output that can be
**checked by someone who was not there.** A closed recommendation vocabulary
makes a §2 violation mechanical rather than arguable. A finding format with
mandatory citations makes an unsupported claim detectable. A coverage
statement makes silence about scope impossible to pass off as completeness.
Every one of those is a property of the artifact, verifiable without access to
the process that made it.

That is the real dividing line. **A method's requirements are checkable
against the artifact. A harness's requirements are only visible in the
trace.**

## Where this stops

Everything above concerns the specification as an object — what kind of
information it holds and what it can constrain. It says nothing about the
considerable problem of keeping an agent going long enough to satisfy it.

And the boundary leaks in a way worth naming here, because it is the strongest
argument that these are genuinely two subjects rather than one.

For most of this document's life the agent fetched it with a tool, as an
observation inside a turn. Observations are stored capped. So what actually
reached the model was a fragment that **ended mid-sentence in §2** — section
two of sixteen.

Everything this essay has been describing — the finding format, the closed
vocabularies, the coverage rule, the deliverable contract — was in a file the
executor had never read past the scope rule. Nothing errored. No log line said
"method truncated." The agent produced audits, and they looked like audits.
The failure was invisible from both ends: the document was complete on disk,
and the model gave no sign of working from a fragment.

Moving the same file into the static system prompt made §3–§16 reachable, and
three contract elements appeared in output for the first time — among them
§9's recommendation taxonomy, which had never survived to the turn that needed
it, and §16's deliverable contract, which until that point existed only inside
the runner's brief and not in the method at all.

Identical content. Different delivery. Different behaviour.

So the method's content is inert until a harness delivers it, the harness can
silently truncate it, and neither the document nor the model will tell you
this is happening. The specification and its execution are separable as
subjects and inseparable in practice, and the second one is where I have spent
far more of my time.

---

*Next: the runner — continuation, stopping rules, and the state an agent
structurally cannot hold.*
