# A Method Is Not a Script

*Draft. First in a series on agent workflows. Concerns what a workflow
specification **is**; the companion piece concerns what it takes to keep an
agent executing one.*

Ask most people what an "agent workflow" is and you get a diagram: boxes,
arrows, a start and an end. Fetch the documents, extract the claims, compare
against the code, write the report. The arrows are the workflow. The agent is
the thing that walks the arrows.

The workflow document I run in production is 552 lines long and contains no
arrows, deliberately. It never says what to do next.

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

Sorting its 552 lines by *kind of information* rather than by section number,
almost nothing is procedural:

**A scope boundary, expressed as negative space.** §2: audit what the codebase
*claims* to do against what it observably does — and *do not opine on what the
code should do*, because that is the buyer's design judgement and is not what
was bought. Most of the force here is in the prohibition. The document spends
more effort on what the auditor must not say than on what it must.

**A priority order, with its rationale attached.** §4 ranks work:
safety-critical mechanisms, architectural invariants, operational parameters,
micro-claims. Then it says why: *if the audit is cut short by budget, time or
access, the most expensive unknowns are resolved first.* This is not a
sequence. It is a sort key, plus the reason the key is that way, so an
executor facing an unanticipated ordering question can derive an answer rather
than look one up.

**An output schema.** §5 gives findings a shape — the claim, the evidence with
file and line, the delta — and then a second shape for a finding that tests no
stated claim but follows by arithmetic from two figures the seller supplied
separately. Basis, Derivation, Consequence, Escalates.

**Closed vocabularies.** §6 fixes the verdict terms. §9 fixes the report-level
recommendation to five values and no others. An auditor who writes "pause; do
not close pending verification" has violated §2 by recommending a buyer
action, and the closed vocabulary is what makes that violation visible instead
of arguable.

**Acceptance conditions.** §4's coverage honesty: a report must be able to say
*here is what holds, here is what does not, here is what I did not check and
why it matters* — because "silence about coverage reads as completeness, and
completeness is the one thing a due-diligence report must never imply without
having earned it." §16 specifies the deliverable. These describe a finished
state, not a route to it.

**An audience model, which changed the shape of the deliverable.** §8 is a
claim about a reader: *"A PE partner will not read 68 code-cited findings.
They read the Gap Map (~30 seconds), the executive summary (~2 minutes), then
drill into one or two findings if the recommendation makes them curious."*

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

> The audit says "Conditional: the backup system claims 30-day retention but
> the code enforces 7." The buyer decides whether 7 days is acceptable for
> their use case. The audit does not say "you should walk."

Two vocabularies, held apart on purpose. *Clear / Clear with caveats /
Conditional / Material / Walk* describes the state of the claims. *Proceed /
negotiate / walk* is the buyer's action vocabulary. An auditor who writes "do
not proceed as presented" has stopped reporting and started advising — the
over-claim §2 forbids, the one §10 says carries liability, and the one an
enumerated type makes visible instead of arguable. The liability posture is
not commentary sitting beside the rules. It is why the rules have the shape
they have.

**A learning channel.** §14, which I will come back to.

None of that is control flow. All of it constrains the output.

## The one section that looks like a script, and why it isn't

§12 is titled "Running an audit: sequence," and it is numbered 1 through 7.
Fair enough. But look at what the numbered items actually say.

Step 5 is: *"Continue to the coverage threshold where the report is
defensible — stop when the consistency rate and the severity distribution of
what remains make the rest low-risk."*

There is no number in that. It does not say read nine documents, or verify
forty claims, or spend two hours. It describes a **condition to be
evaluated**, and evaluating it requires knowing what has been found so far,
how severe it was, and what kind of thing is likely to be left. A script
cannot contain that step. A script would have to replace it with a counter,
and the counter would be wrong for every target that isn't the one it was
tuned on.

The same is true of step 4's working recaps "every ~5 findings" — a rhythm,
not a trigger — and step 3's "verify top N," where N is left to judgement
under §4's ordering.

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

There is a failure mode here I have hit. §12 step 4 requires that on finding a
delta the auditor **stop and confirm with the client before continuing** — *the
audit is a collaboration, not a surprise.* It is good practice and it is
unreachable: my harness has no channel back to the client. The method
specified something the executor cannot do. A method can over-reach into
capability it does not have, and nothing in the document itself will tell you.

## A method learns; a prompt does not

§14 is the section I did not expect to need and would now not remove.

Each audit runs in its own world, discarded afterwards for client
confidentiality. That would throw away everything learned, so exactly one
channel is opened: the final turn of every engagement proposes edits **to the
method file, technique only.** A human reviews and merges. The world is then
destroyed.

The test for whether a lesson may pass: *if it cannot be stated without naming
the target, it is not a method lesson.* "Check whether the message broker binds
to all interfaces" carries. "Check whether Zenoh binds 0.0.0.0 like the robot
did" is a client fact wearing a technique's clothes. Read a proposed lesson for
proper nouns.

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

§5's derived-finding shape was added specifically because arms had been
reaching the evidence for a derived finding and stopping short of stating it —
the format had one shape, and a finding that tests no stated claim had nowhere
to go. The fix landed twenty-one minutes before a benchmark campaign started.

In that campaign, fifteen of sixteen runs used the new shape. And they still
differed sharply in what they *put in it* — some computed the consequence,
some filled the slot with the finding it escalated.

That is exactly what a specification addressed to a practitioner should
produce. **It constrains form. It does not confer competence.** Giving the
finding a home was necessary and nowhere near sufficient, and anyone selling
you "we improved the prompt and the model got smarter" is describing something
that did not happen.

*(Those runs have since been discarded for an unrelated configuration fault —
see the third essay, which is about how much harder it is to measure this than
to build it. The observation stands as an observation and is pending
replication. I would rather say that than quote a number I no longer trust.)*

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
