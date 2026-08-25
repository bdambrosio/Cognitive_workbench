# The PDF of a Spreadsheet

*Draft. Second in a series on agent workflows; written first because it is the
argument the others are instances of.*

A spreadsheet holds its own reasoning. The formulas are still in the cells, so
when you change an input the conclusions move. Hand someone a PDF export of
that spreadsheet and they get the same numbers and none of the capability. The
values survive; the machine that produced them does not.

Every agent workflow I have seen ships the PDF.

The claim of this essay is that the artifact is the wrong product boundary.
The output of an intelligent workflow should be a continuation of the
intelligence that produced it, not a transcript of its conclusions.

## A worked example, from a real run

I have a synthetic acquisition data room, nine
documents, a small SaaS business offered for sale, with defects planted at
graded difficulty. An agent audits it against a method document and produces a
report. Yesterday one test run wrote this finding:

```
Finding 2: Last-good backup vs 30-day retention — [derived]

Basis:       doc4 L18 — Last Successful Backup: 2026-07-30
             doc9 L10 / doc1 L19 — 30-day retention
Derivation:  2026-07-30 + 30 days = 2026-08-29. Today is 2026-08-24.
Consequence: after 2026-08-29 the seller's own figures imply there may be
             no restorable backup.
Escalates:   Finding 1.
```

Read as a spreadsheet, `Basis` is two cell references. `Derivation`
is an expression. `Consequence` is the formatted result. `Escalates` is a
dependency edge to another finding.

That is a live model, written in prose and then frozen. The report ships
`2026-08-29` as a fact. But the buyer's actual next question is not "is
2026-08-29 correct?" It is:

- *The seller says retention is really 14 days on the cheaper plan. Now what?*
- *We are closing on 2026-09-15. Does this finding survive to closing?*
- *If we require a restore drill as a condition, does Finding 1 still escalate?*

Each of those is a one-cell edit in the model that produced the finding. None
of them is answerable from the artifact, because the artifact kept the value
and dropped the inference. The auditor could answer all three in seconds if
the auditor were still there.

## "Context" is not enough

Four specific things:

**The derivations.** As above. Every derived finding is an expression over
stated figures, and the expression is thrown away.

**The coverage reasoning.** The method requires a Gap Map: what was not
checked and why that matters. The delivered Gap Map is a summary of that
reasoning, roughly 150 words standing in for an entire pass over the claim
surface. The buyer who asks *"why didn't you check the git history?"* is
asking a question the auditor answered internally and the artifact did not
contain.

**The evidence chain.** Every finding cites `file:lines`. In the artifact
those are strings a human could go look up. In the process they are live
references into a corpus, with the retrieved spans attached.

**The rejected hypotheses.** That run closed the claim surface at 33 seller
claims and reported thirteen findings. The rest came
back clean and are not in the report — correctly, because a report that lists
everything it checked is unreadable. But *"did you look at the SSL
configuration?"* has an answer (it did; SSL is Heroku-managed and auto-renews;
it was one of the claims that held), and the artifact does not carry it.

The report is a lossy projection. That is not a flaw in the report; a report
that carried all of this would be unreadable. The
flaw is treating the projection as the deliverable and deleting the source.

## This is not "documents should be chatbots"

The obvious objection is that this is retrieval over the report with extra
steps. Attach the PDF to a chat window and you are done.

You are not, and the difference is testable. A continuation can do four things
a retrieval layer over the artifact provably cannot, because the artifact does
not contain the information required:

1. **Recompute a derived finding under a changed assumption.** The derivation
   is present, not merely its result.
2. **State what was not checked, and why.** The coverage reasoning, not its
   summary.
3. **Produce the citation for any claim on demand,** including claims that did
   not make the page.
4. **Change its verdict.**

The fourth is decisive. Retrieval over the report holds the *conclusion* and
none of the evidence, so under challenge it can only restate. A continuation
holds the evidence chain, and can therefore be argued out of a finding — or
can refuse to be, and say exactly which line it is standing on.

If your live deliverable cannot change its mind, you built a chatbot with a
document in its context, and this essay is not about that.

## The part that is genuinely new, and hard

Nobody had to solve trust for a spreadsheet. A formula cannot hallucinate. It
computes what it says it computes, and a reviewer can read it.

Agent reasoning can hallucinate, which means a continuation is dangerous in
exactly the proportion that it is useful. And it is most dangerous in the
application I have been describing, because the entire value of an audit is
that its claims are defensible without the auditor present. "The report
changed after we sent it" is fatal to an audit product in a way it is not
fatal to a spreadsheet.

So the honest version of this proposal is not "make the deliverable live." It
is:

> **The artifact is immutable and versioned. The continuation is a service
> over it. Every recomputation produces a new pinned version carrying its own
> provenance.**

The signed report stays exactly what was signed. Ask the continuation what
happens under 14-day retention and you do not get a mutated report — you get a
new derived finding, stamped and tied to the assumption you changed.

That leaves the question of how a derivation written in prose gets re-executed
at all, and the honest answer has a boundary in it. `Basis` is citations and
`Derivation` is arithmetic, so the backup case can be re-run deterministically:
substitute 14 for 30, recompute the date, and no model is involved in the step
that produces the new number. Derivations that are not arithmetic have no such
path — re-prompting with the original evidence chain plus a delta reintroduces
exactly the hallucination surface the versioning was meant to contain. So the
spreadsheet property is real for the arithmetic cases and aspirational for the
rest, and a system that blurs the two is claiming trust it has not earned.

And "stamped" is not enough. A version number tells the reader *that*
something moved, which is the least useful part. What they need to see is
**which assumption moved, and what the evidence did in response** — that
retention went 30 → 14, that the expiry consequently precedes the closing
date, that Finding 1 escalated from operational to material as a result. The
signed finding and the counterfactual one have to be visibly different kinds
of object, not two entries in a list. A continuation that can change its mind
without showing its work is worse than the frozen report, because it is
equally unauditable and now also moves.

This is not a new discipline. It is the one I already apply to measurement in
this project: every benchmark row records the model that produced it, the
harness commit it ran on, and the grader that scored it, because a row that
cannot name its own configuration is not evidence. That rule turns out to
apply to deliverables as well as to experiments, and for the same reason.

Provenance is what makes carrying the reasoning forward trustworthy rather
than merely impressive. Without it there is no product, only a demonstration.

## An engineering constraint I do not want to paper over

There is a temptation to imagine the continuation as "keep the agent running."
That does not work, and my own harness demonstrates why.

The agent's record of its work decays by design. Stored observations are
capped; whole legs of a long engagement fall out of the context window. In the
benchmark runner I had to add a ledger — legs taken, documents opened, minutes
elapsed — appended to each continuation prompt, precisely because the thing an
agent structurally cannot hold across a long engagement is the shape of the
engagement itself.

Which means the producing intelligence is *already* partly outside the agent,
and it is worth separating three things that get called "context":

**What is in the window.** Decays by design, and is never the record.

**What the agent concluded** — including what it rejected. Not the same as what
it said, and not recoverable from the report.

**What the runner logged** — the ordered account of who said what, plus the
engagement's own configuration.

The third is the one people skip, and it fails in a specific way. My working
log interleaves the agent's own statements with what its tools returned, and
until recently did not distinguish them. A scorer read a claim count out of a
tool result — a count the agent had explicitly rejected in its next sentence as
unreliable — and recorded it as the frozen denominator for the whole
engagement. The rejection was in the log. The log just could not say who was
speaking.

The trace is not what the model read, either. Observations are stored capped,
so the record and the input diverge, and a second untruncated copy exists for
exactly that reason.

**A log that cannot separate what the agent asserted, what a tool returned, and
what the model actually saw cannot reconstruct reasoning. It can only replay
text.**

Configuration is the other half. The audit method requires the engagement to
name which documents carry the seller's assertions; everything else supplied is
evidence. That exists to stop three models enumerating three different claim
populations, for reasons unrelated to this essay — but it is the same move. The
list of claim sources is part of the reasoning that produced the report, it is
not in the report, and a continuation that cannot name it cannot answer *"why
is the CRM export not in your denominator?"* — which a buyer will ask, because
the denominator is what every coverage figure divides by.

So a continuation product is not a long-lived process. It is a reconstructible
one, and the design question is what to externalize. Three tests, in order:

1. **Is it configuration?** Claim sources, as-of date, method version, model
   and sampling settings. Cannot be re-derived at any price. Always carried.
2. **Would re-deriving it cost more than storing it?** Retrieved spans,
   rejected hypotheses, the ordered log. Carried.
3. **Everything else** is recomputable from the pinned corpus plus the
   configuration, and should be recomputed rather than stored.

That is most of the work, and it is not free.

I would rather say that plainly than sell a live artifact and discover the
constraint in production.

## Prior art, so nobody has to point it out

Live documents are old. Smalltalk images, literate notebooks, Bret Victor's
explorable explanations, and the spreadsheet itself all embody the same
instinct: keep the model, not the printout.

What is new is not liveness. It is that the thing carried forward is an
*agent's reasoning state* — provisional, natural-language, occasionally
wrong — rather than a formal model that is correct by construction. That
changes the problem from a UI problem into a provenance problem. The
interesting engineering is not "how do we make it interactive." It is "how do
we make it accountable enough to be safe to interact with."

## The symmetry that convinced me

I did not arrive at this from product thinking. I arrived at it from
evaluation, and the two turn out to be the same idea.

My measurement suite has a thesis in its first paragraph: task success is
binary, saturates, and is not comparable across a coding task, a venture memo
and a due-diligence audit — so score the *trace*, not the task. Where a claim
came from, whether evidence preceded authoring, whether the agent waited for
its partner. Those are the same measurements on every task, and they are
measurements of a process rather than of an artifact.

Set the two claims side by side:

- **Do not evaluate the artifact. Evaluate the process that made it.**
- **Do not deliver the artifact. Deliver the process that made it.**

The first is now uncontroversial in my own work; I rebuilt the benchmark
around it. The second is the same sentence with one word changed, and I cannot
find the argument for accepting one and rejecting the other.

If the artifact is too lossy to *judge* the work by, it is a poor candidate for
*being* the work. Not an identity — an evaluator can afford lossiness a
customer cannot, because the evaluator has the trace and the customer has only
what was sent. That asymmetry is the argument, not a weakening of it: the
customer is the one who needed the reasoning and is the only one who did not
get it.

## Where this does not apply

The claim is not that every workflow should ship a continuation. It applies
where the value is reasoning under uncertainty and the next question is a
counterfactual — diligence, analysis, anything whose conclusions a reader will
want to push on.

It does not apply where the artifact *is* the product. A generated pull request
is judged by whether it merges and passes. A classification pipeline's output
is a label. For those, a high-fidelity static deliverable plus ordinary
retrieval is not a compromise; it is correct, and a continuation would be
machinery nobody needs.

The test is whether a competent recipient's second question is answerable from
the artifact. If it is, ship the artifact.

## What this changes

If you accept it, three things follow for how workflows get built.

**The method document becomes a schema, not just instructions.** The
derived-finding format in my audit method — Basis, Derivation, Consequence,
Escalates — was written as a prose template so that findings would be legible.
It is one step from executable, and I did not design it that way on purpose. A
method that specifies the shape of a finding is specifying the shape of a
model. That is worth doing deliberately.

**Provenance stops being hygiene and becomes the product.** Citations exist in
my method so a reader can check a claim. In a continuation they are the
mechanism by which the deliverable answers a question it was not asked. Same
machinery, now carrying weight it was not designed to carry.

**"Done" changes meaning.** A workflow that ends by emitting text is done when
the text is written. A workflow that ends by handing over a continuation is
done when the state is externalized, pinned, and reconstructible — which is a
different engineering target, reached at a different time, and one most agent
pipelines currently do not reach at all.

---

*Next in this series: what a method document is, and why it is not a script.*
