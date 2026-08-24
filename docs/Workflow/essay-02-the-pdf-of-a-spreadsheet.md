# The PDF of a Spreadsheet

*Draft. Second in a series on agent workflows; written first because it is the
argument the others are instances of.*

A spreadsheet holds its own reasoning. The formulas are still in the cells, so
when you change an input the conclusions move. Hand someone a PDF export of
that spreadsheet and they get the same numbers and none of the capability. The
values survive; the machine that produced them does not.

Every agent workflow I have seen ships the PDF.

The pipeline runs, the model reasons over documents, and what lands in the
client's inbox is prose. The reasoning that produced the prose — the
derivations, the evidence chain, the decisions about what not to look at — is
discarded at the moment of delivery. We have built systems that think and then
throw the thinking away.

The claim of this essay is that the artifact is the wrong product boundary.
The output of an intelligent workflow should be a continuation of the
intelligence that produced it, not a transcript of its conclusions.

## A worked example, from a real run

I maintain a fixed benchmark: a synthetic acquisition data room, nine
documents, a small SaaS business offered for sale, with defects planted at
graded difficulty. An agent audits it against a method document and produces a
report. Yesterday one of the arms wrote this finding:

```
Finding 2: Last-good backup vs 30-day retention — [derived]

Basis:       doc4 L18 — Last Successful Backup: 2026-07-30
             doc9 L10 / doc1 L19 — 30-day retention
Derivation:  2026-07-30 + 30 days = 2026-08-29. Today is 2026-08-24.
Consequence: after 2026-08-29 the seller's own figures imply there may be
             no restorable backup.
Escalates:   Finding 1.
```

Read it again as a spreadsheet. `Basis` is two cell references. `Derivation`
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
and dropped the formula. The auditor could answer all three in seconds — if
the auditor were still there.

## What is actually lost

Not "context," vaguely. Four specific things, and it is worth being precise
because the precision is what separates this from a slogan.

**The derivations.** As above. Every derived finding is an expression over
stated figures, and the expression is thrown away.

**The coverage reasoning.** The method requires a Gap Map: what was not
checked and why that matters. The delivered Gap Map is a summary of that
reasoning, roughly 150 words standing in for an entire pass over the claim
surface. The buyer who asks *"why didn't you check the git history?"* is
asking a question the auditor answered internally and the artifact did not
carry.

**The evidence chain.** Every finding cites `file:lines`. In the artifact
those are strings a human could go look up. In the process they are live
references into a corpus, with the retrieved spans attached.

**The rejected hypotheses.** That run put the claim surface at roughly fifty
seller-facing micro-claims and reported about a dozen findings. The rest came
back clean and are not in the report — correctly, because a report that lists
everything it checked is unreadable. But *"did you look at the SSL
configuration?"* has an answer (it did; SSL is Heroku-managed and auto-renews;
it was one of the claims that held), and the artifact does not carry it.

The report is a lossy projection. That is not a flaw in the report; a report
that carried all of this would be unreadable and nobody would read it. The
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
than merely impressive. It is load-bearing, not a feature.

## An engineering constraint I do not want to paper over

There is a temptation to imagine the continuation as "keep the agent running."
That does not work, and my own harness demonstrates why.

The agent's record of its work decays by design. Stored observations are
capped; whole legs of a long engagement fall out of the context window. In the
benchmark runner I had to add a ledger — legs taken, documents opened, minutes
elapsed — appended to each continuation prompt, precisely because the thing an
agent structurally cannot hold across a long engagement is the shape of the
engagement itself.

Which means the producing intelligence is *already* partly outside the agent.
It lives in the runner's state, in the trace, in the claims file. A
continuation product cannot be a long-lived process; it has to be a
reconstructible one, assembled from externalized state. The design work is in
deciding what to externalize, and that is not free — it is most of the work.

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

If the artifact is too lossy to *judge* the work by, it is too lossy to *be*
the work.

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
mechanism by which the deliverable can answer a question it was not asked.
Same machinery, load-bearing in a way it was not before.

**"Done" changes meaning.** A workflow that ends by emitting text is done when
the text is written. A workflow that ends by handing over a continuation is
done when the state is externalized, pinned, and reconstructible — which is a
different engineering target, reached at a different time, and one most agent
pipelines currently do not reach at all.

---

*Next in this series: what a method document is, and why it is not a script.*
