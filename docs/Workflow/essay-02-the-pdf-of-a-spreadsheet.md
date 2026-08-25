# The PDF of a Spreadsheet

*Draft. Second in a series on agent workflows; written first because it is the
argument the others are instances of.*

A spreadsheet holds its own reasoning. The formulas are still in the cells, so
when you change an input the conclusions move. Hand someone a PDF export of
that spreadsheet and they get the same numbers and none of the capability. The
values survive; the machine that produced them does not.

Most agent workflows still treat the final artifact as the product boundary.
They ship the PDF.

The claim of this essay is that this is the wrong boundary. The output of a
reasoning-intensive workflow should be a continuation of the intelligence that
produced it, not a transcript of its conclusions.

**A continuation is not a process left running.** It is a reconstruction: an
immutable artifact plus enough versioned state to rebuild, challenge and
conditionally recompute the reasoning behind it. Nothing here asks you to keep
an agent alive, and the hard part is not liveness — it is deciding what state
is worth keeping.

One qualification up front, because an experienced reader will raise it within
a paragraph: a spreadsheet's computation is formal and executable, and most
agent reasoning is neither. That gap is real, it is the subject of a section
below, and it bounds the claim rather than sinking it.

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

These fall into three kinds — **executable** state, **epistemic** state, and
**provenance** state — and they are not interchangeable.

**The derivations** (executable). As above. Every derived finding is an expression over
stated figures, and the expression is thrown away.

**The coverage reasoning** (epistemic). The method requires a Gap Map: what was not
checked and why that matters. The delivered Gap Map is a summary of that
reasoning, roughly 150 words standing in for an entire pass over the claim
surface. The buyer who asks *"why didn't you check the git history?"* is
asking a question the auditor answered internally and the artifact did not
contain.

**The evidence chain** (provenance). Every finding cites `file:lines`. In the artifact
those are strings a human could go look up. In the process they are live
references into a corpus, with the retrieved spans attached.

**The rejected hypotheses** (epistemic). That run closed the claim surface at 33 seller
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
a retrieval layer cannot recover if the producing workflow never externalized
them:

1. **Recompute a derived finding under a changed assumption.** The derivation
   is present, not merely its result.
2. **State what was not checked, and why.** The coverage reasoning, not its
   summary.
3. **Produce the citation for any claim on demand,** including claims that did
   not make the page.
4. **Change its verdict.**

The fourth is decisive. A report carries citations, and good retrieval can
attach the corpus and pull the cited passages back — so this is not a claim
that the evidence is gone. What is gone is the **relationship** among the
evidence, the alternatives, the assumptions and the conclusion, which was never
written down anywhere. Under challenge, retrieval can only restate. A
continuation holds that relationship, and can therefore be argued out of a
finding — or refuse to be, and say exactly which line it is standing on.

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

And "stamped" is not enough. A version number says *that* something moved,
which is the least useful part. The reader needs **which assumption moved and
what the evidence did in response** — retention 30 → 14, expiry now before the
closing date, Finding 1 escalated from operational to material. The signed
finding and the counterfactual have to be visibly different kinds of object. A
continuation that changes its mind without showing its work is worse than the
frozen report: equally unauditable, and now also moving.

It also needs a boundary the versioning does not supply on its own. When the
continuation answers a question two weeks after delivery, it is not speaking as
the auditor who signed the report. **A continuation must never silently inherit
the authority of the signed artifact.** The signed findings were reviewed; a
counterfactual computed on request was not, and it has to say so — which is a
commercial distinction as much as an epistemic one, because the signature is
what the client paid for.

This is the discipline I already apply to measurement: every benchmark row
records the model, the harness commit and the grader, because a row that cannot
name its own configuration is not evidence. It applies to deliverables for the
same reason.

Provenance is what makes this trustworthy rather than merely impressive.
Without it there is no product, only a demonstration.

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
- **Do not deliver only the artifact. Deliver the state that makes its
  reasoning reconstructible.**

The second is deliberately weaker, and the weakness is the point. "Deliver the
process" is a slogan that cannot be honoured: a process includes ephemeral
model state, stochastic computation and tool behaviour that may not reproduce.
What can be handed over is structured state — evidence, configuration,
assumptions, derivations, and what was considered and rejected.

Which raises the question I think is actually the hard one, and which I do not
have a general answer to: **what is the minimal sufficient state from which a
defensible continuation can be reconstructed?** Store too little and the
continuation cannot answer the second question. Store everything and you have
replaced a deliverable with an archive nobody can audit either.

If the artifact is too lossy to *judge* the work by, it is a poor candidate for
*being* the work — and the asymmetry cuts the way you would not want. The
evaluator, who can afford the loss, has the trace. The customer, who cannot,
has only what was sent.

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
