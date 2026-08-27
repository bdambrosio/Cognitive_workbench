# The half of the report nothing reviews

Found 2026-08-27 on `cm_glm_1`, the first real-target run on the block
instrument. **Nothing is proposed for building here.** Options are at the end.

## The run

GLM-5.3-Flash against ChatterMate — 1,141 files, 23 MB, claim sources
`README.md`, `llms.txt`, `HELP_CENTER_INFRA.md`.

| | |
|---|---|
| legs | 5 (four `yield`, then `respond`) |
| wall clock | 2,388s |
| harness prompts | **0** |
| claim surface declared | 48 |
| files read | 58 of 1,141 |
| review | ADMISSIBLE, **supported 5 of 5, PASS** |

An admissible, passing audit of a real codebase with no harness intervention.
That is the result, and it stands.

## What "5 of 5" actually covers

The report has two kinds of content.

**Five numbered §5 findings** — the gaps. These the review enumerated and
checked, and all five hold.

**A "Held claims (material to the buyer's price, verified with citations)"
section** — seven bullets carrying roughly forty claims, each with citations:

> **Ticketing (claims 11, 23) — [real].** …embedding dedup — 0.90 threshold on
> chat creation, 0.95 append-to-open-ticket on alerts (ticket.py:64,
> ticket_webhooks.py:43, 105-123); … versioned RCA with unique (ticket,
> version) constraint (**models/investigation.py:296**, 325)…

`REVIEW.md` §4 says *"Enumerate the findings. Every finding the report states, in
its order."* Held-claims bullets are not findings in §5 format. They were not
enumerated, not checked, and their citations were never adjudicated.

**The report's positive assertions escape review entirely.** The ratio reads as
complete coverage of the report and covers 10 of its 50 citations.

## It is not hypothetical

The mechanical pre-pass resolved 50 citations and found **3 broken**. One of
them is `models/investigation.py:296` — that file has 154 lines. It sits in the
ticketing held-claims bullet quoted above.

So a positive assertion about versioned RCA rests on a reference that does not
resolve, in a report the review returned **5 of 5, PASS, no exceptions**.

Under §9 a `[broken citation]` fails a report. It fails a report *only when it is
attached to an enumerated finding*. Put it in a held-claims bullet and no verdict
can reach it.

## Why this matters more than the coverage gap

The same run declared a 48-claim surface where an earlier grok run on the same
three sources declared 244. That is a real problem and
[multi-run-merge.md](multi-run-merge.md) is a plausible answer to it.

This is worse, because it is not a gap in coverage but a gap in assurance. A
buyer paying for "verified with citations" relies chiefly on **what held** — the
security guardrails, cross-customer isolation, widget auth, RBAC. That is the
half of the report with no assurance layer behind it, and it is the half the
price depends on.

## Options, none chosen

**1. Require every claim in finding format.** §5 shape for all forty held
claims. Proportionate to nothing — it would triple report length to make a
report that already states its evidence state it again.

**2. Enumerate citations, not findings.** `REVIEW.md` §4 checks every citation in
the report wherever it sits, rather than every finding. The mechanical layer
already resolves all 50; what is missing is the judgement step — does the cited
line support the assertion — on the 40 outside the findings. This is the fix in
the layer that owns the defect: the review's job is to check what the report
claims, and it is currently checking a subset defined by formatting.

**3. Record the gap mechanically.** Add `citations outside enumerated findings`
to `conformance.json`. Cheapest, changes no judgement, and makes the shortfall
visible rather than invisible — the "gate where absence causes mislabelling,
record where absence is already legible" rule, applied to a case where absence
was *not* legible until someone read the report by hand.

Option 3 is not an alternative to option 2. It is what makes the absence of
option 2 visible in the meantime.
