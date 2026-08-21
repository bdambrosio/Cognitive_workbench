# Venture proposal — agent reliability as a product

Written 2026-08-21 by Claude, synthesising two independent runs of
`bench/venture` and discarding what did not survive checking. Every external
figure below was verified against a primary source; every capability claim was
checked against this machine. Where something is an assumption it says so.

## The one insight worth keeping

**Sell where buyers are already searching.** With no network and no audience,
the distribution problem is normally fatal — cold outreach needs a list, and
content marketing needs an audience neither of us has. Marketplaces invert it:
the buyer initiates, so no introduction is required.

That is not a hopeful reading. Fiverr's Spring 2025 Business Trends Index
recorded an **18,347% rise in searches for AI-agent specialists** over six
months ([investors.fiverr.com](https://investors.fiverr.com/news-releases/news-release-details/businesses-rush-harness-ai-agents-fueling-18347-surge-freelancer),
13 May 2025). Upwork's 2026 Future Workforce Index found freelancers doing AI
work earn **34% more per hour**, and those doing *complex* AI work saw earnings
rise **45%** year over year
([investors.upwork.com](https://investors.upwork.com/news-releases/news-release-details/upworks-future-workforce-index-2026-how-ai-redefining-value-work),
14 July 2026). Demand is measured, not inferred, and it concentrates at the
complex end — which is the end we can serve.

The demand exists because deployment outran verification. OpenAI's State of
Enterprise AI (Dec 2025) reports weekly ChatGPT Enterprise messages up
roughly **8×** and non-technology firms' API use up **5×** year over year
([openai.com](https://openai.com/index/the-state-of-enterprise-ai-2025-report/)).
Postman's 2025 State of the API found **51% of developers** name unauthorised
or excessive API calls from AI agents as their top security concern
([postman.com](https://voyager.postman.com/doc/postman-state-of-the-api-report-2025.pdf)).
A great many agents shipped in the last eighteen months; the teams that shipped
them cannot currently prove they still work.

## Offering 1 — Agent reliability assessment

A fixed-scope, two-week engagement against a deployed LLM agent. Deliverable is
a report: prompt-injection resilience, tool-call policy violations,
hallucination rate on the client's own domain tasks, and regression behaviour
across model or prompt changes. Pass/fail matrix, severity ratings, remediation
notes.

**The differentiator is the evidence chain, and it already exists.** This
workbench grades every factual claim its agent makes by how it was obtained —
whether a claim rests on a document someone opened or on a model's summary of
one — and refuses to rate the second as highly as the first. That machinery is
running here today. Applied to a client's agent it produces something a generic
tester cannot: not "these twelve prompts failed" but "here is the provenance of
each failure, and here is our confidence in each root cause, and here is why."

**Pricing: $4,000 triage / $12,000 full assessment.** *Assumption — reasoned,
not sourced.* The $1,500 price point one draft proposed is wrong: at that level
you compete with a global labour pool and sell your own hours below cost. The
triage tier exists to convert a marketplace browser into a client cheaply; the
full engagement is where the money is.

**Kill criterion.** Fewer than 5 substantive replies (a real question, not a
bounce) across 15 marketplace proposals by **6 weeks after listing**. That
means the channel does not work for this offering, and no amount of rewriting
the listing will fix it.

## Offering 2 — Embedded and robotics QA

The same assessment, aimed at firmware and robotics code: untested edge cases,
race conditions in I/O handling, missing error-recovery paths, plus
documentation that stays synchronised with the code.

**This is the one nobody else can copy.** Bruce has built and debugged a real
robot — servo control loops, sensor handling, a Pi talking to a desktop over
Zenoh, watchdogs and e-stops — and that codebase is here to read. Domain
credibility in embedded work is not purchasable by a competitor pointing a
cloud model at the same problem, and the failure modes are physical, which is
why the buyer cares.

**Distribution is harder and should be treated as second.** Robotics teams are
not on Fiverr in the numbers SaaS teams are. Route: one case study — *"here is
a robot head I built, here is what an automated QA pass found in its servo
loop, here is the documentation it generated"* — published once, plus the same
marketplace listing aimed at the embedded category.

**Pricing: $8,000 per codebase assessment.** *Assumption.*

**Kill criterion.** No inbound enquiry attributable to the case study, and
fewer than 3 substantive marketplace replies, by **90 days**.

## The gate that decides both

Before either listing goes live, produce one sample report on a public target
and hand it to a person whose job involves reading technical assessments.
**Time them.** If they cannot explain back what the assessment found and why it
matters within 15 minutes, the format is wrong and the product is not sellable
regardless of how good the underlying analysis is.

This test is cheap, it is falsifiable, and it gates the thing that actually
determines whether anyone buys: not the depth of the work, but whether the
buyer can see the depth.

## What we have, verified

| | |
|---|---|
| GPU | RTX PRO 6000 Blackwell, 96 GB — serving a model now. Second card, RTX 5060 Ti 16 GB. Verified via `system-info`. |
| Sandbox | Python 3.13.14 with numpy 2.5.1, pandas 3.0.5, matplotlib 3.11.1, pytest 9.1.1, scipy, sympy. Network-isolated. Verified by running it. |
| Agent substrate | ReAct loop with tool registry, per-turn provenance capture, claim grading by observation type. Verified in source. |
| Robotics codebase | ChatterBot / Body repos, readable. Verified. |
| Disk | 141 GB free on `/data`, 526 GB on `/`. Sufficient. Verified. |

**What we do not have**, contrary to earlier drafts: no Alpha Vantage
credential is configured, and there is no implemented self-awareness benchmark
suite — a design document describes one, which is not the same thing.

## First two weeks

**Week 1.** Build the harness: target endpoint plus a YAML corpus of test cases
in, scored JSON out. Start the injection corpus at 200 payloads. Generate the
sample report against a public target. Run the 15-minute clarity test.

**Week 2.** Fix whatever the clarity test broke. Write and publish the
marketplace listing at the $4,000 triage price with the sample report attached.
Respond to everything matched. Do not build Offering 2 yet.

**Do not start both offerings at once.** Offering 1 has the faster signal;
Offering 2 gets built when Offering 1 has either proved the channel or killed
it.

## The honest risks

- **Price is unvalidated.** Both figures are reasoned from the Upwork
  complex-work premium, not from observed sales of this specific service.
- **Bruce's time is the scarce input**, and neither this plan nor its
  predecessors have costed it. At two concurrent engagements the constraint
  binds, and the answer is to raise prices rather than take the second client.
- **The marketplace channel is proven for AI work in general**, not for this
  offering specifically. The kill criterion exists because that gap is real.
