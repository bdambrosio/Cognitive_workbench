# Opportunity Summary — AI implementation claims audit

Draft 2026-08-23. Document 1 of the set in `business-case-plan.md`.

**The decision this supports:** whether to spend three months building this
into a business.

---

## The operation

One question, asked systematically, for a fixed fee:

> **The seller says the software does X. Does it?**

An independent audit of a target's stated technical claims against its actual
implementation, with citations to both — the document making each claim and
the file and line that settles it. The deliverable is a report of what holds,
what does not, and explicitly what was not checked and why that matters, plus
a one-page summary a non-technical buyer reads in thirty seconds.

**$5,000, fixed fee, roughly one week.** Deliberately the rung below full
technical due diligence, which commonly runs $10K–$30K and covers
architecture, scalability, security, team, technical debt and IP. This covers
one thing and refuses the rest.

## Why the problem exists now

The cost of producing plausible software — **and plausible claims about
it** — has fallen much faster than the cost of establishing that those claims
are true. That gap is the business.

It is not a thesis I have to argue from first principles. Regulators have
already named it:

- The **FTC** required Workado to substantiate or stop its claim that its AI
  detector was **98% accurate**. Independent testing cited in the complaint
  put it at **53% on general-purpose content** — FTC staff said it "did no
  better than a coin toss."
- The **SEC** has an enforcement term for it — *AI washing* — and has charged
  advisers who said they used AI in ways they did not.
- **GAO**, reporting on federal AI procurement in April 2026, found agency
  officials had *"difficulty accessing AI technical experts, like data
  scientists, to evaluate contractor proposals."* A large institutional buyer
  saying, in its own words, that it cannot check what it is being sold.

And the buy side is visibly adapting: deal counsel are adding AI-specific
representations on ownership, data governance and performance claims, and at
least one major consultancy is building AI replicas of acquisition targets'
software to test how defensible they really are. Technical AI diligence
already exists at the top of the market. **What does not exist is a version a
$2M transaction can afford.**

## The beachhead

Small M&A — someone buying a software company for one to five million
dollars, whose pitch contains sentences like *"our proprietary AI does X"*,
*"the system handles 90% of Y automatically"*, *"the architecture scales
readily."*

That buyer is frequently well able to judge the economics of the acquisition
and completely unable to judge whether those sentences mean anything. There
is a real information asymmetry, a decision deadline, no internal technical
expertise, and independence has value in itself. $5K against a seven-figure
transaction is small enough not to become its own decision.

Two adjacent markets look plausible and are not the beachhead: **contracted
software acceptance** — *did we receive what we paid $200K to have built?* —
and **internal development audit**. The second is weakest, because the
prospect immediately asks why their own CTO cannot do it.

## Why this person

I have been on the other side of this transaction. I co-founded CleverSet in
2001 to commercialize statistical relational learning, built it into an
e-commerce personalization service deployed to 70+ online retailers, and sold
it to Art Technology Group (NASDAQ: ARTG) in 2008, then served as VP and
Chief Architect for OnDemand Personalization there. **I have written the
technical claims that a buyer's diligence team examined**, which is a useful
thing to have done before auditing someone else's.

Before and alongside that: PhD from Berkeley, Professor Emeritus of Computer
Science at Oregon State, four decades in AI, foundational work on inference
in Bayesian networks, and program co-chair of the Conference on Uncertainty
in Artificial Intelligence. Defense R&D on Bayesian situation assessment and
distributed data fusion under DARPA/AFRL and STTR programs.

The through-line matters more than the length: **my research field is
reasoning under uncertainty from evidence.** That is what this audit is.

For the past year I have been building the machinery — an agent that reads a
data room, grades every factual claim by how it was obtained, and refuses to
report one that cannot produce its source line. Confidentiality is
architectural rather than promised: each engagement runs in an isolated world
with no path to any other client's materials.

## What exists today, honestly

- **A written method** — 16 sections covering scope, priority order, finding
  format, verdict vocabulary, correction protocol, recommendation taxonomy,
  the run sequence and the deliverable contract. Versioned and reviewable.
- **A measurement suite** — a synthetic data room with a published answer
  key, and a scored pass/fail threshold covering the six things a delivered
  report is actually judged on. Eleven runs across two model backends; seven
  clear the bar.
- **A designed deliverable** — the one-page summary, with all five
  recommendation levels and the scope disclaimer built in.
- **One real engagement** — a full audit of a robotics codebase: 31 claims
  verified, **zero deltas**.

## The claim I cannot yet make

The economic proposition is that **the audit finds decision-relevant things a
reasonably diligent buyer would otherwise miss.** Nothing above establishes
it. The one real-target engagement found nothing wrong — which demonstrates
rigour and not value — and the findings I can point to are on a fixture whose
defects I planted myself.

That is the gap, and it is the first thing three months would have to close.

## What three months would buy, and what would decide it

**The experiment is small.** Not a market study — *can I find three strangers
who will pay $5,000 for this?*

Sequenced:

1. **A sample audit that finds something consequential**, on a real target
   with checkable claims. Without it there is nothing to sell.
2. **The fifteen-minute test.** Hand that report to someone who buys this kind
   of work and time whether they can explain back what it found and why it
   matters. If they cannot, the format is wrong and no amount of rigour fixes
   it.
3. **Ten conversations** with small-M&A buyers — search funds, micro-PE,
   individual acquirers — about whether this is a purchase or a nice idea.
4. **Three paid engagements**, or a clear reason nobody will pay.

**Continue if** three strangers pay, and the audits find things the buyers
say changed their decision.

**Stop if** the sample audit cannot find anything consequential on a real
target, or ten qualified conversations produce no purchase intent. Those are
different failures — the first says the product does not work, the second
says the market does not want it — and both are cheap to reach.

**What this needs from me:** `[FILL — how many engagements a year is
realistic against everything else, since the $5K economics turn on it. Ten is
$50K; twenty is $100K.]`

The honest summary: the underlying failure mode is real and recognized, the
price point is plausible, the machinery mostly exists, and **the single thing
that would make this a business has not been demonstrated.** Three months is
roughly what it costs to find out.
