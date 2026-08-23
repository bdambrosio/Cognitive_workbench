# Business case — the documents to write, and why these

Written 2026-08-23. Supersedes an earlier 18-document outline that drifted
into a diligence binder for a business that already exists. This is the
smaller set an entrepreneur needs to decide whether the thing is worth
pursuing at all.

Three structural corrections carried into this version:

- **Sequence is problem → product → customer → evidence → advantage →
  economics → experiment → decision.** The earlier version led with the
  method. Whether the methodology is rigorous is not the interesting
  question; whether anyone has a problem expensive enough to pay for it is.
- **The three use cases are hypotheses, not channels.** Part of the work is
  deciding whether they are one product sold three ways or three different
  businesses with different buyers, incentives, liability and urgency.
- **Deferred until there is evidence of demand:** legal pack, operating
  model, risk register, engagement history, formal unit economics. Real
  documents, wrong phase.

---

## The thesis, as sharpened

Not "deployment outran verification" — too vague to defend. This:

> **AI adoption and AI-assisted software development have grown faster than
> organizations' ability to verify what systems actually do, whether claimed
> capabilities are real, and whether production implementations satisfy those
> claims.**

Underneath it, one abstraction covering all three use cases:

> **Independent verification of software claims where the producer and the
> evaluator have asymmetric information** — and AI widens the gap because the
> cost of producing plausible software *and plausible claims about it* has
> fallen far faster than the cost of establishing that those claims are true.

## Positioning

**An AI implementation claims audit, $5,000 fixed fee.** Not "technical due
diligence, cheaper" — a narrower thing: give me the claims you are relying
on, the code and documentation, reasonable access; I return which important
claims are supported, unsupported, ambiguous or contradicted by the
implementation, with evidence.

Why the price works:

- Ten engagements a year is $50K; twenty is $100K. A solo practice does not
  need a large market to clear, which removes most of the TAM burden.
- $5K against a seven-figure transaction is small enough not to become its
  own decision.
- Comparable published pricing puts short fixed-scope technical assessments
  in the $2.5K–$30K band, with full diligence commonly $10K–$30K+. A $5K
  engagement sits deliberately below that as the rung under full diligence.

**The central design problem is scope discipline**, not price. $5K cannot
survive three weeks inside an undocumented repository. What the engagement
refuses to do matters more than what it does.

## The three hypotheses, ranked

| rank | use case | risk type | why |
|---|---|---|---|
| **1** | Small M&A / investment verification | **claims risk** — *the seller says X; does it do X?* | Consequential decision, real information asymmetry, no internal expertise, explicit deadline. Independence itself has value. |
| **2** | Contracted software acceptance | **acceptance risk** — *is this what we contracted for?* | $5K is trivial against a $100K–$500K build. Buyer has standing to ask. |
| **3** | Internal development audit | **development risk** — *we built this fast; do we know what we have?* | Real problem, fuzzy buyer. The prospect immediately asks why their own CTO cannot do it. |

Note §11 of `audit/METHOD.md` currently rules out 2 and 3 — *"this method
applies to claims-verification in a transactional context: a buyer evaluating
a target they do not own."* That boundary was drawn deliberately and now
conflicts with the plan. Document 4 has to resolve it rather than ignore it;
an evaluator reading both will see the contradiction immediately.

---

## The twelve documents

Status: ✅ exists · ◐ partial · ○ to write

| # | document | one line | status |
|---|---|---|---|
| 1 | **Opportunity Summary** | Two pages: the business, the customer problem, the beachhead market, why now, and the decision this analysis supports. | ○ |
| 2 | **Product Definition** | Exactly what the audit examines, what evidence it uses, what the customer receives, how long it takes, and what is explicitly out of scope. | ◐ — `audit/METHOD.md` §2, §11, §16 |
| 3 | **Customer Problem / Why Now** | Evidence that buyers face a verification problem existing diligence, code review, testing and security processes do not solve. | ○ — **the document that decides everything** |
| 4 | **Use-Case and Buyer Analysis** | The three hypotheses compared on buyer, trigger, pain, budget, urgency, sales process and required product changes — and whether they are one business or three. | ○ |
| 5 | **Sample Audit / Proof of Product** | One complete engagement: inputs, investigation, findings, evidence, report — with defects consequential enough to show economic value. | ◐ — see below |
| 6 | **Competitive and Substitute Analysis** | Against technical diligence firms, security audits, conventional QA, Big Four, internal engineering review, and the buyer simply using a frontier model. | ○ |
| 7 | **Why Us / Defensibility** | What lets this practice produce a materially better or cheaper result than those alternatives. | ◐ — provenance grading, geofenced access, the measurement record |
| 8 | **Market and Business Model** | Reachable market from the beachhead outward, engagement pricing and frequency, acquisition mechanism, gross economics, repeat potential. | ○ |
| 9 | **Evidence and Validation Record** | What has actually been demonstrated — benchmark results, completed audits, buyer reactions, failures and corrections — with observation separated from hypothesis. | ◐ — `measure/fixtures/dataroom/RESULTS.md` |
| 10 | **Risks and Open Questions** | The assumptions most capable of killing it: willingness to pay, audit accuracy, liability, access, founder dependence, AI commoditization, and whether the three cases share one product. | ○ |
| 11 | **Go-to-Market Experiment** | The smallest set of conversations and paid pilots that tests demand, pricing, deliverable usefulness, and which market is strongest. | ○ |
| 12 | **Decision / Action Memo** | What to do next, resources required, measurable milestones, and explicit continue / pivot / abandon criteria. | ○ |

### On document 5, which is the one that matters

A polished sample is not enough. The sample has to establish the central
economic proposition:

> **the audit discovers decision-relevant things a reasonably diligent buyer
> would otherwise have missed.**

A beautifully rigorous zero-delta audit does not establish that. The Body
audit — the only real-target engagement — found **31 claims verified, zero
deltas**, and is additionally only partially recovered
(`docs/audit-sample-body-report.md`). The synthetic `flowmetrics` fixture
does produce consequential findings, and being synthetic is not the weakness
it first appears: §14 means a real client's report can never be shown to a
prospect, so a fictional target with a published answer key is the only
sample that is ever showable. What it lacks is a real target's credibility.

Resolving this is a prerequisite for documents 3, 6 and 11.

---

## Evidence for document 3

Independently verified here, 2026-08-23:

- **FTC v. Workado.** Advertised its AI content detector as **98% accurate**;
  independent testing cited in the complaint found **53% on general-purpose
  content** — FTC officials said it "did no better than a coin toss." The
  order bars efficacy claims without competent and reliable evidence, and
  requires that evidence be retained.
  [FTC](https://www.ftc.gov/news-events/news/press-releases/2025/04/ftc-order-requires-workado-back-artificial-intelligence-detection-claims)
- **GAO on federal AI procurement (April 2026).** Agency officials reported
  *"difficulty accessing AI technical experts, like data scientists, to
  evaluate contractor proposals."* A major institutional buyer stating the
  acceptance-risk problem in its own words.
  [GAO-26-107859](https://www.gao.gov/products/gao-26-107859)

Supplied and **not** independently checked here — verify before any of it
goes in front of a buyer:

| claim | source |
|---|---|
| FTC / accessiBe, $1M settlement over AI accessibility compliance claims | [FTC](https://www.ftc.gov/news-events/news/press-releases/2025/01/ftc-order-requires-online-marketer-pay-1-million-deceptive-claims-its-ai-product-could-make-websites) |
| FTC / IntelliVision, facial recognition accuracy and bias claims | [FTC](https://www.ftc.gov/news-events/news/press-releases/2024/12/ftc-takes-action-against-intellivision-technologies-deceptive-claims-about-its-facial-recognition) |
| SEC "AI washing" enforcement against advisers | [SEC](https://www.sec.gov/newsroom/press-releases/2024-36) |
| Builder.ai — $400M+ raised, >$1B valuation, 2024 revenue presented at ~$220M against ~$50–55M actual | [Rest of World](https://restofworld.org/2025/builderai-ai-apps-downfall/) · [Bloomberg](https://www.bloomberg.com/news/articles/2025-05-22/startup-builder-ai-overestimated-sales-by-300-to-key-creditors) |
| Veracode: 45% of LLM-generated code samples failed security tests despite being functionally correct | [Veracode](https://www.veracode.com/blog/genai-code-security-report/) |
| No relationship between passing functional tests and security/quality | [arXiv 2508.14727](https://arxiv.org/abs/2508.14727) · [arXiv 2508.21634](https://arxiv.org/abs/2508.21634) |
| GitHub added automatic security/quality validation to Copilot output, then to third-party agents | [GitHub](https://github.blog/changelog/2025-10-28-copilot-coding-agent-now-automatically-validates-code-security-and-quality/) |
| IDC: 88% of AI proofs-of-concept did not reach widespread deployment | [CIO](https://www.cio.com/article/3850763/88-of-ai-pilots-fail-to-reach-production-but-thats-not-all-on-it.html) |
| Deal counsel adding AI-specific reps on ownership, data governance, performance claims | [Reuters](https://www.reuters.com/legal/transactional/ai-defensibility-what-it-means-why-it-matters-how-diligence-deal-documents-are--pracin-2026-06-16/) |
| Bain building AI replicas of targets' software for PE diligence | [FT](https://www.ft.com/content/e5bac4d1-b1f8-43a4-bd54-b182d5357af0) |

**Two cautions on how this evidence is used.**

Do not lead with "95% of AI projects fail." That figure has been repeated far
beyond what the underlying studies support, and most of those failures are
bad business cases, data problems or organizational dysfunction — not
verification gaps. The defensible version is narrower: *AI systems are
unusually easy to demo and unusually hard to characterize under real
operating conditions.*

Do not claim this audit would have caught Builder.ai. Much of that collapse
was financial. What it establishes is smaller and still useful: sophisticated
investors placed enormous sums in an AI software company without
independently establishing how much of the purported capability existed.

**What the evidence does and does not support.** It shows the failure mode is
real, recognized by regulators, and present in all three hypothesized
settings. It does **not** show a large addressable market for an independent
audit. The open empirical question is whether enough buyers will pay $5K
rather than solve it internally or accept the risk.

## The experiment that settles it

Everything above is preamble to one question, and it is small enough to run:

> **Can you find three strangers who will pay about $5,000 for this?**

If yes, decide then whether it becomes a repeatable consultancy, a
productized service, or a pleasant expert practice. If no, another hundred
pages of market analysis will not rescue it.

The two things that gate even attempting it: a sample audit that finds
something consequential (document 5), and the fifteen-minute clarity test —
hand the deliverable to someone who buys this kind of work and time whether
they can explain back what it found and why it matters.
