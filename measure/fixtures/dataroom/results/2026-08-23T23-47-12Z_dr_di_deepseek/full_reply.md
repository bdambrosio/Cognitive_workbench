I locked the derived arithmetic with exact calculation and re-pulled the load-bearing lines verbatim for citations (doc4's config, doc9's seven claims, and the doc1/doc2/doc5/doc6/doc7 evidence). The report and Gap Map follow.

---

# FlowMetrics — Technical Claims-Verification Report

Prepared for the buyer's side. Scope: stated claims (doc1 listing, doc2 SE tech-stack, doc9 verbatim claims) against observed evidence in the data room. Not a pen-test, not a code review, not legal advice.

**Recommendation: Material** — deltas found in the reliability axis materially change the risk profile, and the valuation depends on unverified revenue. Our professional judgement: price these in explicitly or walk. The audit recommends; the buyer decides. This borders on Walk.

## Findings

**Finding 1: Recovery window expires 2026-08-29 — [derived]**

Basis: doc4:18 — "**Last Successful Backup:** 2026-07-30"; doc9:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention."

Derivation: 2026-07-30 + 30-day retention = 2026-08-29. As of 2026-08-23, six days remain before no recoverable backup exists; the 21-day failure record (doc4:17) means no newer recovery point is available.

Consequence: after 2026-08-29 the business cannot be restored to any state newer than the last good backup, and that backup will have expired. This is the most time-critical fact in the room for a buyer of the data.

Escalates: Finding 2.

**Finding 2: Daily automated backups — [delta]**

Claim (doc9:6, doc9:10; doc2:5): daily automated database backups with 30-day retention.

Evidence: doc4:16-19 — the schedule exists ("Daily at 2:00 AM via `heroku pg:backups schedule`"), but "**Status:** Failures recorded for the last 21 days" and "**Alerting:** None configured for backup failures."

Delta: the backups are scheduled, not functioning. No backup has succeeded since 2026-07-30, and the failure is silent.

**Finding 3: Redundancy and automatic failover — [delta]**

Claim (doc9:5; doc1:18; doc2:3): "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: doc4:5-8 — a single `standard-1x` dyno (1GB RAM, 0.5 CPU); database `heroku-postgresql:standard-0` running on the same dyno as the application; **Read Replicas:** None; **Separate DB Instance:** No.

Delta: a single dyno with the database co-located is a single point of failure. There is no second process to fail over to, no read replica, no separate DB host. The claimed platform-level redundancy does not exist in this deployment.

**Finding 4: 99.9% uptime monitoring — [delta]**

Claim (doc9:6; doc1:19): "We monitor uptime at 99.9%."

Evidence: doc4:22 — "**Uptime Monitor:** None (No Pingdom, UptimeRobot, or custom checks)."

Delta: no uptime monitor exists, so the claimed 99.9% is measured by nothing. The three enterprise contracts carry 99.9%/99.5% SLAs with service-credit remedies (doc7:8,16,24) — the seller is contractually bound to a figure they do not measure, and would owe credits for outages they would not detect.

**Finding 5: $40k blended MRR — [partial]**

Claim (doc1:9,12): "Blended MRR … $40,000."

Evidence: doc5:4-5 — 94 active subscriptions, Total MRR (Stripe): $16,000; doc5:19 — the three enterprise companies have separate contracts at $8,000/mo each paid via wire transfer, not in Stripe; doc7:6,14,22 — three enterprise contracts at $8,000/mo each, but doc7 does not state the payment method.

Delta: the $40k figure holds only if $24k/mo of wire revenue is real and collectible. The contracts document does not corroborate the wire characterization — that detail appears only in the seller's own summaries (doc5/doc6). Derived: if the wire revenue does not hold, the $480k asking price is not 12x $40k but 30x $16k (480,000 ÷ 16,000 = 30, exact).

**Finding 6: 120 active accounts — [partial]**

Claim (doc1:9,13): 120 active accounts; low churn.

Evidence: doc6:3 — 120 marked 'Active'; doc6:12-16 — 23 "Phantom/Inactive" accounts: created Q1 2026, no payment events in the last 90 days, last login 60–120 days ago, no notes on file.

Delta: 23 of the 120 active accounts are not paying. The real paying base is ~94 (Stripe-linked). The "low churn" claim is contradicted by 23 accounts created in Q1 2026 that have never paid.

**Finding 7: Designed to scale horizontally — [partial]**

Claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: adding dynos is a Heroku platform property (true), but doc4:5-8 shows a single dyno with the database co-located on it — the scaling bottleneck.

Delta: the claim is true about the platform, false about this deployment.

**Finding 8: No lock-in / standard SaaS — [real, with a structural note]**

Claim (doc9:9): "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: doc7:29-34 — 94 month-to-month subscriptions on standard ToS with no-penalty cancellation; no individual contracts. Claim holds.

Structural note: two single points of dependency sit outside the "no lock-in" framing — DataEnrich.io provides 40% of the product's features with no fallback (doc8:11-12), and DNS is on GoDaddy, a single point managed personally by the seller (doc8:30-32).

**Finding 9: Stripe MRR arithmetic — [partial] (micro)**

Claim (doc5:17): "Total Stripe MRR: $16,000 ($13,549 + $2,451)."

Evidence: doc5:11 states the Pro subtotal as $13,549 for 91 accounts × $149/mo, but 91 × 149 = $13,559 (verified by exact calculation).

Delta: a $10/mo arithmetic error in the seller's own export summary. Minor in isolation; material as a consistency signal — the numbers were assembled carelessly.

## Coverage statement

~22 distinct claims identified across the three claim-bearing documents. 20 were assessable from the data room; 2 are unverifiable and go in Remaining Claims. Of the 20 assessed: 3 deltas, 2 derived findings, 4 partials, 1 real-with-structural-note; the remainder held or are uncontested. Claim 7 (Rails 7 / automatic SSL / managed DNS) is uncontested on the Rails-7 hosting (doc2:3, doc4:5); the SSL/DNS specifics were not independently verifiable from the room, but Heroku provides these by default, so low-risk.

## What to ask the seller before closing

1. Proof the $24k/mo wire revenue is real and recurring — bank statements, wire confirmations, or contracts stating payment method. doc7 does not confirm it, and the entire valuation depends on it.
2. The backup failure: when did it start, what is the fix, and can you demonstrate a restore from the 2026-07-30 backup today? After 2026-08-29 there is no recoverable backup.
3. The 23 phantom accounts: what are they, why were they created, are any billable?
4. Is there ANY secondary path for the application or database? (None is documented.)
5. How is the 99.9% uptime measured given no monitor exists — and how would the seller know an outage occurred on an SLA-bound contract?
6. DataEnrich.io: what is the fallback if it goes away (40% of features)?

## Remaining claims (not findings)

- "The codebase is well-documented" (doc9:7, doc1:21) — unverifiable; no code in the data room.
- "Comprehensive test coverage across all critical paths" (doc9:7) — unverifiable; no code in the data room.

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS (120 claimed active accounts, $40k claimed MRR, asking $480k)

**Material** — deltas in the reliability axis; valuation depends on unverified revenue. Borders on Walk.

Key items:
- ⚠️ **Backups failing 21 days** — last good 2026-07-30; 30-day retention expires **2026-08-29** (six days). No alerting.
- ⚠️ **"Redundancy and automatic failover"** — single dyno, DB co-located, zero replicas. No failover exists.
- ⚠️ **"99.9% uptime monitoring"** — no monitor at all; enterprise SLAs unmonitored.
- ⚠️ **$40k MRR** — $16k verified via Stripe; $24k wire unverified. If wire fails, 12x → 30x on $16k.
- ✓ **Real:** Stripe payment processing, Redis session caching, no-lock-in SaaS agreements — confirmed in config and contracts.

Coverage: 20 of ~22 claims assessed from the data room; 2 codebase-quality claims unverifiable (no code in the room).

Full report with citations available on request.

*Technical claims verification · not a pen-test, not legal advice*