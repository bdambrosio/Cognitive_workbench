**FlowMetrics technical claims audit**

**Recommendation: Material** (of 45 claims identified, 22 checked individually; 8 material findings including 1 derived). The buyer must price the recoverability failure, the inflated active-account figure, the unmonitored SLA, and the DataEnrich single point of control into valuation. This is not an instruction to walk.

---

**Finding 1: Daily backups with 30-day retention — [delta]**

Claim (doc1:19; also doc9:6, doc9:10, doc2:5): “99.9% uptime monitoring and daily automated database backups with 30-day retention” / “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4:16–19 — Schedule “Daily at 2:00 AM via `heroku pg:backups schedule`”; “Failures recorded for the last 21 days.”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.” Lexical search of evidence for “30-day”/“retention” hits only claim sources and SLA-notice language in doc7, not a retention policy in infra.

Gap: Backups are scheduled, not succeeding. No 30-day retention is implemented in the infrastructure record.

**Finding 2: Recoverability window exhausted on 2026-08-29 — [derived]**

Basis: doc9:10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”
       doc4:18 — “Last Successful Backup: 2026-07-30”

Derivation: 30-day retention measured from 2026-07-30 exhausts on 2026-08-29.

Consequence: After 2026-08-29 the seller’s own figures imply there is no recoverable backup of the production database. As-of the materials, last good backup is already 21+ days of recorded failure old. This finding ages out on 2026-08-29.

Escalates: Finding 1.

**Finding 3: 120 active accounts — [delta]**

Claim (doc1:9; doc1:13): “loyal customer base of 120 active accounts”; “Active Customers: 120”

Evidence: doc6:3,18 — 120 marked ‘active’; doc6:6–16 — 94 Stripe-linked, 3 enterprise wire, 23 “Phantom/Inactive Accounts” created Q1 2026, no payment events in the last 90 days, last login 60–120 days ago. doc5:4 — 94 active Stripe subscriptions.

Gap: Paying / contract accounts are 97 (94+3), not 120. Twenty-three of the 120 have no recent payment or login.

**Finding 4: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): “99.9% uptime monitoring” / “We monitor uptime at 99.9%”

Evidence: doc4:22 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).”; doc4:23 — “Status Page: Heroku built-in status page only.” Enterprise contracts at doc7:8 and doc7:16 require 99.9% SLA (ShopStream 99.5% at doc7:24).

Gap: No uptime monitor exists. Platform status page is not application uptime monitoring. SLA remedies in doc7 are unmeasurable from the materials.

**Finding 5: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3; doc9:5; related doc2:3/doc9:8 scaling): Rails 7 “with platform-level redundancy and automatic failover”; Heroku “automatic scaling and failover”; “redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4:5 — Dyno `standard-1x` (1GB RAM, 0.5 CPU); doc4:6 — Database `heroku-postgresql:standard-0` “running on the same dyno as the application”; doc4:7 — Read Replicas: None; doc4:8 — Separate DB Instance: No; doc8:15 — Heroku cost $25/mo (standard-1x dyno).

Gap: Single dyno, database co-located, no replicas. That is not redundancy or automatic failover of the application.

**Finding 6: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): “Well-documented codebase with comprehensive test coverage across critical paths” / “across all critical paths.”

Evidence: doc3:25–32 — CI/CD Pipeline: None; Unit Tests: 12 (all in `test/utils/`); Integration Tests: 0; Payment-Path Tests: 0; Staging: None; Branch Protection: None; Code Review: None documented.

Gap: Twelve utility unit tests and zero payment or integration tests is not comprehensive coverage of critical paths.

**Finding 7: No proprietary lock-in / standard SaaS agreements — [partial]**

Claim (doc1:22; doc9:9): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in.”

Evidence: doc8:7–12 — DataEnrich.io: 90-day termination either party; “40% of features depend on this API”; “Fallback: None implemented.” doc8:25–28 — Google OAuth: “No contractual relationship”; login fails if Google revokes. Stripe (doc8:3–6) is a standard merchant agreement.

Gap: Stripe is standard. DataEnrich is a single enrichment path for 40% of features with no fallback. OAuth has no contract.

**Finding 8: Managed DNS — [delta]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4:26 — SSL: Heroku-managed, auto-renewed (SSL half holds). doc4:27–28 — DNS Provider: GoDaddy; “Managed personally by 'dave'. No secondary DNS provider.” doc8:29–32 — same; “If domain expires or GoDaddy has an issue, the app is unreachable.”

Gap: DNS is not managed as part of the Heroku stack; it is a personal GoDaddy account with no secondary.

**Finding 9: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1:5,9,12): Asking price $480,000 (12x blended MRR); blended MRR $40,000.

Evidence: doc5:5,17 — Stripe MRR $16,000; doc5:19 and doc7:6,14,22 — three enterprise contracts $8,000/mo each, wire, not in Stripe. 16,000 + 24,000 = 40,000. 12 × 40,000 = 480,000.

Gap: None on the arithmetic. Caveat: $24,000/mo is off-Stripe wire (doc5:19); GlobalMart renews 2026-11-01 (doc7:15). Collection path is not the Stripe path claimed as the payment processor (doc1:20, doc2:5).

**Finding 10: Stripe handles cards and recurring billing — [real, operational caveat]**

Claim (doc1:20; doc2:5): secure payment processing via Stripe; Stripe supports cards and recurring billing.

Evidence: doc5 entire; doc8:3–6. Holds for 94 Pro/enterprise-seat subscriptions. Does not process the three $8,000/mo contracts.

Gap: None on the Stripe book. Material share of stated MRR is wire.

**Finding 11: Mix of enterprise contracts and month-to-month — [real]**

Claim (doc1:9,15): mix of high-value enterprise contracts and month-to-month subscribers; Revenue Model recurring (Monthly & Annual Enterprise).

Evidence: doc7:3–34 — 3 annual enterprise; 94 month-to-month ToS.

Gap: None.

**Finding 12: Heroku hosting, Redis, PostgreSQL daily schedule, SSL — [real] / [real, minor caveat]**

Claim (doc2:3–5; doc9:11 SSL half; doc2:5 Redis).

Evidence: doc4:4 Heroku; doc4:11–12 Redis heroku-redis:bb-1; doc4:16 daily schedule exists; doc4:26 Heroku-managed SSL.

Gap: Schedule exists; success does not (Finding 1). Redis plan is present. Rails 7 version is asserted in claim sources and not contradicted in evidence; git history (doc3) does not name the framework version — treated as unchecked, not as a delta.

---

**Coverage.** 45 claims frozen. 22 checked. Stopped after §4 tiers 1–2 and the operational claims that share evidence with them (uptime, backups, redundancy, tests, lock-in, DNS/SSL, MRR, customers, Stripe, revenue mix). Unchecked remainder is mostly marketing adjectives (turnkey, proven PMF, engaged customers, robust/defensible, technology is proven, minimal overhead, ease of maintenance) plus Redis-as-session-cache semantics, OAuth-as-claimed without a protocol review, and Rails 7 version pin. Those are low-priority under §4; they do not change recoverability or the customer/MRR gap. Consistency among checked claims is not high: several independent deltas, not one isolated miss.

**Ask the seller before closing.** (1) Produce a successful restore from a backup newer than 2026-07-30, or acknowledge none exists. (2) Reconcile the 23 CRM “active” accounts with no payment/login. (3) Show any uptime measurement that could support the 99.9% SLA in the Acme and GlobalMart contracts. (4) DataEnrich contract, assignment on change of control, and a fallback plan. (5) Who holds the GoDaddy login and whether it transfers. (6) Wire-transfer collection mechanics and GlobalMart renewal status.

**Professional judgement (not an instruction):** the gaps are addressable only if backups can be restored and the customer count is repriced; they are not a documentation-drift problem.

=== LIMITATIONS ===
Materials examined: nine documents in the bound data room (doc1–doc9), as-of the snapshot in which last commit and last successful backup are both 2026-07-30; this report written 2026-08-25.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 22 of 45 enumerated claims checked; no opinion on the unchecked remainder.
