=== CLAIM SURFACE ===
32 claims

**Recommendation: Material** — of 32 claims identified, 18 were checked individually. Multiple `[delta]` findings on recoverability, architecture, lock-in, monitoring, DNS, and test coverage, plus `[partial]` on the 120-account count, change valuation and risk. Coverage: 18/32; remaining 14 are low-impact restatements or qualitative marketing (turnkey, loyal, proven PMF) that do not reverse the material gaps.

**Finding 1: Last recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc1:19 / doc9:10 — "daily automated database backups with 30-day retention" / "backed up daily to Heroku's managed storage with 30-day retention"
       doc4:18 — "Last Successful Backup: 2026-07-30"

Derivation: 30-day retention from 2026-07-30 exhausts on 2026-08-29.

Consequence: After 2026-08-29 the materials show no remaining restorable backup. As-of the materials snapshot (session 2026-08-25) that date is four days out.

Escalates: Finding 2

**Finding 2: Daily automated backups — [delta]**

Claim (doc1:19, doc9:6, doc9:10, doc2:5): daily automated database backups / "We monitor uptime at 99.9% and maintain daily automated database backups."

Evidence: doc4:16–19 — Schedule daily at 2:00 AM; "Failures recorded for the last 21 days."; Last Successful Backup 2026-07-30; "Alerting: None configured for backup failures."

Gap: Schedule exists; backups have failed for 21 days with no alerting. Last success coincides with last git commit (doc3:21).

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc1:19, doc9:6): "99.9% uptime monitoring" / "We monitor uptime at 99.9%"

Evidence: doc4:22–23 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."; Status page is Heroku built-in only. doc8:16 — Heroku SLA is 99.95% platform uptime, "not contractual for the app." Enterprise contracts (doc7:8,16,24) require 99.9% or 99.5% app SLA.

Gap: No uptime monitor. Platform SLA is not the claimed app monitoring.

**Finding 4: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18, doc2:3, doc9:5): Rails 7 with platform-level redundancy and automatic failover / Heroku automatic scaling and failover.

Evidence: doc4:5–8 — single `standard-1x` dyno; `heroku-postgresql:standard-0` "running on the same dyno as the application"; Read Replicas: None; Separate DB Instance: No.

Gap: Single dyno, DB co-located, no replicas. Stated redundancy/failover does not hold in the config.

**Finding 5: Designed to scale horizontally / automatic scaling — [delta]**

Claim (doc9:8, doc2:3): "The stack is designed to scale horizontally — adding dynos is a one-click operation"; Heroku "automatic scaling".

Evidence: doc4:5 — one `standard-1x` dyno. No second dyno, no autoscaling config in the materials.

Gap: Horizontal add-a-dyno is a Heroku capability, not an observed configuration. Automatic scaling is not shown.

**Finding 6: 120 active accounts — [partial]**

Claim (doc1:9, doc1:13): "120 active accounts" / "Active Customers: 120"

Evidence: doc6:3,6–18 — 120 marked active = 94 Stripe-linked + 3 enterprise wire + 23 phantom/inactive (created Q1 2026, no payment in last 90 days, last login 60–120 days ago). doc5:4 — 94 Stripe subscriptions. doc7:29,36,42 — 94 m2m, 8 unpaid pilots, 15 Pro users with no ToS on file.

Gap: CRM status is 120; paying/operating accounts in the materials are 97 (94+3). 23 have no payment events.

**Finding 7: No proprietary lock-in / standard SaaS agreements — [delta]**

Claim (doc1:22, doc9:9): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in" / "no lock-in."

Evidence: doc8:7–12 — DataEnrich.io $400/mo; "Termination Notice: 90 days (either party)"; "40% of features depend on this API"; "Fallback: None implemented." Enterprise required features (doc7:11,19,27) include product matching, price comparison, category classification — the DataEnrich functions (doc8:8).

Gap: Either-party 90-day terminate with 40% of features and no fallback is lock-in the claim denies.

**Finding 8: Managed DNS — [delta]**

Claim (doc9:11): "We use a modern Rails 7 stack with automatic SSL and managed DNS."

Evidence: doc4:26–28 — SSL Heroku-managed auto-renewed; DNS Provider GoDaddy; "Managed personally by 'dave'. No secondary DNS provider." doc8:29–32 — same; if domain expires or GoDaddy has an issue, the app is unreachable.

Gap: SSL holds. DNS is personal GoDaddy, not managed DNS as claimed.

**Finding 9: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21, doc9:7): well-documented codebase with comprehensive test coverage across critical / all critical paths.

Evidence: doc3:24–32 — CI/CD none; Unit Tests 12, all in `test/utils/`; Integration Tests 0; Payment-Path Tests 0; Staging none; Branch protection none; Code review none.

Gap: 12 utility unit tests, zero payment-path or integration tests. "Comprehensive… critical paths" is false on the materials.

**Finding 10: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1:5,9,12): Blended MRR $40,000; asking $480,000 (12x blended MRR).

Evidence: doc5:5,17,19 — Stripe MRR $16,000; three enterprises $8,000/mo each via wire, not in Stripe. 16000 + 3×8000 = 40000 if wires are collected. doc7:6,14,22 confirm $8,000/mo each. Asking 12×40000 = 480000 (doc1:5).

Gap: None on the arithmetic if wires collect. Caveat: $24,000/mo is off-Stripe wire; Stripe alone is $16,000. GlobalMart renews 2026-11-01 (doc7:15).

**Finding 11: Rails 7, Heroku, Redis, Stripe, SSL — [real]**

Claim (doc1:18,20; doc2:3,5; doc9:11): Rails 7 on Heroku, Redis, Stripe payments, managed SSL.

Evidence: doc4:4,11–12,26; doc8:3–5,13–20; doc2:3,5. Holds.

Gap: None on these stack items (DNS and scaling are Findings 5 and 8).

**Finding 12: Recurring subscription model (monthly and annual enterprise) — [real, minor caveat]**

Claim (doc1:15): Recurring subscription (Monthly & Annual Enterprise).

Evidence: doc5, doc7: 94 m2m Stripe + 3 annual enterprise. Holds. Caveat: 8 unpaid pilots expire Q3 2026 (doc7:36–40); 15 Pro accounts have no ToS on file (doc7:42–45).

**Coverage.** 18 of 32 claims checked. Unchecked remainder: qualitative listing language (loyal, turnkey, proven PMF, immediate cash flow, pain point, strategic partner, low churn as a measured rate, well-documented as comments, Redis session caching as purpose, "real-time" analytics, "minimal operational overhead"). Those do not reverse recoverability, lock-in, or architecture. Churn as a rate is `[unverifiable]` — no cohort table. Wire collection itself is `[unverifiable]` — Stripe export notes the contracts, not bank receipts.

**Ask the seller before closing.** (1) Restore drill from the 2026-07-30 backup and backup-failure logs. (2) Bank evidence of the three $8,000 wires. (3) DataEnrich contract and whether 90-day terminate is mutual in the signed paper. (4) DNS registrar login transfer. (5) Whether the 23 CRM "active" accounts will be excluded from the 120. (6) GlobalMart renewal 2026-11-01.

=== LIMITATIONS ===
Materials: nine data-room documents (doc1–doc9) as bound to inspect_external; as-of 2026-08-25; last commit and last successful backup dated 2026-07-30 in those files.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 18 of 32 enumerated claims checked; conclusion is only about those 18.

=== GAP MAP ===
**FlowMetrics** — B2B e-commerce analytics SaaS, asking $480k.

**Material** (18 of 32 claims checked).

- Backups: daily schedule, failed 21 days, last success 2026-07-30; 30-day retention exhausts **2026-08-29**.
- Architecture: one standard-1x dyno, DB on the same dyno, no replicas, no uptime monitor — not the claimed redundancy, failover, or 99.9% monitoring.
- Customers: 120 CRM-active vs 97 paying (94 Stripe + 3 wire); 23 phantoms.
- Lock-in: DataEnrich.io, 40% of features, 90-day either-party terminate, no fallback; enterprise SLAs need those features.
- Tests/CI: 12 unit tests in test/utils, 0 payment-path, no CI. DNS is Dave’s GoDaddy, not managed DNS.
- MRR $40k arithmetic holds if the three $8k wires collect; Stripe alone is $16k.

Coverage: 18/32 checked; remainder low-priority marketing language. Professional judgement: the recoverability date and the DataEnrich concentration are the items that move price or structure; they are not sprint-fixable after 2026-08-29 if the backup ages out.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice