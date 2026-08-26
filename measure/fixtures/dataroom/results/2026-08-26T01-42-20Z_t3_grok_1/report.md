**FlowMetrics technical claims audit**

**Recommendation: Material** (of 54 claims identified, 40 checked individually; remainder low-impact marketing copy). Asking price $480,000 rests on blended MRR and recoverability that the materials only partly support.

Materials as-of 2026-08-25.

---

**Finding 1: Last recoverable backup ages out 2026-08-29 — [derived]**

Basis: doc9:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention."
       doc4:18 — "Last Successful Backup: 2026-07-30"

Derivation: 30-day retention from 2026-07-30 last-good backup exhausts on 2026-08-29.

Consequence: After 2026-08-29 the materials show no remaining recoverable backup of production data.

Escalates: Finding 2

**Finding 2: Daily automated backups — [partial]**

Claim (doc1:19; doc9:6,10): daily automated database backups with 30-day retention.

Evidence: doc4:16–19 — schedule daily at 2:00 AM via `heroku pg:backups schedule`; "Failures recorded for the last 21 days"; last successful backup 2026-07-30; alerting none configured.

Gap: Schedule exists; backups have failed for 21 days with no alerting.

**Finding 3: 120 active accounts — [delta]**

Claim (doc1:9,13): 120 active customers/accounts.

Evidence: doc6:3–18 — 120 marked active; 94 Stripe-linked, 3 enterprise wire, 23 phantom/inactive (created Q1 2026, no payment in 90 days, last login 60–120 days). doc5:4 — 94 active Stripe subscriptions.

Gap: CRM status is not payment-active. Paying accounts are 97 (94+3), not 120.

**Finding 4: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1:5,9,12): blended MRR $40,000; asking $480,000 (12x blended MRR).

Evidence: doc5:5,17,19 — Stripe MRR $16,000; three enterprises $8,000/mo each by wire, not in Stripe. 16,000 + 24,000 = 40,000.

Gap: Arithmetic holds if wire contracts collect. $24,000/mo is off-Stripe; GlobalMart renews 2026-11-01 (doc7:15).

**Finding 5: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "We monitor uptime at 99.9%".

Evidence: doc4:21–23 — Uptime Monitor: None; status page is Heroku built-in only. Lexical search of evidence for Pingdom/UptimeRobot/custom checks: none. Structural: monitoring section of doc4 is the place it would be.

Gap: No uptime monitor. Enterprise SLAs of 99.9%/99.5% (doc7:8,16,24) have no measurement path in the materials.

**Finding 6: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3,5; doc9:5): redundancy and automatic failover through Heroku platform-level process management.

Evidence: doc4:4–8 — one `standard-1x` dyno; postgres `heroku-postgresql:standard-0` on the same dyno as the application; read replicas none; separate DB instance no.

Gap: Single dyno, database co-located, no replicas. Platform process restart is not the claimed redundancy/failover architecture.

**Finding 7: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): well-documented codebase with comprehensive test coverage across critical/all critical paths.

Evidence: doc3:25–32 — CI/CD none; unit tests 12, all in `test/utils/`; integration 0; payment-path 0; staging none; branch protection none; code review none documented. Documentation of the codebase is not present in the data room (lexical: document; structural: git summary has no docs tree).

Gap: Tests are 12 utility units; payment path untested; no CI.

**Finding 8: No proprietary lock-in / standard SaaS agreements — [delta]**

Claim (doc1:22; doc9:9): all third-party integrations on standard SaaS agreements with no lock-in / no proprietary lock-in.

Evidence: doc8:7–12 — DataEnrich.io 90-day termination either party; 40% of features depend on it; fallback none. doc8:25–28 — Google OAuth, no contractual relationship; if Google revokes, users cannot log in. doc8:29–32 — GoDaddy DNS managed personally by dave; no secondary DNS.

Gap: Material product share has a 90-day-notice sole vendor and no fallback; auth and DNS are single-person/single-vendor with no contract or secondary.

**Finding 9: Managed DNS — [delta]**

Claim (doc9:11): "modern Rails 7 stack with automatic SSL and managed DNS."

Evidence: doc4:26–28 — SSL Heroku-managed auto-renewed [holds]; DNS Provider GoDaddy; managed personally by dave; no secondary. doc8:29–32 same.

Gap: DNS is not Heroku-managed; it is Dave’s personal GoDaddy account.

**Finding 10: Horizontal scale as one-click dynos — [real, operational caveat]**

Claim (doc9:8): stack designed to scale horizontally — adding dynos is a one-click Heroku operation.

Evidence: doc4:4–8 — currently one standard-1x; postgres on the same dyno, no separate DB instance, no replicas. Heroku can add dynos; the database topology does not scale with that click.

Gap: Dyno add is real; data layer is not horizontally scaled in the materials.

**Finding 11: Stripe payment processing — [real]**

Claim (doc1:20; doc2:5): secure payment processing via Stripe; credit cards and recurring billing.

Evidence: doc5 entire; doc8:3–6. Stripe path exists and is the recorded $16,000 MRR.

Gap: None for the Stripe claim. Wire $24,000 is outside it (Finding 4).

**Finding 12: Rails 7 on Heroku, Redis, SSL — [real]**

Claim (doc2:3–5; doc9:11 in part): Rails 7, Heroku, PostgreSQL, Redis, automatic SSL.

Evidence: doc4:4–12,26. Holds.

Gap: None on stack names.

**Finding 13: Low churn — [unverifiable]**

Claim (doc1:9,14): low churn, stable base of recurring subscribers.

Evidence: no churn rate, cohort, or cancellation series in docs 3–8. 23 CRM phantoms and 8 unpaid pilots (doc7:36–40) are not a churn series.

Not a finding; Remaining Claims.

**Finding 14: Asking 12× blended MRR = $480,000 — [real]**

Claim (doc1:5): $480,000 (12x blended MRR).

Evidence: 12 × 40,000 = 480,000. Holds as arithmetic on the claimed MRR, not as a valuation opinion.

---

**Coverage.** 54 claims frozen (doc1: 33, doc2: 13, doc9: 8). 40 checked individually, including all priority-1 (recoverability, payment integrity, single points of control) and priority-2 architectural invariants. Unchecked remainder is marketing ("turnkey", "proven product-market fit", "defensible product", "strategic partner") — low-impact under §4. Consistency on checked material claims is poor: multiple [delta] on reliability, tests, lock-in, customer count.

**Ask the seller before closing.** (1) Can they produce a successful restore from the 2026-07-30 backup, and what exists after 2026-08-29? (2) Wire-transfer collection evidence for the three $8,000 contracts. (3) Disposition of 23 CRM-active non-payers and 8 Q3 2026 pilots. (4) DataEnrich.io assignment / fallback plan. (5) Transfer of GoDaddy DNS and Google OAuth app ownership. (6) How 99.9% SLA is measured given no uptime monitor.

===
LIMITATIONS ===
1. Materials: nine data-room documents (doc1–doc9) as of 2026-08-25; no live systems, no source tree beyond the git summary.
2. The seller was not consulted and has not confirmed this reading of their own claims.
3. Limited assurance (§1a): 40 of 54 identified claims checked individually; recommendation Material rests on that coverage.
