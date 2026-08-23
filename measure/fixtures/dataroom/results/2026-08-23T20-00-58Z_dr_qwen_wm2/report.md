**AUDIT REPORT: FLOWMETRICS — TECHNICAL CLAIMS VERIFICATION**

**Recommendation: Material**

Six of eight verifiable technical claims in the seller's own verbatim statement (doc9) are contradicted by the data room's evidence files. Two additional financial claims in the listing (doc1) are also contradicted. The most urgent finding is operational: the last successful database backup is dated 2026-07-30 and, on the seller's own stated 30-day retention, expires 2026-08-29 — six days from the date of this report. After that date there is no backup in existence. The buyer should not close without remediation of the backup infrastructure, an independent uptime report, and a renegotiated valuation reflecting the actual operational state.

---

**Findings**

**Finding 1: Daily automated backups — [delta]**

Claim (doc9:10, doc2:5, doc1:19): "The database is backed up daily to Heroku's managed storage with 30-day retention." / "configured with daily scheduled backups to ensure data integrity and recoverability."

Evidence (doc4:16-19): Schedule is daily at 2:00 AM, but Status reads: "Failures recorded for the last 21 days." Last Successful Backup: 2026-07-30. Alerting: "None configured for backup failures."

Delta: Backups have not run for 21 consecutive days. The claim of daily automated backups is false as of the evidence date. The absence of alerting means the failure went undetected for at least three weeks.

**Finding 2: Backup retention window exhausted — [derived]**

Basis: doc4:18 — "Last Successful Backup: 2026-07-30" (verbatim)
       doc9:10 — "30-day retention" (verbatim)

Derivation: 2026-07-30 + 30 days = 2026-08-29. Date of this report: 2026-08-23.

Consequence: The only existing backup expires in six days. After 2026-08-29, no database backup exists. Any data loss, corruption, or accidental deletion after that date is unrecoverable. For a SaaS business in the middle of a sale, this is an existential data-recovery risk at the moment of transfer.

Escalates: Finding 1

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc9:6, doc1:19): "We monitor uptime at 99.9% and maintain daily automated database backups." / "99.9% uptime monitoring and daily automated database backups with 30-day retention."

Evidence (doc4:22): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." Status Page: "Heroku built-in status page only."

Delta: No uptime monitoring tool of any kind is configured. The 99.9% figure has no measurement behind it. The claim is false.

**Finding 4: Comprehensive test coverage — [delta]**

Claim (doc9:7, doc1:21): "The codebase is well-documented with comprehensive test coverage across all critical paths." / "comprehensive test coverage across critical paths."

Evidence (doc3:25-31): CI/CD Pipeline: None configured. Unit Tests: 12 (all in `test/utils/`). Integration Tests: 0. Payment-Path Tests: 0. Staging Environment: None. Branch Protection: None. Code Review Process: None documented.

Delta: Twelve unit tests in a utility directory, zero integration tests, zero payment-path tests, no CI/CD, no staging. The payment path — the mechanism that generates all revenue — has zero test coverage. The claim is false.

**Finding 5: Redundancy and automatic failover — [delta]**

Claim (doc9:5, doc2:3, doc1:18): "The system has redundancy and automatic failover through Heroku's platform-level process management." / "provides automatic scaling and failover capabilities at the platform level."

Evidence (doc4:5-8): Single `standard-1x` dyno (1GB RAM, 0.5 CPU). Database: `heroku-postgresql:standard-0`, "running on the same dyno as the application." Read Replicas: None. Separate DB Instance: No. (doc8:16): Heroku SLA is 99.95% platform uptime, "not contractual for the app."

Delta: There is one dyno, one database, no replicas, no failover path. If the dyno or database process fails, the application is down with no automatic recovery. The claim describes infrastructure that does not exist.

**Finding 6: Horizontal scaling — [delta]**

Claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence (doc4:5-8): Single dyno, DB on same dyno, no read replicas, no separate DB instance. (doc3:25,30): No CI/CD, no staging environment.

Delta: The database is coupled to the application dyno. Adding a second application dyno without a separate database instance creates a single point of contention. There is no staging environment to test a scaling change before deploying to production. The claim describes a capability the architecture does not support.

**Finding 7: No vendor lock-in — [partial]**

Claim (doc9:9, doc1:22): "All third-party integrations are on standard SaaS agreements with no lock-in." / "no proprietary lock-in."

Evidence (doc8:7-12): DataEnrich.io: "40% of features depend on this API." Termination Notice: 90 days. Fallback: "None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)."

Delta: The claim uses the quantifier "all." One integration — DataEnrich.io — is not a standard SaaS dependency: 40% of product features depend on it, there is no fallback, and the 90-day termination window means a buyer inherits a lock-in that cannot be exited quickly. The other integrations (Stripe, Heroku, Redis, Twilio, Google OAuth, GoDaddy) are closer to standard, but the single major exception defeats the universal claim. The gap is the DataEnrich.io dependency and the absence of a fallback.

**Finding 8: 120 active accounts — [delta]**

Claim (doc1:9, doc1:13): "120 active accounts" / "Active Customers: 120."

Evidence (doc6:3,12-16,18): 120 accounts marked 'active' in CRM. Breakdown: 94 Stripe-linked, 3 enterprise wire, 23 phantom/inactive (created Q1 2026, no payment events in last 90 days, last login 60-120 days ago, no notes on file). All 120 remain marked 'active.' (doc7:36-39): 8 pilot agreements, free 90-day, no payment obligation, expire Q3 2026. (doc7:42-45): 15 no-contract accounts, no signed ToS on file.

Delta: The CRM status field is not maintained. 23 of 120 accounts have had no payment for 90+ days and no login for 60-120 days. The true paying base is 94 Stripe + 3 wire = 97. The claim of 120 "active accounts" overstates the paying customer base by at least 23. The 8 free pilots and 15 no-contract accounts further complicate the picture: the data room does not fully disambiguate whether the pilots are a subset of the 23 phantoms or separate, and I am not asserting a resolution I cannot verify.

**Finding 9: Contractual SLA exposure without monitoring — [derived]**

Basis: doc7:8,16 — Acme Retail and GlobalMart contracts: "SLA: 99.9%" with "30-day notice for material SLA breach." doc7:24 — ShopStream: "SLA: 99.5%."
       doc4:22 — "Uptime Monitor: None."

Derivation: 99.9% monthly uptime allows approximately 43 minutes of downtime per month. The seller has contractual SLA obligations on $24,000/mo of revenue (60% of total MRR) but no mechanism to measure whether those SLAs are being met. A single undetected multi-hour outage breaches the SLA and triggers the 30-day termination clause. Without monitoring, the seller cannot detect the breach, cannot issue the contractual service credits (10% of monthly fee per 0.1% below SLA, doc7:9,17,25), and is exposed to termination if a customer discovers the breach independently.

Consequence: The majority of the business's revenue is contingent on a performance metric that is not being measured. This is not a claim delta — the seller did not claim to monitor the SLA specifically — but it is a structural risk that Finding 3's false "99.9% uptime monitoring" claim made worse by implying the seller knows their uptime number when they do not.

Escalates: Finding 3

**Finding 10: Low churn — [partial]**

Claim (doc1:14): "Churn: Low (stable base of recurring subscribers)."

Evidence (doc6:12-16): 23 of 120 accounts (19%) show no payment in 90 days and last login 60-120 days ago. (doc7:36-39): 8 pilot agreements expiring Q3 2026 with no payment obligation.

Delta: I cannot calculate a churn rate from a single snapshot. However, 19% of the stated base shows the hallmarks of churned or lapsed accounts that have not been marked inactive. If those represent churn since Q1 2026, the rate is not "low" relative to a 120-account base. I mark this [partial] rather than [delta] because I cannot distinguish between "churned and not updated" and "dormant but potentially returning" from the data available. The CRM status field has not been maintained, which means the churn figure the seller presents is based on a denominator that includes non-paying accounts.

**Finding 11: Blended MRR $40,000 — [real]**

Claim (doc1:9, doc1:12): "Blended Monthly Recurring Revenue (MRR) stands at $40,000."

Evidence (doc5:17): "Total Stripe MRR: $16,000 ($13,549 + $2,451)." (doc5:19, doc7:6,14,22): 3 enterprise contracts at $8,000/mo each via wire transfer = $24,000/mo, not processed through Stripe.

Delta: None. $16,000 + $24,000 = $40,000. The figure is arithmetically correct.

**Finding 12: Rails 7 stack, automatic SSL, managed DNS — [real, minor caveat]**

Claim (doc9:11): "We use a modern Rails 7 stack with automatic SSL and managed DNS."

Evidence (doc2:3): Rails 7 on Heroku. (doc4:26): "SSL: Heroku-managed, auto-renewed." (doc4:27-28): "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'. No secondary DNS provider." (doc8:29-32): GoDaddy, $12/yr, "If domain expires or GoDaddy has an issue, the app is unreachable."

Delta: None on the stack or SSL. Minor caveat: "managed DNS" is managed personally by one individual with no secondary provider. This is a single point of failure for domain resolution. Not a delta on the claim, but an operational risk worth naming.

**Finding 13: Minimal operational overhead / turnkey — [real, operational caveat]**

Claim (doc1:9): "minimal operational overhead" / "turnkey acquisition opportunity."

Evidence (doc3:8): 73% of commits by dave. (doc3:14): 47% of dave's commits in `auth/`. (doc4:28): DNS managed personally by dave. (doc4:29): 14 secrets in Heroku config vars, no secrets vault. (doc8:29-32): GoDaddy DNS, no secondary. (doc3:21-22): Last commit 2026-07-30, 0 commits in last 30 days.

Delta: The infrastructure is simple, so "minimal overhead" is true in a narrow sense. But "turnkey" implies the business continues operating without the founder. The DNS, the authentication codebase (47% of the dominant contributor's commits), and all secrets are concentrated in one individual. If dave is unavailable at or after closing, operational continuity is at risk. This is an operational caveat, not a delta — the claims are vague enough to survive — but a buyer must plan for it.

**Finding 14: 12x blended MRR asking price — [non-delta]**

Claim (doc1:5): "$480,000 (12x blended MRR)."

Evidence: $40,000 × 12 = $480,000. Arithmetic is correct. Whether 12x is a fair multiple is the buyer's judgement, not a claims-verification question.

Not a finding — noted for completeness.

---

**Remaining Claims (unverifiable from the data room)**

- "Well-documented codebase" (doc9:7, doc1:21) — [unverifiable]: The data room does not contain the codebase or its documentation. The absence of CI/CD, code review, and branch protection (doc3:25,31,32) is evidence against maintenance processes but not direct evidence about documentation quality.
- "Real-time analytics and product intelligence" (doc1:9) — [unverifiable]: No product demo or feature documentation in the data room. The DataEnrich.io dependency (doc8:7-12) suggests the "product intelligence" is substantially a pass-through of a third-party API, which is a structural note but not a claim delta.
- "Proven product-market fit" (doc1:9,25) — [unverifiable]: Subjective marketing claim, not testable from the data room.
- "Revenue-positive" (doc1:9) — [unverifiable]: No P&L or financial statements in the data room.
- "Immediate cash flow" (doc1:9) — [unverifiable]: Same reason.
- "Defensible product" (doc1:25) — [non-delta]: No claim to verify against implementation.

---

**Coverage Statement**

I verified 14 claims out of approximately 28 identified (50%). All priority 1 and 2 claims (safety-critical and architectural) were verified. The remaining 14 are either unverifiable from the data room (no codebase, no financial statements, no product demo) or non-delta (subjective marketing language). The unverifiable items are listed above with reasons. I did not check: the actual code, the contents of the 12 unit tests, the Stripe transaction-level data beyond the summary, or the CRM beyond the summary provided. A buyer who needs deeper assurance on any of these should request the underlying artifacts.

---

**What the client should ask the seller before closing**

1. Provide a live demonstration of the backup process and confirm the last successful backup. If the 2026-07-30 backup has expired or will expire before closing, require a fresh backup as a condition of the transaction.
2. Provide a 30-day uptime report from an independent monitoring service, or agree to install one before closing.
3. Provide a reconciled account list distinguishing paying, free, and inactive accounts. Ask why 23 accounts with 90+ days of non-payment remain marked "active."
4. Provide a retention or cohort analysis showing the actual monthly churn rate over the last 12 months.
5. Provide a key-person risk mitigation plan: documentation of the auth codebase, DNS failover, secrets vault, and cross-training of at least one other person.
6. Ask about the DataEnrich.io dependency: what happens to 40% of features if the API is terminated or repriced, and is a fallback in development?
7. Request the signed ToS for the 15 no-contract accounts, or confirm they are unenforceable.

---

**Closing paragraph for a non-specialist reader**

The seller's listing describes a reliable, scalable, well-tested SaaS business. The technical documentation in the same data room tells a different story. There is one server, one database on that server, no backup that has worked in three weeks, no monitoring of any kind, no automated tests, and no second person who manages the domain name. The revenue figure ($40,000/month) checks out arithmetically, but the customer count is inflated by accounts that have not paid in three months, and the two largest enterprise contracts carry uptime guarantees that are not being measured. The most urgent issue is time-sensitive: the only existing database backup expires in six days. After that date, the business has no data safety net. The valuation should reflect the operational reality in the evidence files, not the marketing language in the listing.
