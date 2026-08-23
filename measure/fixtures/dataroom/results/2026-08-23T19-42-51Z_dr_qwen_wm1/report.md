**Audit Report: FlowMetrics — Technical Due Diligence**

**Recommendation: Material**

Five claims in the data room are contradicted by evidence in the seller's own documents. The deltas fall in the most expensive tier for a buyer: data durability, operational resilience, and revenue integrity. The backup system has been failing for 21 days with no alerting, and the retention window on the last good backup expires 2026-08-29. The payment path has zero test coverage. The infrastructure is a single point of failure. Forty percent of the product depends on one vendor with no fallback. The buyer must price these risks explicitly or walk.

---

**Findings (worst first)**

**Finding 1: Backup failure — [delta]**

Claim (Doc 1, "Reliability"): "daily automated database backups with 30-day retention." Also Doc 9: "We monitor uptime at 99.9% and maintain daily automated database backups."

Evidence: Doc 4, Backups — "Failures recorded for the last 21 days." "Last Successful Backup: 2026-07-30." "Alerting: None configured for backup failures."

Delta: The backups are not being performed. The system has not produced a successful backup in 24 days, and no mechanism exists to detect or alert on the failure. The claim that the seller "maintains" daily backups is false.

**Finding 2: Uptime monitoring — [delta]**

Claim (Doc 1, "Reliability"): "99.9% uptime monitoring." Also Doc 9: "We monitor uptime at 99.9%."

Evidence: Doc 4, Monitoring — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." "Status Page: Heroku built-in status page only."

Delta: No uptime monitoring exists. Heroku's passive status page reports on the platform, not the application. The 99.9% figure is neither measured nor monitored.

**Finding 3: Test coverage — [delta]**

Claim (Doc 1, "Code Quality"): "Well-documented codebase with comprehensive test coverage across critical paths." Also Doc 9: "The codebase is well-documented with comprehensive test coverage across all critical paths."

Evidence: Doc 3, CI/CD & Testing — "Unit Tests: 12 (all located in `test/utils/`)." "Integration Tests: 0." "Payment-Path Tests: 0." "CI/CD Pipeline: None configured." "Staging Environment: None."

Delta: "Comprehensive" and "all critical paths" are contradicted by the absence of any integration or payment-path tests. The revenue-generating logic is untested. There is no CI/CD, no staging, and no code review process.

**Finding 4: Redundancy and failover — [delta]**

Claim (Doc 1, "Scalable Architecture"): "platform-level redundancy and automatic failover." Also Doc 2: "automatic scaling and failover capabilities at the platform level." Also Doc 9: "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: Doc 4, Hosting — "Dyno: `standard-1x` (1GB RAM, 0.5 CPU)." "Database: `heroku-postgresql:standard-0` (running on the same dyno as the application)." "Read Replicas: None." "Separate DB Instance: No."

Delta: The architecture is a single point of failure. Application and database share one dyno. If it fails, both are down simultaneously. There is no redundancy and no failover mechanism.

**Finding 5: Vendor lock-in — [delta]**

Claim (Doc 1, "Low Vendor Lock-in"): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in." Also Doc 9: "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: Doc 8, DataEnrich.io — "Dependency: 40% of features depend on this API." "Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)." "Termination Notice: 90 days (either party)."

Delta: The product is structurally dependent on a single third-party vendor for 40% of its functionality. No fallback exists. The 90-day termination notice means either party can end the relationship with three months' notice. This is functional lock-in, not a standard SaaS arrangement.

**Finding 6: Backup retention exhaustion — [derived]**

Basis: Doc 9 — "The database is backed up daily to Heroku's managed storage with 30-day retention."
       Doc 4 — "Last Successful Backup: 2026-07-30."

Derivation: 2026-07-30 + 30 days = 2026-08-29. After 2026-08-29, the 2026-07-30 backup is purged under the 30-day retention policy. All data changes made after 2026-07-30 — a minimum of 24 days and growing by one day per day — exist only in the live database with no backup copy.

Consequence: If the database is lost after 2026-08-29, the most recent recoverable state is 2026-07-30, and all subsequent data is permanently lost. With backups still failing and no alerting, the window of unrecoverable data grows daily. The buyer inherits a hard, date-certain data-loss risk.

Escalates: Finding 1

**Finding 7: Active account inflation — [partial]**

Claim (Doc 1, "Key Financials"): "Active Customers: 120."

Evidence: Doc 6 — "Total Accounts Marked 'Active': 120." "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment."

Delta: 23 of the 120 accounts (19.2%) generate no revenue and show no recent activity. The revenue-generating base is 97 accounts (94 Stripe + 3 enterprise wire), not 120. The MRR figure is arithmetically correct, but the account count used to imply scale is inflated.

**Finding 8: MRR arithmetic — [real, minor caveat]**

Claim (Doc 1, "Key Financials"): "Blended MRR: $40,000."

Evidence: Doc 5 — "Pro Plan: Count: 91 accounts, Price: $149/mo, Total MRR: $13,549." 91 × $149 = $13,559, not $13,549. Corrected total: $13,559 + $2,451 (3 × $817) + $24,000 (3 × $8,000 wire) = $40,010.

Delta: The stated MRR is $10 lower than the computed figure. Immaterial to valuation. Noted because it indicates the document was not checked against its own arithmetic.

**Finding 9: Horizontal scaling — [real, operational caveat]**

Claim (Doc 9): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: Doc 4 — "Dyno: `standard-1x` (1GB RAM, 0.5 CPU)." Single dyno in production.

Delta: None. The claim is about capability, not current state. Adding dynos is a one-click operation in Heroku. The operational caveat is that the application is currently running on a single dyno with 1 GB RAM; the buyer should not assume current headroom.

**Finding 10: Managed SSL — [real]**

Claim (Doc 1, "Security"): "Managed SSL." Also Doc 9: "automatic SSL."

Evidence: Doc 4, Security & DNS — "SSL: Heroku-managed, auto-renewed."

Delta: None.

**Finding 11: OAuth authentication — [real]**

Claim (Doc 1, "Security"): "OAuth authentication."

Evidence: Doc 4 — "14 secrets stored in Heroku config vars (… Google OAuth …)." Doc 8 — "Google OAuth: Free tier."

Delta: None.

**Finding 12: Rails 7 stack — [real]**

Claim (Doc 1, Doc 2, Doc 9): "Rails 7."

Evidence: Doc 2 — "The core application is a Rails 7 application hosted on Heroku."

Delta: None.

**Finding 13: Asking price multiple — [real]**

Claim (Doc 1): "Asking Price: $480,000 (12x blended MRR)."

Evidence: 12 × $40,000 = $480,000. Arithmetic checks. The $10 discrepancy in Finding 8 is immaterial to the multiple.

Delta: None.

**Finding 14: Enterprise contract values — [real]**

Claim (Doc 5, Note): "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each."

Evidence: Doc 7 — Acme Retail $8,000/mo; GlobalMart $8,000/mo; ShopStream $8,000/mo. Consistent.

Delta: None.

**Finding 15: Infrastructure cost consistency — [real]**

Claim (Doc 4, Doc 8): Heroku $25/mo; Redis $50/mo; DataEnrich.io $400/mo; Twilio $100/mo; GoDaddy $12/yr; Stripe 2.9% + $0.30/txn.

Evidence: Doc 4 and Doc 8 are consistent on all six figures.

Delta: None.

---

**Remaining Claims (unverifiable)**

- **Churn** (Doc 1: "Low (stable base of recurring subscribers)"). No churn metric, cohort data, or cancellation history is provided in the data room. The 23 phantom accounts and 8 pilot agreements expiring Q3 2026 are relevant context but do not constitute a churn rate. Cannot be verified or falsified from available materials.

- **Documentation quality** (Doc 1: "Well-documented codebase"). No documentation artifacts are included in the data room. Doc 3 shows no code review process, which speaks to process rather than documentation. Cannot be assessed.

---

**Coverage Statement**

59 distinct claims were enumerated from the nine documents. 57 were verified: 15 produced findings (5 [delta], 1 [derived], 1 [partial], 1 [real, minor caveat], 1 [real, operational caveat], 6 [real]), and 42 were checked for cross-document consistency and found consistent (Tier 4 micro-claims: specific dates, counts, configuration values, DNS, secrets inventory, dependency costs, contract terms). 2 claims (churn, documentation quality) are unverifiable from the materials provided. The remainder of the unverified surface is marketing language ("turnkey acquisition," "proven product-market fit," "robust, defensible product") which is outside the scope of technical claims verification.

---

**Questions for the Seller Before Closing**

1. Why did the backup schedule fail on or around 2026-07-30, and why was no alert triggered? What is the remediation plan, and will it be completed before 2026-08-29?
2. What uptime monitoring will be in place post-acquisition? The current state is no monitoring.
3. What is the contingency plan if DataEnrich.io terminates the relationship or changes pricing? 40% of features depend on it with no fallback.
4. Why are 23 accounts marked "active" in the CRM with no payment events in 90 days and no notes on file?
5. Can you provide churn data — monthly cancellation and win-back counts for the trailing 12 months?
6. What is the state of the codebase's documentation? No documentation artifacts were included in the data room.
7. Will the 15 accounts with no signed ToS on file be brought under a signed agreement pre-closing?
