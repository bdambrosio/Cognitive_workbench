Here is the full deliverable. Report first, Gap Map after the marker line.

---

**FlowMetrics — AI-Readiness Audit**

**RECOMMENDATION: Conditional**

The seller's financials are real and the stack is what it says it is. But three of the reliability and quality claims are false as stated — backups have not run in 24 days, no uptime monitoring exists, and the test suite does not cover any critical path. These are addressable in a sprint or two and do not change the core revenue picture, but they indicate a pattern of overstatement in the listing. The derived finding (last good backup expires 2026-08-29) makes the backup issue time-critical.

---

**FINDINGS (worst first)**

**Finding 1: Daily automated backups not running — [delta]**

Claim (doc1_seller_listing_description.md line 19): "99.9% uptime monitoring and daily automated database backups with 30-day retention." Also doc9_seller_s_technical_claims_verbatim.md: "The database is backed up daily to Heroku's managed storage with 30-day retention."

Evidence: doc4_infrastructure_config.md lines 15–19 — Schedule: Daily at 2:00 AM via `heroku pg:backups schedule`. Status: "Failures recorded for the last 21 days." Last Successful Backup: 2026-07-30. Alerting: "None configured for backup failures."

Delta: The backup schedule is configured but not succeeding. The document reports 21 consecutive days of failures; the last successful backup was 2026-07-30. As of this audit date (2026-08-23), 24 days have elapsed since the last good backup. No alerting is configured, so failures are silent — nobody is notified. The claim of daily automated backups is false in practice.

**Finding 2: No uptime monitoring — [delta]**

Claim (doc1 line 19): "99.9% uptime monitoring." Also doc9: "We monitor uptime at 99.9%."

Evidence: doc4 lines 21–23 — Monitoring section: "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." "Status Page: Heroku built-in status page only."

Delta: No application uptime monitoring exists. The Heroku built-in status page reports platform-level status and does not monitor whether the FlowMetrics application is responding, generating alerts, or meeting any SLA. The claim of monitoring at 99.9% is false. There is no mechanism to measure, let alone guarantee, that figure.

**Finding 3: No test coverage on critical paths — [delta]**

Claim (doc1 line 21): "Well-documented codebase with comprehensive test coverage across critical paths." Also doc9: "The codebase is well-documented with comprehensive test coverage across all critical paths."

Evidence: doc3_git_history_summary.md lines 24–32 — CI/CD Pipeline: None configured. Unit Tests: 12 (all located in `test/utils/`). Integration Tests: 0. Payment-Path Tests: 0. Staging Environment: None. Branch Protection: None. Code Review Process: None documented.

Delta: The test suite consists of 12 unit tests, all in `test/utils/`. Zero integration tests. Zero payment-path tests. No CI/CD pipeline, no staging environment, no branch protection, no documented code review. The critical paths — payment processing, authentication, data enrichment — are entirely untested. The claim of comprehensive test coverage across critical paths is false.

**Finding 4: 120-account figure includes 23 non-paying accounts — [partial]**

Claim (doc1 line 13): "Active Customers: 120." Also doc1 line 11: "a loyal customer base of 120 active accounts."

Evidence: doc6_crm_export_summary.md — Total Accounts Marked 'Active': 120. Stripe-Linked Accounts: 94. Enterprise Accounts (Wire Transfer): 3. Phantom/Inactive Accounts: 23. The 23 phantom accounts: created in Q1 2026, no payment events in the last 90 days, last login dates 60–120 days ago, no notes or memos on file.

Delta: 94 Stripe + 3 wire = 97 paying accounts. The 23 additional accounts are marked 'active' in the CRM but have had no payment in 90 days and no login in 60–120 days. The blended MRR of $40,000 is supported by payment data (verified: $16,000 Stripe + 3×$8,000 wire = $40,000; see doc5 and doc7). The revenue is real. The account count that the 12x multiple is applied to is inflated by 23 non-paying accounts.

**Finding 5: Single dyno, no redundancy — [partial]**

Claim (doc1 line 18): "Built on a modern Rails 7 stack with platform-level redundancy and automatic failover." Also doc9: "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: doc4 lines 5–6 — Dyno: `standard-1x` (1GB RAM, 0.5 CPU). Database: `heroku-postgresql:standard-0` (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No.

Delta: A single dyno with the database co-located has no failover target. Heroku's platform-level process management restarts a crashed dyno; it does not provide redundancy across multiple instances. With one dyno, if it crashes, there is no second instance to serve traffic, and the database is on the same dyno, so a crash takes both application and data layer down simultaneously. The claim of redundancy is not supported by the configuration.

**Finding 6: No-lock-in claim contradicted by dependency profile — [partial]**

Claim (doc1 line 22): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in." Also doc9: "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: doc8_external_dependency_list.md — DataEnrich.io: Cost $400/mo flat. Termination Notice: 90 days (either party). Dependency: 40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode. Google OAuth: If Google changes API or revokes app, users cannot log in. No contractual relationship. GoDaddy DNS: Managed personally by 'dave'. No secondary DNS.

Delta: 40% of product features depend on a single vendor with a 90-day termination notice and no fallback. Authentication depends on Google with no contractual relationship. DNS is managed personally by one individual with no secondary provider. The claim of no lock-in is contradicted by the dependency profile.

**Finding 7: Last good backup expires 2026-08-29 — [derived]**

Basis: doc4 line 18 — "Last Successful Backup: 2026-07-30"; doc1 line 19 / doc9 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. After that date, the last known good backup is outside the claimed retention window. If the retention claim is accurate and backups continue to fail, no backup will exist at all.

Consequence: As of 2026-08-23, there are 6 days remaining before the last good backup expires from retention. A data loss event after 2026-08-29, in the absence of a working backup, is unrecoverable. This is the most time-critical item in this report.

Escalates: Finding 1

---

**VERIFIED CLAIMS (claim holds; no finding)**

- Rails 7 on Heroku. (doc1 line 18, doc2, doc9 vs doc3 directory structure, doc4 hosting config.)
- Managed SSL, auto-renewed. (doc1 line 20, doc9 vs doc4 SSL section.)
- OAuth authentication via Google. (doc1 line 20, doc8 Google OAuth entry.)
- Secure payment processing via Stripe. (doc1 line 20, doc2, doc5, doc8 Stripe entry.)
- PostgreSQL database. (doc2, doc4.)
- Redis for session caching. (doc2, doc4.)
- Third-party data enrichment API for real-time product data. (doc2, doc8 DataEnrich.io entry.)
- Blended MRR $40,000. (doc1 line 12 vs doc5: $16,000 Stripe MRR + doc7: 3×$8,000 wire = $40,000. Arithmetic verified.)

---

**REMAINING CLAIMS (unverifiable from data room; not findings)**

- Low churn, stable base (doc1 line 14). No churn data, cohort analysis, or cancellation records in any document.
- Proven product-market fit (doc1 line 24). Marketing language; no retention, NPS, or competitive data provided.
- Robust, defensible product (doc1 line 24). Subjective; no IP, patent, or competitive moat documentation.
- Well-documented codebase (doc1 line 21, doc9). No documentation content provided in the data room; only the git summary is available.
- 30-day retention specifically (doc1 line 19, doc9). The retention period is claimed but not stated in doc4's infrastructure config. Functionally moot given Finding 1, but the specific figure cannot be independently confirmed.

---

**COVERAGE STATEMENT**

24 distinct claims were identified across the nine documents. 22 were individually verified or found: 7 findings (3 delta, 3 partial, 1 derived), 8 verified claims, and 2 structural notes. 2 claims remain unverifiable from the data room (documentation content, product-market fit). Coverage: 22/24 = 92%. The remaining 2 are marketing language or require materials not in the data room, and neither is load-bearing for the valuation decision.

---

**WHAT THE BUYER SHOULD ASK THE SELLER BEFORE CLOSING**

1. Why have the backups been failing since 2026-07-30, and why was no alerting configured? Has a manual backup been taken since? If not, the 2026-08-29 expiry is live.
2. What is the actual uptime over the last 90 days? If no monitoring exists, can the seller provide Heroku's platform-level logs as a proxy?
3. What is the plan for the 23 phantom accounts? Are they to be removed from the active count at closing, and does the purchase price adjust accordingly?
4. What is the migration plan for DataEnrich.io if the 90-day termination notice is triggered? Is there a budget for a fallback or re-implementation of the 40% of features that depend on it?
5. Will the seller commit to a working backup system, an uptime monitor, and a minimum test suite (payment path, auth, data enrichment) as a condition of closing?

---

**SCOPE DISCLAIMER**

This is a technical claims-verification audit. It is not a penetration test, not a code-quality review, and not legal advice. The scope is: stated claims in the seller's documents versus observed evidence in the data room, with citations. The buyer has the right to commission a full security assessment and a full code review as separate engagements.
