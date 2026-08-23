## FlowMetrics Data Room Audit — Full Report

**Recommendation: DO NOT PROCEED as presented.**

The seller's technical and operational claims are contradicted by the data room's own evidence documents in every category where a check was possible. The MRR figure is arithmetically supported but rests on unverified wire transfers and a customer base inflated by 23 phantom accounts. The infrastructure does not support the reliability, redundancy, or backup claims made in the listing. This is not a matter of one or two overstated features; it is a systematic gap between the claims presented and the system described in the seller's own documents.

---

### Findings (worst first)

**F1 — Backups have been failing for 21 days; no alerting exists.
Verdict: DELTA (material)**

Seller claim (doc9:10): "The database is backed up daily to Heroku's managed storage with 30-day retention."
Seller claim (doc1:19): "99.9% uptime monitoring and daily automated database backups with 30-day retention."
Evidence (doc4:17): "Status: Failures recorded for the last 21 days."
Evidence (doc4:18): "Last Successful Backup: 2026-07-30"
Evidence (doc4:19): "Alerting: None configured for backup failures."

Delta: The backup system the seller represents as a daily automated process with 30-day retention has not produced a successful backup in over three weeks, and no one is alerted when it fails. A buyer inheriting this system has no recoverable data state from the last 21+ days. This is the single most expensive delta in the data room: if the database is corrupted or lost today, the last known-good state is 2026-07-30.

---

**F2 — No uptime monitoring exists.
Verdict: DELTA (material)**

Seller claim (doc9:6): "We monitor uptime at 99.9% and maintain daily automated database backups."
Seller claim (doc1:19): "99.9% uptime monitoring and daily automated database backups with 30-day retention."
Evidence (doc4:22): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."

Delta: There is no tool, service, or script monitoring whether the application is up. The claim of "99.9% uptime monitoring" has no referent in the infrastructure. The enterprise contracts (doc7) impose 99.9% SLAs on Acme Retail and GlobalMart with service-credit remedies and 30-day termination rights for material SLA breach. The seller cannot demonstrate compliance with the SLAs they have contracted to meet because no measurement exists.

---

**F3 — Single point of failure: co-located DB and app on one dyno.
Verdict: DELTA (material)**

Seller claim (doc9:5): "The system has redundancy and automatic failover through Heroku's platform-level process management."
Seller claim (doc1:18): "Built on a modern Rails 7 stack with platform-level redundancy and automatic failover."
Evidence (doc4:5): "Dyno: standard-1x (1GB RAM, 0.5 CPU)"
Evidence (doc4:6): "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)"
Evidence (doc4:7): "Read Replicas: None."
Evidence (doc4:8): "Separate DB Instance: No."

Delta: The application and its database share a single dyno. If that dyno crashes, both the app and the data are down simultaneously. There is no failover, no replica, no separate instance. "Platform-level redundancy" in Heroku's process management refers to dyno restart on crash — it does not protect a co-located database from data loss during an unclean shutdown, and it does not provide failover to a second instance. The claim is a misreading of what Heroku's platform guarantees.

---

**F4 — 23 phantom accounts inflate the customer base from 97 to 120.
Verdict: DELTA (material to valuation)**

Seller claim (doc1:13): "Active Customers: 120"
Evidence (doc6): "Phantom/Inactive Accounts: 23 — Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment."
Evidence (doc6): "Status Field: All 120 accounts are currently marked as 'active' in the CRM."

Delta: 23 of the 120 "active" accounts have no payment activity in 90 days, no login in 60–120 days, and no documentation of their origin. The verifiable active customer base is 97 (94 Stripe + 3 enterprise wire). The 120 figure inflates the customer count by 24% and, at $149/mo per Pro account, implies up to $3,427/mo of revenue that does not exist. The 12x multiple in the asking price ($480k = 12 × $40k) is computed on the $40k figure which is arithmetically correct only if all 120 accounts are genuinely active and paying.

---

**F5 — Test coverage claim is unsupported.
Verdict: DELTA**

Seller claim (doc9:7): "The codebase is well-documented with comprehensive test coverage across all critical paths."
Seller claim (doc1:21): "Well-documented codebase with comprehensive test coverage across critical paths."
Evidence (doc3:27): "Unit Tests: 12 (all located in test/utils/)"
Evidence (doc3:28): "Integration Tests: 0"
Evidence (doc3:29): "Payment-Path Tests: 0"
Evidence (doc3:30): "Staging Environment: None."
Evidence (doc3:31): "Branch Protection: None."

Delta: There are 12 unit tests, all in a utils directory. There are zero integration tests and zero tests covering the payment path — the most critical business logic in a subscription SaaS. No staging environment means changes go directly to production. No branch protection means any contributor can push to main. "Comprehensive test coverage across all critical paths" is contradicted by the absence of any test touching a critical path.

---

**F6 — Vendor lock-in: 40% of features depend on one API with no fallback.
Verdict: DELTA**

Seller claim (doc9:9): "All third-party integrations are on standard SaaS agreements with no lock-in."
Seller claim (doc1:22): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."
Evidence (doc8:4): "DataEnrich.io — Dependency: 40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)."

Delta: The contractual agreement may be standard, but the functional dependency is not. 40% of the product's features require DataEnrich.io. If that vendor raises prices, changes the API, or goes out of business, the product loses a significant portion of its value proposition with no fallback. "No lock-in" in the contractual sense does not mean no lock-in in the operational sense.

---

**F7 — Horizontal scaling claim is misleading.
Verdict: DELTA**

Seller claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."
Evidence (doc4:6): "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)"

Delta: Adding application dynos is trivially possible, but the database does not scale with them. The co-located DB means the system cannot actually handle more load without first migrating the database to a separate, scalable instance. The scaling claim is true only for the application tier and false for the data tier.

---

**F8 — No CI/CD, no code review, no staging.
Verdict: DELTA (operational risk)**

Seller claim (doc2:3): "no manual intervention required for routine operations"
Evidence (doc3:25): "CI/CD Pipeline: None configured."
Evidence (doc3:30): "Staging Environment: None."
Evidence (doc3:32): "Code Review Process: None documented."
Evidence (doc3:22): "Commits in Last 30 Days: 0"

Delta: "No manual intervention required" is contradicted by the absence of any automated deployment pipeline. All deploys are manual. There is no staging environment to test changes. No code review process is documented. The last commit was 2026-07-30 — 23 days before this audit — and zero commits in the last 30 days. The system is not under active development.

---

**F9 — DNS is a single point of failure managed by an individual.
Verdict: DELTA (operational risk)**

Seller claim (doc9:11): "We use a modern Rails 7 stack with automatic SSL and managed DNS."
Evidence (doc4:28): "DNS Management: Managed personally by 'dave'. No secondary DNS provider."
Evidence (doc8:14): "GoDaddy — Risk: Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable."

Delta: "Managed DNS" implies an automated, resilient service. In practice, DNS is managed by one individual through GoDaddy with no secondary provider. If Dave's GoDaddy account lapses, or if Dave is unavailable, the entire platform is unreachable. This is a key-person dependency on a $12/yr service.

---

**F10 — MRR is arithmetically supported but contingent on unverified wire transfers.
Verdict: PARTIALLY CONFIRMED (with material caveat)**

Seller claim (doc1:12): "Blended MRR: $40,000"
Evidence (doc5): "Total MRR (Stripe): $16,000" plus "The 3 enterprise companies... have separate contracts for $8,000/mo each, paid via wire transfer."
Evidence (doc7): Three enterprise contracts at $8,000/mo each confirmed.

The arithmetic holds: $16,000 + $24,000 = $40,000. However, the $24,000 in wire transfers is not verifiable from any document in the data room. There are no bank statements, no wire transfer confirmations, no ACH records. The buyer is asked to accept $24,000/mo of revenue based solely on contract documents. Additionally, doc5 notes a $10 internal arithmetic discrepancy (91 × $149 = $13,559, not $13,549), which is immaterial but indicates the export was not independently verified.

---

**F11 — Enterprise SLA exposure without monitoring capability.
Verdict: DELTA (compounding with F2)**

Evidence (doc7): Acme Retail and GlobalMart contracts specify 99.9% SLA with "Service credit of 10% of monthly fee per 0.1% below SLA" and "30-day notice for material SLA breach."
Evidence (doc4:22): "Uptime Monitor: None."

The seller has contracted to 99.9% uptime for two of their three enterprise customers but has no means of measuring whether that SLA is being met. A buyer inherits the risk of SLA breach claims with no data to defend against them.

---

### Coverage Statement

**What I did not check and why it matters:**

- **Actual bank statements / wire transfer records.** The $24,000/mo in enterprise revenue is supported only by contract documents, not by payment evidence. Without bank records, the MRR figure is unverified for 60% of its value. This is the single largest coverage gap.
- **Actual application code.** The data room contains no source code. Claims about documentation quality, code structure, and the 12 unit tests are based solely on the git history summary (doc3). I cannot verify whether the code is actually well-documented or whether the 12 tests are meaningful.
- **Actual Stripe dashboard / transaction history.** Doc5 is a summary export. I cannot verify whether the 94 subscriptions are genuinely active or whether any have been dunning for failed payments.
- **DataEnrich.io contract terms.** Doc8 lists a 90-day termination notice and $400/mo cost, but I have not seen the actual agreement. The claim of "standard SaaS agreement" is unverified.
- **Google OAuth app status.** Doc8 notes "No contractual relationship" and that revocation would lock users out. I have not verified whether the OAuth app is in good standing or whether it is subject to Google's API terms.
- **Churn data.** Doc1 claims "Low (stable base of recurring subscribers)" but no churn metrics, cohort data, or cancellation history is provided in any document. This claim is unverifiable from the data room.
- **The 8 pilot agreements and 15 no-contract accounts** (doc7) are noted but not deeply analyzed. The 15 accounts with "No signed ToS on file (acceptance not recorded)" represent a legal exposure that I flag but do not fully assess.

---

### Questions for Dave Before Closing

1. **Backups:** When was the last successful backup, and why did the backup schedule fail 21 days ago with no alert? Can you demonstrate a successful backup and restore right now? What is the RPO (recovery point objective) you are actually operating at?

2. **Uptime:** What is your actual uptime over the last 90 days? If you have no monitoring tool, how do you know you are meeting the 99.9% SLA in the Acme Retail and GlobalMart contracts? Have any SLA breach notifications been received?

3. **Wire transfers:** Can you provide 12 months of bank statements showing the $8,000/mo wire transfers from Acme Retail, GlobalMart, and ShopStream? Are all three currently in good standing, or are any in dispute or dunning?

4. **Phantom accounts:** What are the 23 accounts marked active in the CRM with no payment in 90 days and no login in 60–120 days? How were they created in Q1 2026? Why are they still marked active? Should they be excluded from the customer count?

5. **Database architecture:** Why is the database on the same dyno as the application? What is the plan to separate them? What happens to data integrity if the dyno crashes mid-write?

6. **DataEnrich.io:** What is the actual contract with DataEnrich.io? What happens to the 40% of features that depend on it if the vendor changes pricing or terminates? Is there a migration plan?

7. **Development activity:** Zero commits in the last 30 days. Is the system under active development? Who maintains it day-to-day? What happens if Dave is unavailable?

8. **No-contract accounts:** 15 Pro-plan users have no signed ToS on file. What is the legal basis for billing them? What is the exposure if any dispute a charge?

9. **Churn:** What is your actual monthly churn rate over the last 12 months? Can you provide cohort retention data?

10. **Key-person dependency:** The system is 73% Dave's commits, DNS is managed personally by Dave, and there is no documentation of operational procedures. If Dave leaves, what is the buyer's path to operational independence?

---
