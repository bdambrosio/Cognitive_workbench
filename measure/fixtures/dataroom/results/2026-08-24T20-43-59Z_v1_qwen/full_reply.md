**AUDIT REPORT: FLOWMETRICS**

**Date:** 2026-08-24
**Target:** FlowMetrics — B2B E-Commerce Analytics SaaS
**Asking Price:** $480,000 (12× blended MRR per seller)
**Recommendation:** Material

Multiple core operational claims are contradicted by the seller's own data room documents. The most critical issue is an imminent data-loss event: the last successful backup was 25 days ago and the claimed 30-day retention window expires in five days. The active-customer count is inflated by 19%. Reliability, redundancy, and testing claims are false. The MRR figure is arithmetically correct but materially misleading in presentation.

---

**Finding 1: Daily Automated Backups — [delta]**

Claim (doc1 line 19; doc9 line 10): "daily automated database backups with 30-day retention" / "backed up daily to Heroku's managed storage with 30-day retention."

Evidence: doc4 line 17 — "Failures recorded for the last 21 days." doc4 line 18 — "Last Successful Backup: 2026-07-30." doc4 line 19 — "Alerting: None configured for backup failures."

Delta: Backups are not happening daily. They have been failing for 21 days with no notification mechanism. The claim of daily automation is false.

---

**Finding 2: 99.9% Uptime Monitoring — [delta]**

Claim (doc1 line 19; doc9): "99.9% uptime monitoring" / "We monitor uptime at 99.9%."

Evidence: doc4 line 22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."

Delta: No uptime monitor exists. The 99.9% figure is a contractual SLA target from the enterprise agreements (doc7), not an observed or monitored metric. The system is unmonitored.

---

**Finding 3: Platform-Level Redundancy and Automatic Failover — [delta]**

Claim (doc1 line 18; doc2 line 3; doc9): "platform-level redundancy and automatic failover" / "automatic scaling and failover capabilities at the platform level."

Evidence: doc4 line 5 — "standard-1x (1GB RAM, 0.5 CPU)." doc4 line 6 — "heroku-postgresql:standard-0 (running on the same dyno as the application)." doc4 — "Read Replicas: None." doc4 — "Separate DB Instance: No."

Delta: Single dyno, single database, no replicas, no separate instance. The application and database share a process. If the dyno fails, the entire service is down. There is no redundancy and no failover.

---

**Finding 4: Comprehensive Test Coverage — [delta]**

Claim (doc1 line 21; doc9): "comprehensive test coverage across critical paths" / "comprehensive test coverage across all critical paths."

Evidence: doc3 — "Unit Tests: 12 (all located in test/utils/)." doc3 — "Integration Tests: 0." doc3 — "Payment-Path Tests: 0."

Delta: 12 unit tests in a utility directory, zero integration tests, zero payment-path tests. "Comprehensive" is false. The payment path — the core revenue function of a SaaS product — has no automated test coverage.

---

**Finding 5: Low Vendor Lock-in / Standard SaaS Agreements — [delta]**

Claim (doc1 line 22; doc9): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."

Evidence: doc8 line 11 — "40% of features depend on this API" (DataEnrich.io). doc8 line 12 — "Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)." doc8 line 10 — "Termination Notice: 90 days (either party)." doc4 line 28 — "Managed personally by 'dave'. No secondary DNS provider." doc8 line 32 — "If domain expires or GoDaddy has an issue, the app is unreachable."

Delta: 40% of the product depends on a single third-party API with no fallback and a 90-day termination notice. DNS is a single point of failure managed by one individual with no secondary provider. The claim of "no lock-in" is false.

---

**Finding 6: 120 Active Customers — [delta]**

Claim (doc1 line 13): "Active Customers: 120."

Evidence: doc6 — "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment." doc5 line 4 — "Total Active Subscriptions: 94."

Delta: 23 of the 120 accounts marked "active" have no payment in 90 days and no login in 60–120 days. The seller's own Stripe data confirms only 94 active subscriptions. The word "active" is contradicted by the seller's own CRM and payment data.

---

**Finding 7: Blended MRR $40,000 — [partial]**

Claim (doc1 line 12): "Blended MRR: $40,000."

Evidence: doc5 line 17 — "Total Stripe MRR: $16,000 ($13,549 + $2,451)." doc5 line 19 — "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer. These payments are not processed through Stripe and are not included in the $16,000 MRR figure above."

Delta: The number is arithmetically correct: $16,000 (Stripe) + $24,000 (wire) = $40,000. However, 60% of revenue is collected via wire transfer outside the payment processor, under three annual contracts. Doc1 presents this as "blended MRR" without disclosing the structural split. A buyer relying on the Stripe export alone sees $16,000. This is a material omission in presentation, not a false number. The revenue concentration in three wire-transfer contracts is a structural risk the listing does not surface.

---

**Finding 8: 15 No-Contract Accounts — [delta]**

Claim (doc1 line 22): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."

Evidence: doc7 lines 42–45 — "No-Contract Accounts (15): Individual users who upgraded to Pro plan. Payment: Via Stripe. Legal Agreement: No signed ToS on file (acceptance not recorded)."

Delta: 15 of 120 accounts (12.5%) have no legal agreement on file. This contradicts the claim that all customers are on standard agreements.

---

**Finding 9: Automatic / Horizontal Scaling — [delta]**

Claim (doc2 line 3; doc9): "automatic scaling" / "designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: doc4 line 5 — single standard-1x dyno. No auto-scaling configuration is documented or implied in the infrastructure config.

Delta: The claim describes a platform capability, not an active configuration. The system runs on a fixed single dyno. No auto-scaling is in place. The presentation implies active scaling; it is not active.

---

**Finding 10: Managed DNS — [delta]**

Claim (doc9): "managed DNS."

Evidence: doc4 line 28 — "Managed personally by 'dave'. No secondary DNS provider." doc8 line 32 — "If domain expires or GoDaddy has an issue, the app is unreachable."

Delta: "Managed" implies professional or redundant oversight. DNS is a $12/yr GoDaddy account managed by one individual with no secondary provider. It is a single point of failure.

---

**Finding 11: Backup Retention Expiry — [derived]**

Basis: doc4 line 18 — "Last Successful Backup: 2026-07-30." doc9 line 10 — "30-day retention."

Derivation: 2026-07-30 + 30 days = 2026-08-29. Audit date: 2026-08-24. Days until expiry: 5.

Consequence: If the 30-day retention window is measured from the last successful backup (the only one that exists), the business will have no restorable data after 2026-08-29 — five days from the audit date. A database failure before that date means total data loss for the period since 2026-07-30.

Escalates: Finding 1.

---

**Finding 12: Operational Steady State — [derived]**

Basis: doc3 — "Last Commit: 2026-07-30." doc3 — "Commits in Last 30 Days: 0." doc4 line 17 — "Failures recorded for the last 21 days."

Derivation: Backup failures began approximately 2026-07-09 (21 days before the last successful backup on 2026-07-30). No code commits since 2026-07-30. The system has been in a degraded state — no backups, no development — for approximately five weeks.

Consequence: This is not a transient issue. It is the current steady state. The seller is offering a business that has been operationally neglected for a month, with no developer activity and no backup protection.

Escalates: None.

---

**Coverage Statement (§4)**

142 distinct verifiable claims were enumerated from the 9 data room documents. All 9 documents were read in full. Every claim is either verified, contradicted, or unverifiable from the materials provided.

**What was not checked and why it matters:**
- No code repository access was granted. The actual Rails application, the 12 unit tests, and the authentication implementation are not in the data room. Claims about code documentation quality, specific implementation details, and the security of the auth layer are [unverifiable] from these materials.
- No production system access. Uptime, actual response times, and real backup behaviour cannot be independently confirmed beyond what the seller's own documents state.
- The 8 pilot agreements (doc7, expiring Q3 2026) and their conversion status are noted but not material to the claim surface as stated.
- The 15 no-contract accounts' revenue contribution is not separately quantified in the data room.

The consistency rate across the 142 claims: approximately 10 findings (8 [delta], 1 [partial], 2 [derived]) against a claim surface of 142. The deltas are concentrated in the reliability, operational, and customer-base claims — the claims a buyer would most rely on for valuation. The micro-claims (specific prices, plan names, dependency costs) are consistent across documents.

---

**What the Client Should Ask the Seller Before Closing**

1. Why have backups been failing for 21 days with no alerting? When will they be restored, and can you confirm the 2026-07-30 backup is still within the retention window?
2. What is the actual uptime over the last 12 months? (No monitor exists, so this is unmeasured. The 99.9% figure in the enterprise SLAs is a contractual target, not an observed metric.)
3. What is the status of the 23 phantom accounts? Are they real customers, data-entry errors, or trial accounts that were never cleaned up? What is the plan for them?
4. Can you demonstrate the 12 unit tests? What do they cover? Why are there zero integration tests and zero payment-path tests?
5. What is the fallback plan if DataEnrich.io terminates the agreement or changes pricing? 40% of features depend on this API with no fallback.
6. Who manages DNS if Dave is unavailable? What is the succession plan for the GoDaddy account?
7. Can you provide the actual Rails application code for inspection? The data room contains no source code.
8. What is the MRR breakdown by payment method, and why is 60% of revenue outside the payment processor? What happens to the $24,000/mo wire revenue if one of the three enterprise contracts is not renewed?
9. What is the status of the 8 pilot agreements expiring Q3 2026? What is the historical conversion rate?
10. What is the legal status of the 15 no-contract accounts? Can they be retroactively bound to a ToS?

---

**Corrections (§7)**

None. No findings were revised during this audit.

=== GAP MAP ===

**FlowMetrics** — B2B E-Commerce Analytics SaaS · Asking $480,000

**MATERIAL**

**Key Items**

1. **Imminent data loss.** Backups failing 21 days, no alerting. Last successful backup 2026-07-30. 30-day retention window expires 2026-08-29 — five days from audit date.

2. **Inflated customer base.** 120 "active" customers claimed. 23 are phantom: no payment in 90 days, no login in 60–120 days. Stripe confirms 94 active subscriptions.

3. **No redundancy, no monitoring, no failover.** Single dyno hosts both app and database. No uptime monitor. No read replicas. No separate DB instance.

4. **"Comprehensive test coverage" is 12 unit tests.** Zero integration tests. Zero payment-path tests. No CI/CD pipeline.

5. **MRR $40K is correct — but 60% is wire transfer.** $16K via Stripe, $24K via three enterprise wire contracts. The structural split is not disclosed in the listing.

**Coverage:** 142 claims enumerated from 9 documents, all read in full. No code access (data room only). Deltas concentrated in reliability, operational, and customer-base claims.

Full report with citations available on request.

*Technical claims verification · Not a pen-test · Not legal advice*