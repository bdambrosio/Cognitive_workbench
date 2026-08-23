## FlowMetrics claims-verification report

### Recommendation: **Conditional**

I identified six material claim gaps: backup continuity, uptime monitoring, active-account status, critical-path test coverage, end-to-end failover, and third-party integration lock-in. The materials also leave low churn, secure payment processing, minimal operational overhead, and several scaling and maintenance assertions unverified. The recommendation is **Conditional** because the identified gaps are specific and potentially addressable, but they affect reliability, customer quality, operational continuity, and buyer transition risk.

This is a claims-verification review, not a penetration test, code-quality review, legal review, or acceptance test.

### Scope and coverage

I enumerated nine data-room documents and approximately 150 micro-claims. I reviewed all nine documents, prioritising safety-critical and availability claims, architectural invariants, operational parameters, then commercial and lower-priority claims. Six claims produced findings. Other claims were classified as supported, partially supported, or remaining claims where the supplied material could not verify them.

The six findings are not six verified claims out of 150: they are six exceptions found while examining the highest-priority claim classes. The remaining unverified claims matter because the data room does not contain repository implementation evidence, monitoring exports, backup-restore tests, payment-control records, churn calculations, or operating-cost data sufficient to close them.

### Findings

**Finding 1: Backup continuity does not match the stated backup claim — [delta]**

Claim (`doc1_seller_listing_description.md:19`; `doc9_seller_s_technical_claims_verbatim.md:10`): “99.9% uptime monitoring and daily automated database backups with 30-day retention”; “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: `doc4_infrastructure_config.md:15-19` — “Schedule: Daily at 2:00 AM via `heroku pg:backups schedule`”; “Status: Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.”

Delta: A daily schedule exists, but the supplied status record shows failures for the last 21 days and no failure alerting. The materials do not establish that the claimed 30-day retention is currently usable.

**Finding 2: Claimed uptime monitoring is not configured — [delta]**

Claim (`doc1_seller_listing_description.md:19`; `doc9_seller_s_technical_claims_verbatim.md:6`): “99.9% uptime monitoring”; “We monitor uptime at 99.9% and maintain daily automated database backups.”

Evidence: `doc4_infrastructure_config.md:21-23` — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)”; “Status Page: Heroku built-in status page only.”

Delta: No application uptime monitor is configured. Heroku’s built-in status page does not establish that FlowMetrics is monitored at 99.9%.

**Finding 3: Active-account count is internally inconsistent — [partial]**

Claim (`doc1_seller_listing_description.md:9, 13`): “We have built a loyal customer base of 120 active accounts”; “Active Customers: 120.”

Evidence: `doc6_crm_export_summary.md:12-18` — “Phantom/Inactive Accounts: 23”; those accounts had “No payment events in the last 90 days,” last login dates “60-120 days ago,” and “All 120 accounts are currently marked as 'active' in the CRM.”

Delta: The CRM status field contains 120 accounts marked active, but the same record identifies 23 as phantom/inactive and supplies inactivity indicators. The materials do not establish that all 120 are operationally active outside the CRM field.

**Finding 4: Comprehensive critical-path test coverage is not present in the supplied repository record — [delta]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:7`): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: `doc3_git_history_summary.md:24-32` — “CI/CD Pipeline: None configured”; “Unit Tests: 12 (all located in `test/utils/`)”; “Integration Tests: 0”; “Payment-Path Tests: 0”; “Staging Environment: None”; “Branch Protection: None”; “Code Review Process: None documented.”

Delta: The supplied repository record documents 12 unit tests confined to `test/utils/`, with zero integration tests and zero payment-path tests. That does not match comprehensive coverage across all critical paths.

**Finding 5: Platform-level redundancy does not establish database failover — [partial]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:5`; `doc2_tech_stack_description_as_provided_by_se.md:3`): “The system has redundancy and automatic failover through Heroku's platform-level process management”; Heroku “provides automatic scaling and failover capabilities at the platform level.”

Evidence: `doc4_infrastructure_config.md:3-8` — “Dyno: `standard-1x` (1GB RAM, 0.5 CPU)”; “Database: `heroku-postgresql:standard-0` (running on the same dyno as the application)”; “Read Replicas: None”; “Separate DB Instance: No.”

Delta: The materials establish Heroku hosting, but do not establish redundant application/database placement or automatic database recovery. The recorded single-dyno topology, no read replica, and no separate database instance leave the end-to-end failover claim only partially supported.

**Finding 6: Google OAuth has no contractual relationship despite the no-lock-in claim — [delta]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:9`): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: `doc8_external_dependency_list.md:25-28` — “Google OAuth”; “Function: Authentication”; “Risk: If Google changes API or revokes app, users cannot log in. No contractual relationship.”

Delta: The dependency record expressly states that Google OAuth has no contractual relationship and that users cannot log in if Google changes its API or revokes the app. That contradicts the blanket no-lock-in claim.

### Confirmed claims and non-deltas

- The DataEnrich dependency is disclosed: 40% of features depend on it, no fallback is implemented, and an outage or pricing change degrades the product to basic mode (`doc8_external_dependency_list.md:7-12`).
- The MRR arithmetic reconciles from the supplied figures: `$16,000` Stripe MRR plus three enterprise contracts at `$8,000/mo` each equals `$40,000` (`doc5_stripe_export_summary.md:3-5, 17-19`; `doc7_customer_contracts_summary.md:3-27`). This verifies the arithmetic, not the durability of that revenue.
- SSL is recorded as Heroku-managed and auto-renewed; DNS is recorded as GoDaddy-managed (`doc4_infrastructure_config.md:25-28`). This supports the narrow SSL/managed-DNS claim. It does not remove the continuity caveat that DNS is personally managed by Dave with no secondary provider (`doc8_external_dependency_list.md:29-32`).

### Remaining Claims

These are not additional deltas. The available materials did not settle them:

- **Low churn:** the listing says “Low,” but provides no churn rate, cohort analysis, cancellation history, or period definition (`doc1_seller_listing_description.md:9, 14`; `doc7_customer_contracts_summary.md:29-34`).
- **Secure payment processing:** Stripe activity and recurring billing are evidenced, but the data room does not provide security-control evidence, refund/dispute handling, failed-payment handling, or payment-path implementation tests (`doc1_seller_listing_description.md:20`; `doc2_tech_stack_description_as_provided_by_se.md:5`; `doc3_git_history_summary.md:24-29`; `doc5_stripe_export_summary.md:3-19`).
- **Minimal operational overhead:** no staffing, support-load, operating-cost, or process data was supplied (`doc1_seller_listing_description.md:9`).
- **Horizontal scaling and one-click scaling:** the claim is stated, but the supplied records do not verify scaling tests, dyno configuration history, capacity limits, or scaling outcomes (`doc9_seller_s_technical_claims_verbatim.md:8`; `doc2_tech_stack_description_as_provided_by_se.md:3, 5`).
- **Code documentation:** “well-documented” is not established by the supplied git summary (`doc9_seller_s_technical_claims_verbatim.md:7`; `doc3_git_history_summary.md:24-32`).
- **OAuth implementation and authentication controls:** the dependency record confirms Google OAuth as a dependency, but does not verify authorization, session, account-recovery, or access-control behavior.
- **Secrets controls:** the infrastructure record says 14 secrets are in Heroku config vars and that there is no secrets vault (`doc4_infrastructure_config.md:25-29`). The materials do not state a seller claim precise enough to classify this as a claim delta.
- **Contract and pilot durability:** 94 subscriptions auto-renew; eight pilots are free 90-day trials expiring Q3 2026; and 15 upgraded users have no signed ToS on file (`doc7_customer_contracts_summary.md:29-45`). These are buyer-relevant facts, but the supplied documents do not provide a matching blanket claim whose truth they independently disprove.

### Questions for the seller before closing

1. Provide backup execution logs, retention evidence, and a successful restore test; explain the 21-day failure period and why no alert fired.
2. Provide uptime-monitor configuration, historical uptime reports, and the measurement definition behind 99.9%.
3. Reconcile the 23 phantom/inactive CRM accounts and provide churn calculations by month for the last 12 months.
4. Provide the actual deployment repository or a reproducible repository export, including integration and payment-path tests, staging configuration, and deployment history.
5. Demonstrate application and database failover, including recovery-point and recovery-time objectives.
6. Provide the Google OAuth ownership/transfer arrangements and explain the user-login contingency if Google revokes or changes the application.
7. Provide Stripe control documentation and records for failed payments, refunds, disputes, chargebacks, webhook handling, and reconciliation.
8. Provide signed ToS evidence for the 15 upgraded users and clarify the conversion status of the eight pilots.
9. Provide staffing, support, infrastructure-cost, and operational-process records supporting “minimal operational overhead.”
