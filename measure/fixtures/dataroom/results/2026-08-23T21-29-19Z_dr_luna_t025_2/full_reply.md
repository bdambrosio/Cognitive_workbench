# FlowMetrics Claims-Verification Report

## Recommendation: Material

I identified approximately 68 micro-claims across the nine-document data room. I verified the load-bearing safety, architecture, operational, commercial, and selected micro-claim categories first. The reported material gaps are concentrated in backup recoverability, uptime monitoring, testing, failover architecture, customer-account quality, and dependency exposure. The stated $40,000 MRR reconciles; that is not a revenue delta.

This is a claims-verification review, not a penetration test, code-quality review, legal opinion, or acceptance test.

## Findings

**Finding 1: Backup retention and recoverability — [delta]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:10`): “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: `doc4_infrastructure_config.md:15-19` — the configuration records a daily backup schedule, failures for the last 21 days, the last successful backup on 2026-07-30, and no backup-failure alerting.

Delta: The stated 30-day operational backup protection is not evidenced in the current record. The record instead shows 21 consecutive days of failed backups, a last successful backup dated 2026-07-30, and no failure alerting. The separate stack description also says daily scheduled backups (`doc2_tech_stack_description_as_provided_by_se.md:5`), but does not establish successful execution or 30-day retention.

**Finding 2: Uptime monitoring — [delta]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:6`): “We monitor uptime at 99.9% and maintain daily automated database backups.”

Evidence: `doc4_infrastructure_config.md:21-23` — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)” and “Status Page: Heroku built-in status page only.”

Delta: The data room does not show an uptime-monitoring system capable of supporting the stated monitoring claim. Heroku’s status page is the only listed mechanism. The 99.9% figure is therefore unsupported by the implementation record, even though the claim is stated.

**Finding 3: Critical-path test coverage — [delta]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:7`): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: `doc3_git_history_summary.md:24-32` — no CI/CD pipeline; 12 unit tests, all in `test/utils/`; zero integration tests; zero payment-path tests; no staging environment; no branch protection; and no documented code-review process.

Delta: The recorded test inventory does not support “comprehensive test coverage across all critical paths.” In particular, the records show zero integration tests and zero payment-path tests. The absence of CI/CD, staging, and documented review further qualifies the operational basis for the claim.

**Finding 4: Automatic failover and redundancy — [partial]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:5`): “The system has redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: `doc2_tech_stack_description_as_provided_by_se.md:3` — the Rails application is hosted on Heroku, which is said to provide “automatic scaling and failover capabilities at the platform level.” `doc4_infrastructure_config.md:4-8` records one `standard-1x` dyno, PostgreSQL running on the same dyno as the application, no read replicas, and no separate database instance.

Delta: Heroku platform capability may provide process-level handling, but the deployed configuration does not evidence redundant application capacity or database redundancy. The claim is therefore only partially supported: platform-level failover language exists, while the recorded single-instance application/database arrangement leaves a structural exception for database and shared-instance failure.

**Finding 5: No third-party lock-in — [partial]**

Claim (`doc9_seller_s_technical_claims_verbatim.md:9`): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: `doc8_external_dependency_list.md:7-12` — DataEnrich.io supports “40% of features,” requires 90 days’ termination notice, and has “Fallback: None implemented”; if unavailable or repriced, the product degrades to basic mode. `doc8_external_dependency_list.md:25-32` records Google OAuth authentication with no contractual relationship and GoDaddy DNS with no secondary DNS provider; if GoDaddy has an issue or the domain expires, the application is unreachable.

Delta: The broad no-lock-in claim is not supported across the dependency set. DataEnrich.io is a material functional dependency with no fallback, and DNS and authentication introduce operational dependencies. The records do not establish that every integration is freely replaceable without functional or availability consequences.

**Finding 6: Active-account and low-churn characterization — [partial]**

Claim (`doc1_seller_listing_description.md:9,13-14`): FlowMetrics has “120 active accounts” and “low” churn with a “stable base of recurring subscribers.”

Evidence: `doc6_crm_export_summary.md:3-18` — all 120 accounts are marked active, but 23 are identified as “Phantom/Inactive Accounts”; those accounts have no payment events in the last 90 days and last logged in 60-120 days ago. `doc5_stripe_export_summary.md:3-5` records 94 active Stripe subscriptions, while `doc6_crm_export_summary.md:6-11` records three wire-transfer enterprise accounts.

Delta: The CRM status supports a count of 120 records marked active, but the activity evidence does not support treating all 120 as economically active customers. Twenty-three accounts have stale payment and login indicators. The source material does not provide a churn definition or cohort calculation, so I cannot convert this directly into a churn percentage; the “low churn” characterization is only partially supported.

**Finding 7: Revenue reconciliation — [derived]**

Basis: `doc5_stripe_export_summary.md:5` — “Total MRR (Stripe): $16,000.”

`doc5_stripe_export_summary.md:19` — “The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer.”

Derivation: $16,000 Stripe MRR + (3 × $8,000 wire-transfer MRR) = $16,000 + $24,000 = $40,000.

Consequence: The supplied revenue components reconcile to the seller’s stated $40,000 blended MRR (`doc1_seller_listing_description.md:12`). This arithmetic does not create a revenue delta. It does, however, confirm that the three wire-transfer contracts must be included when interpreting the headline MRR.

Escalates: None.

## Confirmations and non-deltas

- The Rails 7 claim is supported by `doc2_tech_stack_description_as_provided_by_se.md:3`, which identifies the core application as Rails 7.
- Managed SSL is recorded in `doc4_infrastructure_config.md:25-26` as Heroku-managed and auto-renewed.
- Stripe payment processing is described as secure in `doc2_tech_stack_description_as_provided_by_se.md:5`; this review did not perform a security test of Stripe integration.
- The three enterprise contracts are each recorded at $8,000/month (`doc7_customer_contracts_summary.md:5-23`), matching the revenue reconciliation.
- The recurring-revenue model is supported by 94 month-to-month subscriptions and three annual enterprise contracts (`doc7_customer_contracts_summary.md:29, 3-23`).

## Coverage and remaining claims

The nine documents were identified and the claim surface was estimated at approximately 68 micro-claims. I individually checked the load-bearing categories first: safety and recoverability, architecture, operational controls, selected security/dependency claims, revenue, and customer-account activity. The findings above cover the material contradictions and qualifications found in those categories.

I did not individually verify every low-impact micro-claim, including all product-feature descriptions, every cost figure, every stated maintenance characteristic, and the complete Rails, DNS, OAuth, and Stripe implementation surface. I also did not independently measure uptime, execute backups, inspect production traffic, test failover, validate customer payment records, or perform a penetration test. Those limits matter because the records establish documentary contradictions and exposure, not the full live-system behavior.

Specific remaining verification items include:

1. Obtain backup-provider execution logs, retention settings, restore tests, and failure-alert history.
2. Obtain uptime-monitor history and the measurement definition behind 99.9%.
3. Run or review payment-path, integration, and critical-path tests.
4. Confirm Heroku dyno/process behavior and database recovery or redundancy arrangements.
5. Obtain DataEnrich.io terms, portability details, API limits, and a documented fallback design.
6. Reconcile the 23 stale CRM accounts against invoices, renewals, cancellations, and customer communications.
7. Obtain signed terms or recorded click-through acceptance for the 15 no-contract accounts (`doc7_customer_contracts_summary.md:42-45`).
8. Confirm ownership and transferability of the GoDaddy domain, Google OAuth configuration, Heroku account, secrets, and vendor accounts.

The recommendation is **Material** because the backup records, absent uptime monitoring, absent payment-path testing, and single-instance architecture materially change the reliability picture presented by the seller. The buyer should decide whether those gaps are acceptable and how to price them; this report does not prescribe that transaction decision.

=== GAP MAP ===

# FlowMetrics — Claims Verification

**Recommendation: Material**

- **Backup protection:** The claimed daily backups with 30-day retention are contradicted by 21 days of recorded failures, a last successful backup on 2026-07-30, and no failure alerting.
- **Reliability evidence:** The claimed 99.9% uptime monitoring has no corresponding uptime monitor; only Heroku’s status page is recorded.
- **Architecture and testing:** Heroku is present, but the deployment records one dyno, a same-instance database, no replicas, and no separate database instance; the test inventory has zero integration and payment-path tests.
- **Customer quality and dependencies:** 23 of 120 CRM accounts have no payment events for 90 days and stale logins; DataEnrich.io supports 40% of features with no fallback, while DNS has no secondary provider.
- **What holds:** The $40,000 MRR reconciles from $16,000 Stripe MRR plus three $8,000 wire-transfer contracts; Rails 7, managed SSL, and the recurring contract structure are supported by the reviewed records.

**Coverage:** Approximately 68 claims identified across nine documents; load-bearing categories and selected commercial, security, dependency, and operational claims were checked. Lower-impact micro-claims and live-system validation remain outside this review.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice