# FlowMetrics technical claims audit

**Recommendation: Material — 33 claims identified; 14 claims checked individually, including the highest-priority recoverability, payment, customer-status, dependency, architecture, operational, and testing claims.** The checked claims contain material gaps in backup recoverability, uptime monitoring, customer activity, vendor lock-in, and critical-path testing. This conclusion applies only to the claims examined in this limited-assurance review.

**As-of date for materials:** 2026-08-25.

## Findings

**Finding 1: Backup failures threaten recoverability — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6,10): “We monitor uptime at 99.9% and maintain daily automated database backups.” “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:16-19 — “**Schedule:** Daily at 2:00 AM via `heroku pg:backups schedule`”; “**Status:** Failures recorded for the last 21 days”; “**Last Successful Backup:** 2026-07-30”; “**Alerting:** None configured for backup failures.”

Gap: Daily scheduling exists, but the materials show 21 consecutive days of recorded failures, no failure alerting, and no successful backup after 2026-07-30. The evidence does not establish current recoverability.

**Finding 2: Backup-retention expiry — [derived]**

Basis: doc4_infrastructure_config.md:18 — “**Last Successful Backup:** 2026-07-30”
       doc9_seller_s_technical_claims_verbatim.md:10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Derivation: 2026-07-30 + 30 days = 2026-08-29. On 2026-08-29, the last successful backup identified in the materials reaches the stated 30-day retention limit, unless a newer successful backup exists but was not supplied.

Consequence: After that date, the materials do not show a recoverable backup within the seller's stated retention window. This escalates Finding 1.

Escalates: Finding 1.

**Finding 3: Claimed 99.9% uptime monitoring is absent — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6): “We monitor uptime at 99.9% and maintain daily automated database backups.”

Evidence: doc4_infrastructure_config.md:21-23 — “**Uptime Monitor:** None (No Pingdom, UptimeRobot, or custom checks).” “**Status Page:** Heroku built-in status page only.”

Gap: The evidence shows no external or custom uptime monitor. A Heroku status page is not evidence that FlowMetrics monitors uptime at 99.9%.

**Finding 4: Active-customer count includes phantom/inactive accounts — [partial]**

Claim (doc1_seller_listing_description.md:9,13): “We have built a loyal customer base of 120 active accounts”; “**Active Customers:** 120.”

Evidence: doc6_crm_export_summary.md:3,12-16 — “**Total Accounts Marked 'Active':** 120”; “**Phantom/Inactive Accounts:** 23”; “No payment events in the last 90 days”; “Last login dates: 60-120 days ago.”

Gap: The CRM label matches 120, but 23 accounts are expressly identified as phantom/inactive. The materials do not support 120 genuinely active paying accounts.

**Finding 5: $40,000 MRR is not independently reconciled — [partial]**

Claim (doc1_seller_listing_description.md:9,12): “Our blended Monthly Recurring Revenue (MRR) stands at $40,000”; “**Blended MRR:** $40,000.”

Evidence: doc5_stripe_export_summary.md:4-5,17-19 — “**Total Active Subscriptions:** 94”; “**Total MRR (Stripe):** $16,000”; “The 3 enterprise companies ... have separate contracts for $8,000/mo each, paid via wire transfer.” Evidence: doc7_customer_contracts_summary.md:3-7,13-16,21-24 — three enterprise contracts are each valued at $8,000/mo.

Gap: The three wire contracts mathematically account for $24,000/mo, and together with $16,000 Stripe MRR reconcile to $40,000. However, the materials supplied are summaries rather than payment receipts or bank evidence, and 23 CRM accounts are phantom/inactive. The amount is arithmetically explained but not fully substantiated as collected recurring revenue.

**Finding 6: No-vendor-lock-in claim conflicts with dependency evidence — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:9): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supplies product-data enrichment for “40% of features”; “**Fallback:** None implemented”; if unavailable, “product degrades to basic mode.” Evidence: doc4_infrastructure_config.md:27-29 — DNS is managed personally by Dave, with “No secondary DNS provider.”

Gap: A material product surface depends on one provider with no fallback, and DNS has a single personal control point. That is operational dependency and lock-in risk inconsistent with an unqualified “no lock-in” claim.

**Finding 7: Critical-path testing claim is false on the supplied test inventory — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:7): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: doc3_git_history_summary.md:24-32 — “**CI/CD Pipeline:** None configured”; “Unit Tests: 12 (all located in `test/utils/`)”; “Integration Tests: 0”; “Payment-Path Tests: 0”; “**Staging Environment:** None.”

Gap: The supplied inventory shows no integration or payment-path tests and only 12 utility tests, so it does not support comprehensive critical-path coverage. The “well-documented” component is also not established by this evidence.

**Finding 8: Horizontal-scaling claim is only partly evidenced — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:8): “The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard.”

Evidence: doc4_infrastructure_config.md:3-8 — Heroku hosts one `standard-1x` dyno; PostgreSQL runs on the same dyno; “**Read Replicas:** None”; “**Separate DB Instance:** No.”

Gap: Heroku hosting and a dyno configuration are shown, but the materials do not establish horizontal scaling, multiple dynos, or the one-click operation claim. The database arrangement is a separate capacity and availability constraint.

**Finding 9: Third-party enrichment exists but has a material operational caveat — [partial]**

Claim (doc2_tech_stack_description_as_provided_by_se.md:3-5): the product integrates with a third-party data-enrichment API providing real-time product data.

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io provides “Product data enrichment API”; “**Dependency:** 40% of features depend on this API”; “**Fallback:** None implemented”; outage or pricing change causes basic mode “(no enrichment).”

Gap: The integration is evidenced, but 40% of features depend on it and there is no fallback. The claim holds only with that material limitation.

**Finding 10: Stripe payment use is evidenced, but the security and coverage wording is broader than the evidence — [real, operational caveat]**

Claim (doc2_tech_stack_description_as_provided_by_se.md:5): “Payment processing is securely handled through Stripe, which supports both credit card transactions and recurring billing.”

Evidence: doc5_stripe_export_summary.md:4-5,12-19 shows 94 active subscriptions and $16,000 Stripe MRR, including an “Enterprise Plan (Stripe Subscription).” doc8_external_dependency_list.md:3-6 identifies Stripe's function as “Payment processing.”

Gap: Stripe use and recurring subscriptions are evidenced. The materials do not independently show security controls or credit-card transaction detail, and the three $8,000/mo enterprise contracts are paid by wire transfer rather than Stripe.

**Finding 11: SSL is evidenced; DNS has a single personal control point — [real, operational caveat]**

Claim (doc9_seller_s_technical_claims_verbatim.md:11): “We use a modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4_infrastructure_config.md:25-29 — “**SSL:** Heroku-managed, auto-renewed”; “**DNS Provider:** GoDaddy”; “**DNS Management:** Managed personally by 'dave'. No secondary DNS provider.”

Gap: Managed SSL is evidenced. DNS is managed through one provider and personally by Dave, with no secondary provider; the cited evidence does not independently verify the “modern Rails 7” portion.

## Remaining claims and coverage

The frozen claim surface was 33 claims: doc1 = 18, doc2 = 8, doc9 = 7. Fourteen were checked individually. The remaining 19 were not individually tested because the priority order reached lower-impact business-description, pricing, product-market-fit, operational-overhead, and documentation assertions after material gaps had already been established. They remain unverified, not disproved. The review therefore does not support a conclusion about all 33 claims.

The low-churn claim was not assigned a delta: the evidence shows 23 phantom/inactive accounts, but does not state a churn rate, cancellation count, or cohort-retention measure. This is **unverifiable** from the supplied materials. Absence checks for uptime monitoring and critical-path tests used both lexical searches for the claim terms (“uptime monitor”/“monitoring”; “integration”, “payment-path”, and “critical path”) and structural review of the monitoring/infrastructure and CI/testing sections of docs 3 and 4.

## Questions for the seller before closing

1. Provide successful backup logs after 2026-07-30, restore-test evidence, and the actual retention policy.
2. Explain the 21 days of backup failures and why no failure alerting is configured.
3. Provide uptime-monitoring history supporting 99.9%, including measurement method and exclusions.
4. Reconcile the $40,000 MRR to bank receipts, invoices, Stripe records, and the three enterprise contracts.
5. Identify which of the 120 CRM accounts are paying and active, and provide churn and cohort-retention data.
6. Provide DataEnrich.io agreement terms, pricing-change rights, portability, and a fallback plan.
7. Demonstrate dyno scaling and database separation/failover, not merely Heroku's platform capability.
8. Provide the test inventory for billing, integrations, authentication, backup/restore, and other critical paths.
9. Document DNS ownership, transfer procedure, recovery access, and any secondary DNS arrangement.

## Proposed technique-only METHOD.md edits

- For claims involving availability or recoverability, require a current-success check and a restore test, not merely a configured schedule or platform feature.
- When a seller supplies a retention period and a last-success date, automatically derive and date the retention expiry, and link it to the corresponding ordinary finding.
- For customer and revenue claims, reconcile independent system labels against payment activity and contracts; do not treat a CRM status as proof of active customer status.
- For absence claims, record both a lexical search using the seller's terminology and a structural review of the expected subsystem or document section.

=== LIMITATIONS ===
Materials examined: nine data-room documents (doc1 through doc9); conclusions are as of 2026-08-25.
The seller was not consulted and has not confirmed my reading of their own claims.
This is a limited-assurance engagement; the conclusion rests on 14 of 33 claims checked individually, prioritised toward material continuity, payment, dependency, architecture, operational, and testing claims.
