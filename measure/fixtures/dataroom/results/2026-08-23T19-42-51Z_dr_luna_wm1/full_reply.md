# FlowMetrics Claims-Verification Report

## Recommendation: Material

The data room contains material deltas in reliability, testing, dependency resilience, and customer-account reporting. These gaps change the risk profile presented in the listing. I did not treat incomplete security evidence as proof of a security failure, but the available materials do not substantiate several broader security and operational assertions.

## Scope and coverage

I reviewed all nine data-room documents and identified roughly 30–40 micro-claims across product description, revenue, customer status, architecture, reliability, testing, dependencies, security, and operations. I prioritised safety and continuity mechanisms, architectural invariants, operational controls, and then commercial and micro-claims.

The findings below cover the load-bearing claims and the commercial claims most relevant to valuation. I did not perform a penetration test, source-code review, access-control test, or independent uptime measurement. I also could not verify undocumented deployment ownership, incident response, access provisioning, encryption beyond managed SSL, or the implementation details of Google OAuth. Those remain unresolved claims rather than findings that the claims are false.

## Findings

**Finding 1: Backup continuity and retention — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6, 10): “We monitor uptime at 99.9% and maintain daily automated database backups.” and “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:15-19 — “Schedule: Daily at 2:00 AM via `heroku pg:backups schedule`”; “Status: Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.”

Delta: The schedule exists, but the recorded backup process has failed for 21 days, with no failure alerting. The data room therefore does not support the claim that daily automated backups are currently being maintained.

**Finding 2: Derived recovery-window exposure — [derived]**

Basis: doc4_infrastructure_config.md:18 — “Last Successful Backup: 2026-07-30”

doc9_seller_s_technical_claims_verbatim.md:10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Derivation: 2026-07-30 + 30 days = 2026-08-29. If the stated retention period is measured from the last successful backup, the stated 30-day window expires on 2026-08-29 while the subsequent 21 days have no successful backup recorded.

Consequence: The seller’s own stated retention figure, combined with the recorded last-success date, entails a finite date after which the stated recovery window is exhausted unless another successful backup exists that is not shown in the data room.

Escalates: Finding 1.

**Finding 3: Uptime monitoring — [delta]**

Claim (doc1_seller_listing_description.md:19; doc9_seller_s_technical_claims_verbatim.md:6): “99.9% uptime monitoring” and “We monitor uptime at 99.9%”.

Evidence: doc4_infrastructure_config.md:21-23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).” and “Status Page: Heroku built-in status page only.”

Delta: No application uptime monitor is configured. The Heroku status page is not evidence that FlowMetrics monitors its own 99.9% uptime claim.

**Finding 4: Comprehensive critical-path testing — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:7): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: doc3_git_history_summary.md:24-31 — “CI/CD Pipeline: None configured”; “Unit Tests: 12 (all located in `test/utils/`)”; “Integration Tests: 0”; “Payment-Path Tests: 0”; “Staging Environment: None”; “Branch Protection: None.”

Delta: The recorded test inventory does not support comprehensive critical-path coverage. In particular, there are no integration or payment-path tests, and no staging or CI/CD controls are recorded.

**Finding 5: Platform redundancy and failover — [partial]**

Claim (doc1_seller_listing_description.md:18; doc9_seller_s_technical_claims_verbatim.md:5): “Built on a modern Rails 7 stack with platform-level redundancy and automatic failover” and “The system has redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4_infrastructure_config.md:4-8 — Heroku standard-1x hosting; one `heroku-postgresql:standard-0` database; “Read Replicas: None”; “Separate DB Instance: No.”

Delta: Heroku may provide platform process management, but the data room shows no read replicas and no separate database instance. The claim is therefore only partially supported: application-process failover does not establish database redundancy or continuity across the stated persistence layer.

**Finding 6: No vendor lock-in — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:9; doc1_seller_listing_description.md:22): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supplies product matching, price comparison, and category classification; “Dependency: 40% of features depend on this API”; “Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment).”

Delta: Standard commercial agreements do not eliminate technical dependency. Forty percent of the stated feature set depends on DataEnrich.io, with no implemented fallback.

**Finding 7: Active-customer count and churn description — [delta]**

Claim (doc1_seller_listing_description.md:9, 13-14): “a loyal customer base of 120 active accounts” and “Churn: Low (stable base of recurring subscribers)”.

Evidence: doc6_crm_export_summary.md:3-18 — 120 accounts are marked active, but 23 are identified as “Phantom/Inactive Accounts”; those accounts have “No payment events in the last 90 days” and “Last login dates: 60-120 days ago”; all 120 remain marked active.

Delta: The CRM status field overstates active accounts by the data room’s own classification. The evidence does not establish that the 23 accounts are paying or engaged, so the 120-account and low-churn descriptions are not supported as stated.

**Finding 8: DNS and secrets operational posture — [real, operational caveat]**

Claim (doc1_seller_listing_description.md:20): “Security: Managed SSL, OAuth authentication, and secure payment processing via Stripe.”

Evidence: doc4_infrastructure_config.md:26-29 — “SSL: Heroku-managed, auto-renewed”; “DNS Management: Managed personally by 'dave'. No secondary DNS provider”; “14 secrets stored in Heroku config vars ... No secrets vault.”

Delta: Managed SSL and Stripe processing are supported by the evidence available. The operational caveat is that DNS has a single personal administrator and no secondary provider, and 14 secrets are stored in application configuration variables rather than a secrets vault. This does not establish a breach, but it is material transition and continuity context for a buyer.

**Finding 9: Revenue reconciliation and customer-contract structure — [real, operational caveat]**

Claim (doc1_seller_listing_description.md:12-15): “Blended MRR: $40,000”, “Active Customers: 120”, and recurring subscription revenue from monthly and annual enterprise customers.

Evidence: doc5_stripe_export_summary.md:4-5, 17-19 — 94 active subscriptions and $16,000 Stripe MRR; three enterprise companies have separate $8,000/month wire-transfer contracts, excluded from Stripe. doc7_customer_contracts_summary.md:3-27 — three enterprise contracts at $8,000/month each, with stated SLAs of 99.9%, 99.9%, and 99.5%.

Delta: None on the arithmetic presented: $16,000 Stripe MRR plus $24,000 in separately stated enterprise wire payments equals $40,000. The operational caveat is that three enterprise contracts carry SLA remedies and termination rights, while the 94 month-to-month subscriptions have no SLA and eight pilot agreements are free 90-day trials expiring in Q3 2026 (doc7:29-38). The buyer should verify that the $40,000 is current, collectible, and contractually transferable.

## Remaining questions for the seller before closing

1. Provide successful backup records after 2026-07-30, restore-test evidence, and an explanation for the 21-day failure period.
2. Confirm whether the stated 30-day retention is calendar retention, and identify the storage location and owner of backup credentials.
3. Provide uptime-monitoring history and the calculation supporting the 99.9% figure.
4. Provide the full test inventory, CI/CD configuration, deployment history, and payment-path test evidence.
5. Identify the fallback or replacement plan for DataEnrich.io and provide its termination and export terms.
6. Reconcile the 23 phantom/inactive CRM records, active-customer count, churn calculation, and pilot conversion status.
7. Provide executed enterprise contracts, SLA measurement records, renewal status, and consent or assignment requirements.
8. Transfer DNS, Heroku, Stripe, Google OAuth, Twilio, and DataEnrich.io administration away from personal accounts before closing.
9. Provide evidence of authentication, authorization, encryption, secrets rotation, incident response, and access review. I have not inferred failure from their absence in this data room.

This is a claims-verification report, not a penetration test, code-quality review, legal opinion, or acceptance test.

=== GAP MAP ===

**FlowMetrics — B2B e-commerce analytics SaaS**

**Recommendation: Material**

- **Backups and recovery:** Daily backups are claimed, but the infrastructure record shows 21 days of failures, no failure alerting, and a last successful backup dated 2026-07-30.
- **Testing controls:** Comprehensive critical-path coverage is claimed; the recorded repository has 12 utility-only unit tests, no integration or payment-path tests, and no CI/CD or staging environment.
- **Dependency resilience:** No vendor lock-in is claimed; DataEnrich.io supports 40% of features and has no implemented fallback.
- **Customer base:** 120 active accounts are claimed; 23 CRM accounts are labelled phantom/inactive, with no payment events in 90 days and old last-login dates.
- **Continuity posture:** Heroku-managed SSL and Stripe processing are supported, but DNS is personally managed by Dave, there is no secondary DNS provider, and 14 secrets are held in config vars.

**Coverage:** Roughly 30–40 micro-claims were identified across nine documents. Load-bearing reliability, architecture, testing, dependency, customer, revenue, and selected security/operations claims were checked. Undocumented security controls, access ownership, independent uptime, incident response, and source-code implementation remain unresolved.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice