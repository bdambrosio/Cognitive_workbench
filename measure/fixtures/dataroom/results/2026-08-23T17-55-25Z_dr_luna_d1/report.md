# FlowMetrics AI-Readiness / Claims-Verification Report

## Recommendation: Material

The data room contains nine documents and approximately 195 identifiable micro-claims. The most consequential claims about reliability, redundancy, scaling, monitoring, backups, DNS, testing, and operating burden are not supported by the observed implementation. Several security and recovery claims remain unverifiable rather than disproven. The available evidence is sufficient for a Material recommendation, but not for a conclusion that every seller claim is false.

## Findings

**Finding 1: Backup continuity and uptime monitoring — [delta]**

Claim (doc1_seller_listing_description.md:19; doc9_seller_s_technical_claims_verbatim.md:6): “99.9% uptime monitoring and daily automated database backups with 30-day retention” and “We monitor uptime at 99.9% and maintain daily automated database backups.”

Evidence: doc4_infrastructure_config.md:15-19 records daily scheduling, failures for the last 21 days, last successful backup on 2026-07-30, and no backup-failure alerting. doc4:21-23 states that no uptime monitor is configured.

Delta: The claimed uptime monitoring is absent; backups were failing for 21 days without alerting.

**Finding 2: Derived backup-retention exhaustion — [derived]**

Basis: doc4_infrastructure_config.md:18 — “**Last Successful Backup:** 2026-07-30”

doc1_seller_listing_description.md:19 — “daily automated database backups with 30-day retention.”

doc9_seller_s_technical_claims_verbatim.md:10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Derivation: 2026-07-30 + 30 days = 2026-08-29. If no newer successful backup exists, the stated 30-day retention window is exhausted on 2026-08-29.

Consequence: The seller’s stated recovery-retention protection would no longer cover the current state after that date absent a newer successful backup.

Escalates: Finding 1.

**Finding 3: Database redundancy and automatic failover — [delta]**

Claim (doc1_seller_listing_description.md:18; doc9_seller_s_technical_claims_verbatim.md:5): The system has “platform-level redundancy and automatic failover.”

Evidence: doc4_infrastructure_config.md:3-8 shows one `standard-1x` dyno, the PostgreSQL database running on the same dyno as the application, no read replicas, and no separate database instance.

Delta: The documented database topology does not provide the claimed database redundancy. Application-level failover is not established by the available materials.

**Finding 4: Comprehensive critical-path testing — [delta]**

Claim (seller technical claims / reliability representations): The system is tested across its critical paths.

Evidence: doc3_git_history_summary.md:24-29 records no CI/CD pipeline, 12 unit tests all in `test/utils/`, zero integration tests, and zero payment-path tests.

Delta: The available test record does not match comprehensive critical-path coverage.

**Finding 5: Automatic horizontal scaling and availability under varying load — [delta]**

Claim (doc2_tech_stack_description_as_provided_by_se.md:3; doc9:8): Heroku provides automatic scaling and failover, the application remains available under varying load, and the stack is designed to scale horizontally.

Evidence: doc4_infrastructure_config.md:3-8 shows one standard-1x dyno, the database on that same dyno, no replicas, and no separate database instance.

Delta: The present deployment does not demonstrate automatic horizontal scaling or availability under varying load. The fact that adding dynos is operationally possible is not evidence that automatic scaling is configured.

**Finding 6: Managed DNS — [delta]**

Claim (doc1_seller_listing_description.md:20; doc9:11): The business uses “managed DNS.”

Evidence: doc4_infrastructure_config.md:25-29 identifies GoDaddy as the DNS provider, says DNS is managed personally by Dave, and records no secondary DNS provider.

Delta: The implementation is personally managed single-provider DNS, not the represented managed/automated or redundant arrangement.

**Finding 7: Blended MRR provenance — [real, operational caveat]**

Claim (doc1_seller_listing_description.md:9, 12): Blended MRR is $40,000.

Evidence: doc5_stripe_export_summary.md:5 states “**Total MRR (Stripe):** $16,000.” doc5:19 states that Acme Retail, GlobalMart, and ShopStream have separate contracts for $8,000/mo each, paid by wire and excluded from Stripe. The three contracts are also listed in doc7_customer_contracts_summary.md:3-22.

Delta: None established. The $24,000 difference is arithmetically reconciled by three $8,000/month wire-paid contracts. The caveat is provenance: the buyer should independently verify receipt, enforceability, renewal, and inclusion in the stated blended MRR.

**Finding 8: Low vendor lock-in — [partial]**

Claim (doc1_seller_listing_description.md:22; doc9:9): All third-party integrations use standard SaaS agreements with no proprietary lock-in.

Evidence: doc8_external_dependency_list.md:7-12 states that 40% of features depend on DataEnrich.io, termination requires 90 days’ notice, and no fallback is implemented; if the API is down or pricing changes, the product degrades to basic mode. doc8:28 records that Google OAuth has no contractual relationship.

Delta: The no-lock-in claim is only partially supported. A material feature dependency has no fallback, and one dependency has no contractual relationship.

**Finding 9: 120 active accounts — [partial]**

Claim (doc1_seller_listing_description.md:9, 13): The business has 120 active accounts/customers.

Evidence: doc6_crm_export_summary.md:3-18 marks 120 accounts active but identifies 23 phantom/inactive accounts with no payment events in the last 90 days and last login dates 60-120 days ago. doc7_customer_contracts_summary.md:37-45 identifies eight free pilots expiring in Q3 2026 and 15 accounts with no signed ToS or recorded acceptance.

Delta: The evidence supports 120 CRM records marked active, not 120 verified active paying or contracted customers.

**Finding 10: Low churn — [partial]**

Claim (doc1_seller_listing_description.md:14): “Churn: Low (stable base of recurring subscribers).”

Evidence: doc5_stripe_export_summary.md:3-5 reports the last 12 months and 94 active subscriptions. doc6:12-16 records 23 phantom/inactive accounts, no payment events in the last 90 days, and last logins 60-120 days ago. No explicit cancellation or churn history is provided.

Delta: The records are consistent with some stable recurring revenue but do not establish a low churn rate. The 23 inactive/phantom records qualify the claim.

**Finding 11: Minimal operational overhead and routine no-manual-intervention operation — [delta]**

Claim (doc1_seller_listing_description.md:9): The business operates with “minimal operational overhead.” doc2:3 says routine operations require no manual intervention and doc2:5 says deployments, scaling, and monitoring are streamlined through Heroku’s dashboard.

Evidence: doc4:17-19 records 21 days of failed backups with no alerting. doc3:24-32 records no CI/CD pipeline, no staging environment, no branch protection, and no documented code-review process.

Delta: The documented operating controls require material manual oversight and lack basic automation and review infrastructure; this does not match the broad minimal-overhead/no-manual-intervention representation.

**Finding 12: Secure payment processing — [partial]**

Claim (doc1_seller_listing_description.md:20; doc2:5): Payment processing is securely handled through Stripe.

Evidence: doc5:4-5 records 94 active subscriptions and $16,000 Stripe MRR; doc8:3-5 identifies Stripe as the payment processor. doc5:19 states the three principal enterprise contracts are paid by wire rather than Stripe.

Delta: Stripe processing and recurring subscriptions are evidenced, but the available materials do not independently establish the security of the implementation. The claim also does not cover the separately paid wire contracts.

## Remaining Claims / Coverage Limits

The following were not verified and are not classified as deltas because the available materials do not settle them: authorization and access-control behavior; encryption at rest; secret rotation; rollback capability; recovery procedure and recovery testing; the exact Rails 7 version in the deployed system; the asserted quality/documentation of the codebase; and an exact churn rate. SSL is supported with an operational caveat: doc4:26 says Heroku-managed and auto-renewed, but certificate enforcement and monitoring were not independently tested. OAuth authentication is mentioned, but provider flow and configuration evidence are limited.

Three requested documents were absent from the repository. Their names are not recorded, so I cannot identify them without inventing information. This limits verification of the unresolved security, recovery, legal, and operational claims. The audit is claims verification, not a penetration test, code-quality review, or legal review.

## Questions for the Seller Before Closing

1. Provide backup-success logs after 2026-07-30, retention evidence, restore tests, and the incident explanation for the 21-day failure period.
2. Identify the uptime-monitoring service, alert recipients, historical uptime report, and escalation procedure.
3. Provide the actual Heroku dyno/database topology, scaling policy, failover behavior, and recovery-time and recovery-point objectives.
4. Provide CI/CD, staging, branch-protection, code-review, rollback, and production-change records.
5. Provide authorization rules, encryption-at-rest configuration, secret-rotation history, and evidence that no credentials were exposed.
6. Provide DNS account ownership, transferability, renewal controls, and a secondary-provider or recovery plan.
7. Reconcile the $40,000 MRR to bank statements, invoices, wire receipts, contract renewals, and cancellation history.
8. Define “active account” and provide payment, login, contract, and renewal status for all 120 records.
9. Provide DataEnrich terms, exit/transition plan, feature-by-feature dependency mapping, and any OAuth contractual or transfer documentation.
10. Identify the three absent requested documents and explain why they were not included.
