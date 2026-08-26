# FlowMetrics technical claims audit

**Recommendation: Material — 86 claims identified; 18 checked individually.** The checked population contains material gaps in recoverability, availability monitoring, failover, testing, revenue support, dependency resilience, and control of DNS. This conclusion applies only to the claims examined.

=== CLAIM SURFACE ===
86 claims

## Findings

**Finding 1: Database backup failure and recoverability — [delta]**

Claim (doc1_seller_listing_description.md:19): “99.9% uptime monitoring and daily automated database backups with 30-day retention.”

Evidence: doc4_infrastructure_config.md:15-19 — backups are scheduled daily, but failures were recorded for the last 21 days; the last successful backup was 2026-07-30, and no alerting is configured.

Gap: Daily scheduling exists, but the asserted operational backup capability is not functioning and failures are not alerted. The evidence does not independently establish the claimed 30-day retention.

**Finding 2: Claimed redundancy and automatic failover — [delta]**

Claim (doc1_seller_listing_description.md:18): “Scalable Architecture: Built on a modern Rails 7 stack with platform-level redundancy and automatic failover.”

Evidence: doc4_infrastructure_config.md:6-8 — PostgreSQL runs on the same dyno as the application; there are no read replicas and no separate database instance.

Gap: The materials show no separate database capacity or database failover path. Heroku hosting is present, but the claimed system-level redundancy and automatic failover are not demonstrated for the database.

**Finding 3: Uptime monitoring — [delta]**

Claim (doc1_seller_listing_description.md:19): “99.9% uptime monitoring.”

Evidence: doc4_infrastructure_config.md:21-23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)” and only the Heroku built-in status page is identified.

Gap: No application uptime monitor is configured, so the claimed monitoring is false on the evidence examined.

**Finding 4: Comprehensive critical-path test coverage — [delta]**

Claim (doc1_seller_listing_description.md:21): “Well-documented codebase with comprehensive test coverage across critical paths.”

Evidence: doc3_git_history_summary.md:24-32 — no CI/CD pipeline; 12 unit tests all in `test/utils/`; zero integration tests and zero payment-path tests; no staging environment, branch protection, or documented code-review process.

Gap: The evidence does not support comprehensive critical-path coverage, particularly for integration and payment paths.

**Finding 5: Revenue and active-customer support — [partial]**

Claim (doc1_seller_listing_description.md:12-13): “Blended MRR: $40,000” and “Active Customers: 120.”

Evidence: doc5_stripe_export_summary.md:4-5 — 94 active subscriptions and $16,000 total Stripe MRR. doc5_stripe_export_summary.md:19 — three enterprise contracts are stated at $8,000/month each and excluded from Stripe MRR. doc6_crm_export_summary.md:3,12-18 — all 120 accounts are marked active, but 23 are categorized “Phantom/Inactive,” had no payment events in the last 90 days, and last logged in 60–120 days ago. doc7_customer_contracts_summary.md:29-40 — 94 month-to-month subscriptions and eight free pilots; the pilots have no payment obligation.

Gap: The three wire contracts mathematically explain $24,000 of the $40,000 MRR, and the Stripe amount explains $16,000, but the materials do not verify collection status for the wire contracts. The 120-account count includes 23 accounts characterized as inactive and eight free pilots; it is not equivalent to 120 paying active customers.

**Finding 6: Third-party dependency resilience and lock-in — [partial]**

Claim (doc1_seller_listing_description.md:22): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supplies product-data enrichment, 40% of features depend on it, termination requires 90 days’ notice, and no fallback is implemented; outage or pricing change degrades the product to basic mode.

Gap: The materials show a material operational dependency and no fallback. They do not establish that the relevant agreement has no lock-in or otherwise protects continuity.

**Finding 7: DNS control concentration — [partial]**

Claim (doc2_tech_stack_description_as_provided_by_se.md:5): “The entire stack is managed through Heroku’s dashboard, ensuring that deployments, scaling, and monitoring are streamlined and efficient.”

Evidence: doc8_external_dependency_list.md:29-32 — GoDaddy provides DNS; DNS is managed personally by “dave,” with no secondary DNS, and expiry or a GoDaddy issue would make the app unreachable.

Gap: The stated Heroku-centered management description omits a single-person DNS control point outside Heroku. This creates an availability and transition risk.

## Coverage and remaining claims

I checked 18 of 86 claims, prioritising recoverability, payment and revenue support, dependency withdrawal, architectural invariants, operational monitoring, and critical-path testing. I stopped after the remaining unchecked claims were lower-priority descriptive or low-impact claims, while the checked claims had already established material gaps. The remaining 68 claims were not individually verified; this audit therefore does not support a conclusion about them.

The materials also show no separate evidence of a meaningful external telemetry path beyond the listed service integrations and operational controls; I have not treated that observation as a broader security conclusion.

## Questions for the seller before closing

1. What caused the backup failures since 2026-07-30, and can you provide successful restore-test evidence and the actual retention policy?
2. What application-level uptime monitor produced the claimed 99.9% figure, and can you provide its historical logs?
3. What component fails over automatically, and what is the tested recovery procedure for the database and the shared dyno?
4. Can you provide bank or ledger evidence that each $8,000/month enterprise contract has been collected and remains current?
5. Which of the 120 CRM accounts are paying customers after excluding phantom accounts and free pilots?
6. What contractual portability, price protection, and transition rights exist for DataEnrich.io, and where is the fallback implementation?
7. Who controls the GoDaddy account and DNS credentials, and how would control transfer to the buyer without an availability interruption?

Technique-only proposed METHOD.md edits for review: require exact source retrieval for every material finding before drafting; treat “scheduled” versus “successful” backup operation as separate claims; distinguish account-status fields from payment evidence when testing customer and MRR claims; and require dependency findings to test both contractual portability and implemented fallback behavior.

=== LIMITATIONS ===
Materials examined: nine data-room documents (docs 1–9), as available on 2026-08-25; the report is as of that materials snapshot.
The seller was not consulted and has not confirmed your reading of their own claims.
This is limited assurance based on 18 individually checked claims out of 86 identified; the remaining 68 claims were not individually verified.
