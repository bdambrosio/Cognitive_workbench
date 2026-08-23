# FlowMetrics Claims-Verification Report

## Recommendation: Material

The supplied records contain several material deltas against the seller’s claims: backup execution has failed for 21 days; the claimed 99.9% uptime monitoring does not exist; the redundancy claim conflicts with the documented single-dyno/single-database arrangement; comprehensive critical-path testing is contradicted by the testing record; and the no-lock-in claim conflicts with documented dependency and ownership exposure.

The customer and revenue records also contain reconciliation problems. I did not treat the absence of source code or infrastructure artifacts as proof that those mechanisms do not exist. The findings below are based on the nine seller-provided Markdown summaries in the data room.

## Findings

**Finding 1: Backup service is failing — [delta]**

Claim (doc1_seller_listing_description.md:19; doc9_seller_s_technical_claims_verbatim.md:6, 10): “99.9% uptime monitoring and daily automated database backups with 30-day retention”; “We monitor uptime at 99.9% and maintain daily automated database backups”; “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:15-19 — the schedule is daily at 2:00 AM, but “Failures recorded for the last 21 days,” the last successful backup was 2026-07-30, and no alerting is configured for backup failures.

Delta: A configured schedule is not producing successful backups, and failures are not alerted. The stated daily-backup claim does not hold in the supplied operational record.

**Finding 2: Stated retention window is materially eroding — [derived]**

Basis: doc9_seller_s_technical_claims_verbatim.md:10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”

doc4_infrastructure_config.md:17-18 — “Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30.”

Derivation: 30 stated retention days − 21 stated failure days = 9 days of the stated retention interval remaining, assuming the failure period is continuous and the last successful backup is the relevant retained recovery point.

Consequence: The seller’s stated 30-day recovery window is not presently backed by a 30-day sequence of successful backups; the available recovery point is 21 days old according to the supplied record.

Escalates: Finding 1.

**Finding 3: Uptime monitoring claim is false in the supplied record — [delta]**

Claim (doc1_seller_listing_description.md:19; doc9_seller_s_technical_claims_verbatim.md:6): “99.9% uptime monitoring”; “We monitor uptime at 99.9%.”

Evidence: doc4_infrastructure_config.md:21-23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)” and “Status Page: Heroku built-in status page only.”

Delta: The record identifies no application uptime monitor. Heroku’s status page is not evidence of a FlowMetrics uptime-monitoring system or a measured 99.9% application result.

**Finding 4: Redundancy and automatic failover are not supported by the documented topology — [delta]**

Claim (doc1_seller_listing_description.md:18; doc9_seller_s_technical_claims_verbatim.md:5): “platform-level redundancy and automatic failover”; “The system has redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4_infrastructure_config.md:4-8 — one standard-1x dyno, the PostgreSQL database running on the same dyno as the application, no read replicas, and no separate database instance.

Delta: The supplied topology documents a single application/database placement and no read replica or separate database instance. The available evidence does not support the claimed redundancy and automatic failover; it conflicts with the claim’s ordinary meaning. No deployment configuration was supplied for independent verification.

**Finding 5: Critical-path test coverage claim is contradicted — [delta]**

Claim (doc1_seller_listing_description.md:21; doc9_seller_s_technical_claims_verbatim.md:7): “Well-documented codebase with comprehensive test coverage across critical paths.”

Evidence: doc3_git_history_summary.md:24-29, 32 — no CI/CD pipeline; 12 unit tests, all in `test/utils/`; zero integration tests; zero payment-path tests; and no documented code-review process.

Delta: The documented test inventory does not support comprehensive coverage across critical paths, particularly payment and integration paths. The documentation claim is also not substantiated by the supplied history summary.

**Finding 6: No-lock-in claim conflicts with dependency and account-control records — [delta]**

Claim (doc1_seller_listing_description.md:22; doc9_seller_s_technical_claims_verbatim.md:9): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in”; “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supplies product matching, price comparison, and category classification; 40% of features depend on it; termination requires 90 days’ notice; and no fallback is implemented. doc8_external_dependency_list.md:25-32 — Google OAuth has no contractual relationship, and DNS is personally managed by Dave through GoDaddy with no secondary DNS.

Delta: The documented dependency concentration, absent fallback, non-contractual Google OAuth exposure, and personally controlled DNS are inconsistent with an unqualified “no lock-in” claim.

**Finding 7: DataEnrich dependency has no implemented fallback — [partial]**

Claim (doc1_seller_listing_description.md:9; doc2_tech_stack_description_as_provided_by_se.md:5): The product provides product intelligence and real-time product data as part of the operating platform.

Evidence: doc8_external_dependency_list.md:7-12 — 40% of features depend on DataEnrich.io; “Fallback: None implemented”; if the API is unavailable or pricing changes, the product degrades to basic mode without enrichment.

Delta: The product capability exists subject to the dependency, but continuity is partial: the documented failure mode removes enrichment functionality for a substantial feature share.

**Finding 8: Revenue arithmetic is inconsistent — [derived]**

Basis: doc5_stripe_export_summary.md:9-11 — 91 Pro accounts at $149/mo and stated Pro MRR of $13,549.

doc5_stripe_export_summary.md:14-19 — Enterprise Stripe MRR of $2,451; stated Stripe total of $16,000; and three wire-transfer contracts at $8,000/mo each.

Derivation: 91 × $149 = $13,559. Corrected Stripe MRR = $13,559 + $2,451 = $16,010. Corrected combined MRR = $16,010 + (3 × $8,000) = $40,010. The document instead states $13,549, $16,000, and the seller listing states $40,000.

Consequence: The supplied revenue records do not reconcile exactly. The difference is $10/month, but the error affects the stated calculation supporting the asking-price multiple.

Escalates: None.

**Finding 9: “120 active accounts” is not supported without qualification — [partial]**

Claim (doc1_seller_listing_description.md:9, 12-13): “a loyal customer base of 120 active accounts”; “Active Customers: 120.”

Evidence: doc6_crm_export_summary.md:3, 12-18 — all 120 accounts are marked active, but 23 are labelled “Phantom/Inactive,” have no payment events in the last 90 days, last logged in 60-120 days ago, and have no explanatory notes. doc7_customer_contracts_summary.md:36-45 also records eight free pilot agreements and 15 no-contract accounts.

Delta: The CRM status field supports a count of 120 marked active, but the underlying activity evidence does not support presenting all 120 as unqualified active, engaged, revenue-bearing customers.

## Coverage

The claim surface was enumerated across nine documents at roughly 100 distinct claims. I prioritized safety/reliability, architecture, operational continuity, material dependencies, testing, and financial/customer claims.

The review individually verified a material priority subset, not all approximately 100 claims. Authentication/OAuth implementation, SSL configuration, deployment manifests, actual Heroku settings, code, CI configuration, backup artifacts, uptime measurements, access controls, and secrets handling were not independently inspected because the data room supplied summaries only. Those claims remain [unverifiable], not [delta]. Lower-priority micro-claims and many marketing statements were also not exhaustively tested. This coverage is sufficient to establish a Material recommendation, but it is not a full code review, pen-test, or operational certification.

## Questions for the seller before closing

1. Provide backup job logs, successful backup identifiers, restore-test evidence, retention configuration, and failure-alert configuration covering the period since 2026-07-30.
2. Provide application-level uptime-monitoring configuration and historical measurements supporting the 99.9% figure.
3. Provide Heroku process formation, dyno-count, database, failover, and recovery configuration showing what “redundancy” and “automatic failover” mean operationally.
4. Provide the complete test suite, CI history, coverage report, and payment/integration test evidence.
5. Provide executed DataEnrich, Google, GoDaddy, Stripe, Twilio, Heroku, and Redis agreements, account ownership details, termination rights, and a documented migration plan.
6. Explain why 23 CRM accounts are labelled Phantom/Inactive while all 120 are marked active, and provide payment and engagement evidence for the claimed customer base.
7. Reconcile the Pro-plan, Stripe, wire-transfer, blended-MRR, and asking-price calculations.
8. Provide signed terms or recorded acceptance for the 15 no-contract accounts and status/conversion results for the eight pilots.
