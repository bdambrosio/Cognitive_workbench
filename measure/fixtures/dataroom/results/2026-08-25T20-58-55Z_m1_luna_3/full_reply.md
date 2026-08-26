# Technical Claims Audit — FlowMetrics

**Recommendation: Material.** Of the 90 claims examined as the frozen claim surface, the materials support material gaps in recoverability, redundancy, monitoring, testing, customer/revenue reporting, and dependency resilience. The recommendation rests on the higher-priority claims checked; it is not a conclusion about the 90 claims as a whole.

As-of date for the materials: **2026-08-25**.

=== CLAIM SURFACE ===
90 claims

Surface count: doc1 — 43 claims; doc2 — 30 claims; doc9 — 17 claims. Claims were enumerated only from the three seller claim sources named by the buyer: doc1, doc2, and doc9. The surface is frozen at 90.

## Findings

**Finding 1: Backup program is failing — [delta]**

Claim (doc1_seller_listing_description.md:19): “**Reliability:** 99.9% uptime monitoring and daily automated database backups with 30-day retention.”

Evidence: doc4_infrastructure_config.md:15-19 — “**Schedule:** Daily at 2:00 AM via `heroku pg:backups schedule`”; “**Status:** Failures recorded for the last 21 days”; “**Last Successful Backup:** 2026-07-30”; “**Alerting:** None configured for backup failures.”

Gap: The seller claims daily automated backups, but the evidence records failures for the last 21 days and no failure alerting. The evidence does not establish that 30-day retention is actually operating.

**Finding 2: Claimed redundancy and automatic failover are not evidenced by the deployed topology — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:5): “The system has redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4_infrastructure_config.md:3-8 — “**Dyno:** `standard-1x` (1GB RAM, 0.5 CPU)”; “**Database:** `heroku-postgresql:standard-0` (running on the same dyno as the application)”; “**Read Replicas:** None”; “**Separate DB Instance:** No.”

Gap: The documented deployment has one application dyno, a database running on the same dyno, no read replicas, and no separate database instance. Those materials do not show application or database redundancy or an automatic database failover path.

**Finding 3: The claimed 99.9% uptime monitoring is absent — [delta]**

Claim (doc1_seller_listing_description.md:19): “**Reliability:** 99.9% uptime monitoring and daily automated database backups with 30-day retention.”

Evidence: doc4_infrastructure_config.md:21-23 — “**Uptime Monitor:** None (No Pingdom, UptimeRobot, or custom checks)”; “**Status Page:** Heroku built-in status page only.”

Gap: No external or custom uptime monitor is configured. The materials therefore do not support the asserted 99.9% monitoring program.

**Finding 4: Comprehensive critical-path test coverage is contradicted by the repository summary — [delta]**

Claim (doc1_seller_listing_description.md:21): “**Code Quality:** Well-documented codebase with comprehensive test coverage across critical paths.”

Evidence: doc3_git_history_summary.md:24-32 — “**CI/CD Pipeline:** None configured”; “**Unit Tests:** 12 (all located in `test/utils/`)”; “**Integration Tests:** 0”; “**Payment-Path Tests:** 0”; “**Staging Environment:** None”; “**Branch Protection:** None”; “**Code Review Process:** None documented.”

Gap: The available evidence shows only 12 unit tests, all in one utility directory, and no integration or payment-path tests. That is inconsistent with comprehensive coverage across critical paths.

**Finding 5: Revenue and active-account figures do not reconcile without undocumented treatment of accounts — [partial]**

Claim (doc1_seller_listing_description.md:9, 12-15): “We have built a loyal customer base of 120 active accounts”; “**Blended MRR:** $40,000”; “**Active Customers:** 120”; “**Revenue Model:** Recurring subscription (Monthly & Annual Enterprise).”

Evidence: doc5_stripe_export_summary.md:3-5, 19 — “**Total Active Subscriptions:** 94”; “**Total MRR (Stripe):** $16,000”; “The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer. These payments are not processed through Stripe and are not included in the $16,000 MRR figure above.” doc6_crm_export_summary.md:3-16 — “**Total Accounts Marked 'Active':** 120”; “**Stripe-Linked Accounts:** 94”; “**Enterprise Accounts (Wire Transfer):** 3”; “**Phantom/Inactive Accounts:** 23”; “No payment events in the last 90 days”; “Last login dates: 60-120 days ago.”

Gap: The CRM’s 120 active accounts include 23 accounts described as phantom/inactive. The 94 Stripe subscriptions plus three wire-transfer enterprise accounts do not independently establish 120 paying active accounts. The stated $40,000 MRR can be arithmetically consistent with $16,000 Stripe MRR plus $24,000 in wire contracts, but the account-count and “active” characterization remains materially qualified by the 23 phantom/inactive records.

**Finding 6: A material product capability depends on one API with no fallback — [partial]**

Claim (doc2_tech_stack_description_as_provided_by_se.md:5): “For our product intelligence features, we integrate with a third-party data enrichment API that provides real-time product data, allowing our users to make informed decisions about their inventory and pricing.”

Claim (doc9_seller_s_technical_claims_verbatim.md:9): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — “**DataEnrich.io**”; “**Dependency:** 40% of features depend on this API”; “**Termination Notice:** 90 days (either party)”; “**Fallback:** None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment).”

Gap: The materials show that 40% of features depend on DataEnrich.io and that no fallback exists. The evidence does not establish proprietary contractual lock-in, but it does show operational dependency and material feature degradation on outage or pricing change. The narrow verdict is partial rather than delta for the contractual no-lock-in assertion.

**Finding 7: DNS is a single-person, single-provider control point — [partial]**

Claim (doc1_seller_listing_description.md:18, 22): “**Scalable Architecture:** Built on a modern Rails 7 stack with platform-level redundancy and automatic failover”; “**Low Vendor Lock-in:** All third-party integrations are on standard SaaS agreements with no proprietary lock-in.”

Evidence: doc4_infrastructure_config.md:25-29 — “**DNS Provider:** GoDaddy”; “**DNS Management:** Managed personally by 'dave'. No secondary DNS provider”; “**Secrets Management:** 14 secrets stored in Heroku config vars … No secrets vault.” doc8_external_dependency_list.md:29-32 — “**GoDaddy**”; “**Risk:** Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable.”

Gap: The app’s reachability depends on a single DNS provider and personal seller-controlled administration, with no secondary DNS. The materials do not show proprietary lock-in, but they do show a material control and continuity dependency not disclosed by the broad low-lock-in claim.

**Finding 8: Recent maintenance activity is absent — [real, operational caveat]**

Claim (doc1_seller_listing_description.md:9, 18): “The business operates with low churn, a scalable technology stack, and minimal operational overhead”; “**Scalable Architecture:** Built on a modern Rails 7 stack with platform-level redundancy and automatic failover.”

Evidence: doc3_git_history_summary.md:20-22 — “**Last Commit:** 2026-07-30”; “**Commits in Last 30 Days:** 0.”

Gap: No direct claim says commits occur continuously, so this is not a delta on its own. It is an operational caveat relevant to the seller’s description of a turnkey, scalable business and should be explained before closing.

## Coverage

I checked 14 claims individually out of the frozen 90: the highest-priority recoverability, redundancy, monitoring, testing, revenue/account, dependency, DNS/control, and related operational claims. I did not individually test the remaining 76 claims, including most product-function, Rails implementation, authentication, payment-processing, scaling, documentation, and lower-impact marketing assertions. Those unchecked claims are not treated as findings; their absence matters because the recommendation is limited assurance and cannot be generalized to the remaining surface. I stopped after the checked material claims established a material risk profile; the unchecked claims were not needed to support that conclusion.

No separate finding is made for claims that could not be verified from the available materials. They remain outside checked coverage rather than being classified as deltas.

## Questions for the seller before closing

1. Provide backup execution logs, restore-test results, retention configuration, and the reason for the 21-day failure period.
2. Demonstrate a restore from the last successful backup and identify the actual last recoverable production state.
3. Explain the claimed failover architecture, including what fails over when the database and application share the documented dyno.
4. Provide historical uptime measurements and identify the system that allegedly records 99.9% uptime.
5. Reconcile the 120 CRM accounts, 94 Stripe subscriptions, three wire-transfer accounts, 23 phantom/inactive accounts, and $40,000 blended MRR with source-level payment records.
6. Provide all three enterprise contracts and confirm renewal, termination, SLA-credit, and feature obligations.
7. Identify the DataEnrich.io contract, pricing-change rights, service levels, termination terms, outage history, and migration or fallback plan.
8. Transfer or document DNS, Heroku, Stripe, DataEnrich.io, Twilio, OAuth, and secrets administration, including recovery access not dependent on Dave personally.
9. Provide the codebase documentation and test inventory supporting the “comprehensive” critical-path coverage claim.
10. Explain the absence of commits in the 30 days before the materials’ as-of date.

## Proposed technique-only METHOD.md edits

- Add a reconciliation check for business claims: compare CRM active counts, payment-provider subscriptions, wire-transfer contracts, and inactive-account flags before accepting customer or MRR claims.
- When a seller claims redundancy or failover, inspect the complete application, database, DNS, and control-plane topology separately; platform branding alone is not evidence of end-to-end failover.
- For backup claims, require both schedule configuration and recent successful execution or restore evidence; a configured schedule is not proof of recoverability.
- For monitoring claims, distinguish a provider SLA or built-in status page from an actual customer-facing uptime measurement path.

=== LIMITATIONS ===
Materials examined: doc1 through doc9 in the flowmetrics data room; as-of date 2026-08-25.
The seller was not consulted and has not confirmed our reading of their own claims.
This is a limited-assurance engagement resting on 14 individually checked claims out of the frozen 90-claim surface; the remaining 76 claims were not individually verified.

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS offered at $480,000.

**Recommendation: Material** — based on 14 of 90 claims checked.

- **Daily backups:** The listing claims daily automated backups with 30-day retention; the infrastructure record shows failures for the last 21 days and no alerting.
- **Redundancy/failover:** The seller claims platform-level redundancy; the documented system has one dyno, a database on the same dyno, no read replicas, and no separate database instance.
- **Uptime monitoring:** The listing claims 99.9% monitoring; the infrastructure record says no uptime monitor is configured.
- **Revenue/accounts:** The listing says 120 active accounts and $40,000 MRR; 23 CRM accounts are labelled phantom/inactive, while 94 Stripe subscriptions and three wire-transfer accounts are separately documented.
- **Critical dependency:** DataEnrich.io supplies functionality for 40% of features, with no fallback if it is unavailable or changes pricing.

**Coverage:** 14 of 90 claims were checked, prioritising recoverability, architectural invariants, operational controls, and revenue/account integrity. The remaining 76 claims were not individually checked and are not covered by the recommendation.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*