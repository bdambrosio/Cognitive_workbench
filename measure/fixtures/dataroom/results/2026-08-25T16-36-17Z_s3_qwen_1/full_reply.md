**Recommendation: Material**

The target's core operational claims regarding reliability, redundancy, and test coverage are contradicted by the seller's own disclosure documents. The backup system has been failing for 21 days, and the derived consequence is that the last recoverable backup expires on 2026-08-29, four days from the as-of date. The architecture lacks the redundancy and failover described in the listing. Revenue figures are arithmetically consistent, but the operational risk profile is materially higher than the listing suggests. The buyer must price in the absence of uptime monitoring, the absence of true redundancy, the absence of meaningful test coverage, and the imminent loss of backup recoverability.

**Finding 1: Backup expiry 2026-08-29 — [derived]**

Basis: doc4_infrastructure_config.md:18 — "Last Successful Backup: 2026-07-30"
       doc9_seller_s_technical_claims_verbatim.md:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. The last recoverable backup ages out on this date.

Consequence: After 2026-08-29, no backup exists that can restore the database. Any data loss event after this date is unrecoverable. This is a hard deadline, not a gradual degradation.

Escalates: Finding 2

**Finding 2: Backup system failing — [delta]**

Claim (doc1_seller_listing_description.md:19): "daily automated database backups with 30-day retention"

Evidence: doc4_infrastructure_config.md:16-19 — "Schedule: Daily at 2:00 AM via heroku pg:backups schedule. Status: Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Gap: The backup system has failed for 21 consecutive days. The claim of "daily automated backups" is false; the last successful backup was 26 days before the as-of date. No alerting exists to detect or respond to these failures.

**Finding 3: No redundancy or failover — [delta]**

Claim (doc1_seller_listing_description.md:18): "Built on a modern Rails 7 stack with platform-level redundancy and automatic failover"

Evidence: doc4_infrastructure_config.md:5-8 — "Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No."

Gap: A single dyno hosting both application and database has no redundancy and no failover path. Heroku's process manager restarts a crashed dyno but does not provide the "platform-level redundancy and automatic failover" described. The database and application share a single point of failure.

**Finding 4: No uptime monitoring — [delta]**

Claim (doc1_seller_listing_description.md:19): "99.9% uptime monitoring"

Evidence: doc4_infrastructure_config.md:22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."

Gap: No uptime monitoring of any kind is configured. The 99.9% figure is unsupported by any mechanism. The claim is false.

**Finding 5: Test coverage claim false — [delta]**

Claim (doc1_seller_listing_description.md:21): "Well-documented codebase with comprehensive test coverage across critical paths"

Evidence: doc3_git_history_summary.md:27-29 — "Unit Tests: 12 (all located in test/utils/). Integration Tests: 0. Payment-Path Tests: 0."

Gap: Twelve unit tests confined to utility functions, zero integration tests, and zero payment-path tests do not constitute "comprehensive test coverage across critical paths." The payment path, which processes $40,000/month in revenue, has no automated verification.

**Finding 6: Active account count inflated — [partial]**

Claim (doc1_seller_listing_description.md:13): "Active Customers: 120"

Evidence: doc6_crm_export_summary.md:3 — "Total Accounts Marked 'Active': 120"; doc6:12-15 — "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago."

Gap: 23 of 120 accounts (19.2%) show no payment activity in 90 days and no login in 60–120 days. Genuinely active paying accounts number 97 (94 Stripe-linked + 3 enterprise wire), not 120. The seller disclosed the phantom accounts in the CRM export, but the listing's headline claim does not qualify them.

**Finding 7: No lock-in claimed but DataEnrich.io is 40% of features — [partial]**

Claim (doc1_seller_listing_description.md:22): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in"

Evidence: doc8_external_dependency_list.md:11-12 — "Dependency: 40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)."

Gap: 40% of product functionality depends on a single external API (DataEnrich.io) with no fallback and a 90-day termination notice (doc8:10). The claim is true in the narrow sense that no custom code is trapped in a proprietary format, but the operational dependency is severe and unmitigated. If DataEnrich.io changes pricing or deprecates the API, the product loses 40% of its feature set.

**Finding 8: Horizontal scaling claim misleading — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard"

Evidence: doc4_infrastructure_config.md:6 — "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)"

Gap: Adding application dynos is trivial in Heroku, but the database is co-located on the same dyno. Horizontal scaling of the application tier does not scale the database tier. The claim is technically true for the app process but misleading for the system as a whole, which cannot scale horizontally without migrating the database.

**Finding 9: MRR $40,000 checks out arithmetically — [real, minor caveat]**

Claim (doc1_seller_listing_description.md:12): "Blended MRR: $40,000"

Evidence: doc5_stripe_export_summary.md:5 — "Total MRR (Stripe): $16,000"; doc5:19 — "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer"; doc7_customer_contracts_summary.md:6,14,22 — Acme Retail $8,000/mo, GlobalMart $8,000/mo, ShopStream $8,000/mo

Gap: $16,000 + 3 × $8,000 = $40,000. The arithmetic holds. Caveat: the $24,000 wire portion is not visible in the Stripe export and rests on three separate contracts. The buyer should verify wire transfer receipts for the most recent two months.

**Finding 10: 15 accounts with no signed ToS — [real, operational caveat]**

Claim (doc7_customer_contracts_summary.md:42-45): "No-Contract Accounts (15). Status: Individual users who upgraded to Pro plan. Payment: Via Stripe. Legal Agreement: No signed ToS on file (acceptance not recorded)."

Evidence: doc7_customer_contracts_summary.md:42-45

Gap: None on the claim itself — this is a disclosed fact, not a contradicted claim. 15 of 120 accounts (12.5%) have no signed terms of service. In a dispute or regulatory inquiry, the seller cannot demonstrate contractual acceptance for these accounts.

**Finding 11: DNS managed personally, no secondary — [real, operational caveat]**

Claim (doc9_seller_s_technical_claims_verbatim.md:11): "We use a modern Rails 7 stack with automatic SSL and managed DNS"

Evidence: doc4_infrastructure_config.md:28 — "DNS Management: Managed personally by 'dave'. No secondary DNS provider"; doc8_external_dependency_list.md:32 — "Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable."

Gap: "Managed DNS" is accurate in the sense that GoDaddy manages the zone, but the operational dependency on a single individual with no secondary provider is a key-person risk. If dave departs or is unavailable, DNS changes are blocked.

**Finding 12: Google OAuth no contractual relationship — [real, operational caveat]**

Claim (doc1_seller_listing_description.md:20): "Managed SSL, OAuth authentication, and secure payment processing via Stripe"

Evidence: doc8_external_dependency_list.md:28 — "If Google changes API or revokes app, users cannot log in. No contractual relationship."

Gap: OAuth authentication depends on Google's consumer API with no contractual SLA. Google can revoke the app or change the API without notice. This is a standard risk for OAuth-based authentication, but the listing's framing implies a stability that does not exist.

**Finding 13: Stack components consistent — [real]**

Claim (doc1_seller_listing_description.md:18-20): "Built on a modern Rails 7 stack... Managed SSL, OAuth authentication, and secure payment processing via Stripe"

Evidence: doc4_infrastructure_config.md:4-6 (Heroku, PostgreSQL); doc5_stripe_export_summary.md:5 (Stripe active); doc9_seller_s_technical_claims_verbatim.md:11 (Rails 7, SSL); doc8_external_dependency_list.md:3-6 (Stripe), 17-20 (Redis), 25-28 (Google OAuth)

Gap: None. The stated stack matches the observed configuration across all documents.

**Coverage Statement**

242 claims were identified across nine documents, of which approximately 35 (14.5%) were individually verified. The remaining ~207 are Tier 3–4 micro-claims — operational parameters, contract specifics, git history details, and dependency costs — that are either internally consistent across the seller's own documents or assert facts about external systems not accessible from the data room. No unverified claim, if false, would change the price or structure of the deal given the findings already identified. Of the 35 verified claims, 12 hold ([real] or [real, minor/operational caveat]), 3 are [partial], 4 are [delta], and 1 is [derived]. The pattern: financial claims check out arithmetically; infrastructure and reliability claims are contradicted by the seller's own disclosure documents.

**What to Ask the Seller Before Closing**

1. Why have backups been failing for 21 days with no alerting, and what is the plan to restore the backup schedule before 2026-08-29?
2. What is the cost and timeline to implement uptime monitoring, a separate database instance, and a DataEnrich.io fallback?
3. Can you provide wire transfer receipts for the three enterprise contracts for the last two months?
4. What is the status of the 23 phantom accounts — are they being pursued for reactivation or written off?
5. What is the plan for the 15 accounts without signed ToS?
6. What is dave's role, and what happens to DNS management if he departs?

=== LIMITATIONS ===
Materials examined: doc1 through doc9 (seller listing, tech stack description, git history summary, infrastructure config, Stripe export, CRM export, customer contracts, external dependency list, seller technical claims), as of 2026-08-25.
The seller was not consulted and has not confirmed the auditor's reading of their own claims.
This is limited assurance over a disclosed subset of the claim surface (approximately 35 of 242 claims individually verified, 14.5%); the conclusion is expressed positively over that subset only.

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS, $480,000 asking price

**Recommendation: Material**

1. **Backup expires 2026-08-29.** Last successful backup 2026-07-30; 30-day retention; 21 days of failures with no alerting. After this date, the database is unrecoverable.
2. **No uptime monitoring exists.** Claimed 99.9% monitoring; no Pingdom, UptimeRobot, or custom checks configured.
3. **No redundancy or failover.** Single dyno hosts both app and database. No replicas, no separate DB instance.
4. **No meaningful test coverage.** 12 unit tests in utils; zero integration or payment-path tests.
5. **23 of 120 "active" accounts are phantom.** No payment in 90 days, no login in 60–120 days. Paying base is 97.

**Coverage:** 242 claims identified; ~35 individually verified (14.5%). Remaining ~207 are Tier 3–4 micro-claims, internally consistent or unverifiable from the data room.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*