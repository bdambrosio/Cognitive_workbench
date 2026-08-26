**Recommendation: Material**

Of the 29 claims examined, 4 are `[delta]` and 5 are `[partial]`. The most severe gap is the absence of working automated backups, compounded by a 30-day retention window that expires on 2026-08-29. The stated 99.9% uptime figure is unmeasured, and the claimed platform redundancy does not exist in the implementation. These findings significantly change the risk profile and must be priced in explicitly.

**Correction Note (§7):** The claim surface count is corrected from 40 to 29. The initial count was an overcount; sentence-by-sentence enumeration of the three claim sources (doc1, doc2, doc9) yields 29 distinct checkable assertions. All coverage figures below divide by 29.

### Findings

**Finding 1: Daily automated database backups — [delta]**

Claim (doc1:19, doc9:10, doc2:3 S4): "99.9% uptime monitoring and daily automated database backups with 30-day retention" / "The database is backed up daily to Heroku's managed storage with 30-day retention" / "PostgreSQL, which is configured with daily scheduled backups to ensure data integrity and recoverability."

Evidence: doc4:16 — "Schedule: Daily at 2:00 AM via heroku pg:backups schedule"; doc4:17 — "Status: Failures recorded for the last 21 days."; doc4:18 — "Last Successful Backup: 2026-07-30"; doc4:19 — "Alerting: None configured for backup failures."

Gap: The backup job has been failing for 21 consecutive days. The last successful backup was 2026-07-30. No alerting is configured. The backups are not occurring daily.

**Finding 2: 99.9% uptime monitoring — [delta]**

Claim (doc1:19, doc9:6): "99.9% uptime monitoring" / "We monitor uptime at 99.9%."

Evidence: doc4:22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."; doc4:23 — "Status Page: Heroku built-in status page only."

Gap: No uptime monitoring tool is deployed. The 99.9% figure is unmeasured; there is no mechanism to verify or report uptime. The enterprise contracts (doc7:8, doc7:16, doc7:24) carry 99.9% or 99.5% SLAs with service-credit remedies, but the seller has no means of detecting a breach.

**Finding 3: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18, doc2:3 S2, doc9:5): "Built on a modern Rails 7 stack with platform-level redundancy and automatic failover" / "provides automatic scaling and failover capabilities at the platform level" / "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: doc4:5 — "Dyno: standard-1x (1GB RAM, 0.5 CPU)"; doc4:6 — "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)"; doc4:7 — "Read Replicas: None."; doc4:8 — "Separate DB Instance: No."

Gap: A single dyno hosts both the application and the database. There are no read replicas and no separate database instance. If the dyno fails, both the application and the database are unavailable. Heroku's process manager restarts crashed processes, but this is not architectural redundancy or failover.

**Finding 4: Comprehensive test coverage — [delta]**

Claim (doc1:21, doc9:7): "Well-documented codebase with comprehensive test coverage across critical paths" / "comprehensive test coverage across all critical paths."

Evidence: doc3:27 — "Unit Tests: 12 (all located in test/utils/)"; doc3:28 — "Integration Tests: 0"; doc3:29 — "Payment-Path Tests: 0"; doc3:25 — "CI/CD Pipeline: None configured."; doc3:30 — "Staging Environment: None."; doc3:31 — "Branch Protection: None."; doc3:32 — "Code Review Process: None documented."

Gap: 12 unit tests in a utils directory, zero integration tests, zero payment-path tests. The payment path is the most critical path in a SaaS business. "Comprehensive test coverage across all critical paths" is false.

**Finding 5: 120 active accounts — [partial]**

Claim (doc1:9, doc1:13): "120 active accounts" / "Active Customers: 120."

Evidence: doc6:3 — "Total Accounts Marked 'Active': 120"; doc6:12-16 — 23 accounts: "No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file."; doc7:36-40 — 8 pilot agreements, free 90-day trial, no payment obligation; doc7:42-45 — 15 accounts, "No signed ToS on file (acceptance not recorded)."

Gap: 120 accounts are marked active in the CRM, but 23 show no payment in 90 days, 8 are free pilots with no payment obligation, and 15 have no signed agreement. Genuinely paying contracted customers number 97 (94 Stripe-linked + 3 wire).

**Finding 6: No proprietary lock-in — [partial]**

Claim (doc1:22, doc9:9): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in" / "no lock-in."

Evidence: doc8:7-12 — DataEnrich.io: 90-day termination notice, 40% of features depend on it, no fallback; doc8:25-28 — Google OAuth: free tier, no contractual relationship; doc4:27-28 — GoDaddy DNS: managed personally by dave, no secondary.

Gap: Most integrations are on standard agreements, but DataEnrich.io creates a 40% feature dependency with no fallback and 90-day notice, Google OAuth has no contractual relationship, and GoDaddy DNS is a single point of failure.

**Finding 7: Scalable technology stack — [partial]**

Claim (doc1:9 S4): "a scalable technology stack."

Evidence: doc4:5 — single standard-1x dyno; doc4:6 — database on the same dyno; doc4:7 — no read replicas; doc4:8 — no separate DB instance.

Gap: The stack can scale in the Heroku dashboard sense (add dynos), but the database is co-located on the same dyno and does not scale independently.

**Finding 8: Turnkey acquisition — [partial]**

Claim (doc1:9 S5): "This is a turnkey acquisition opportunity."

Evidence: doc4:17-19 — backup failures for 21 days, no alerting; doc3:25 — no CI/CD; doc4:27-28 — DNS managed by one person; doc3:32 — no documented code review process.

Gap: The business is simple in structure, but the backup system is broken, there is no monitoring, and key operational knowledge is held by one person.

**Finding 9: Entire stack managed through Heroku — [partial]**

Claim (doc2:3 S8): "The entire stack is managed through Heroku's dashboard."

Evidence: doc4:4-8 — compute, database, cache on Heroku; doc4:27 — "DNS Provider: GoDaddy"; doc5:1-2 — payments through Stripe; doc8:7-12 — data enrichment through DataEnrich.io.

Gap: The compute, database, and cache are managed through Heroku, but DNS (GoDaddy), payments (Stripe), and data enrichment (DataEnrich.io) are managed through their respective platforms.

**Finding 10: MRR $40,000 — [real, minor caveat]**

Claim (doc1:9, doc1:12): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000" / "Blended MRR: $40,000."

Evidence: doc5:5 — "Total MRR (Stripe): $16,000"; doc5:19 — "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer." $16,000 + $24,000 = $40,000.

Gap: None on the arithmetic. Caveat: doc6:12-16 shows 23 of the 120 accounts have no payment events in 90 days. The MRR figure reflects what Stripe reports as active subscriptions.

**Finding 11: Managed DNS — [real, operational caveat]**

Claim (doc9:11): "managed DNS."

Evidence: doc4:27 — "DNS Provider: GoDaddy"; doc4:28 — "DNS Management: Managed personally by 'dave'. No secondary DNS provider."; doc8:29-32 — "Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable."

Gap: DNS is managed, but by a single individual with no secondary provider. This is a single point of failure for domain resolution.

**Finding 12: One-click horizontal scaling — [real, operational caveat]**

Claim (doc9:8): "adding dynos is a one-click operation in the Heroku dashboard."

Evidence: doc4:5 — single standard-1x dyno; doc4:6 — database on the same dyno as the application.

Gap: Adding dynos is indeed a one-click operation in the Heroku interface. However, the database is co-located on the same dyno, so adding application dynos does not scale the data layer.

**Finding 13: Redis session caching — [real, minor caveat]**

Claim (doc2:3 S5): "We utilize Redis for session caching."

Evidence: doc4:11-13 — "Heroku Redis Add-on, heroku-redis:bb-1, $50/mo."; doc8:17-20 — "If down, app is slow but functional."

Gap: None on the claim. Caveat: Redis is a single instance with no failover.

**Finding 14: Managed SSL — [real]**

Claim (doc1:20, doc9:11): "Managed SSL" / "automatic SSL."

Evidence: doc4:26 — "SSL: Heroku-managed, auto-renewed."

Gap: None.

**Finding 15: OAuth authentication — [real, minor caveat]**

Claim (doc1:20): "OAuth authentication."

Evidence: doc4:29 — Google OAuth listed in secrets; doc8:25-28 — "Google OAuth, Free tier. If Google changes API or revokes app, users cannot log in. No contractual relationship."

Gap: None on the claim. Caveat: OAuth is on a free tier with no contractual relationship.

**Finding 16: Secure payment via Stripe — [real]**

Claim (doc1:20, doc2:3 S6): "secure payment processing via Stripe" / "Payment processing is securely handled through Stripe."

Evidence: doc5:1-5 — Stripe processing 94 active subscriptions; doc8:3-6 — "standard merchant agreement," low risk.

Gap: None.

**Finding 17: Real-time analytics and product intelligence — [real, minor caveat]**

Claim (doc1:9 S1): "providing real-time analytics and product intelligence."

Evidence: doc8:7-12 — DataEnrich.io provides product matching, price comparison, category classification; 40% of features depend on it.

Gap: None on the claim. Caveat: 40% of features depend on a single external API with no fallback.

**Finding 18: Immediate cash flow — [real, minor caveat]**

Claim (doc1:9 S5): "immediate cash flow."

Evidence: doc5:5 — $16,000 Stripe MRR; doc5:19 — $24,000 wire MRR.

Gap: None. Caveat: 23 of 120 accounts show no recent payment activity (Finding 5).

**Finding 19: Standard SaaS stack — [real]**

Claim (doc2:3 S1, doc2:3 S9): "a standard, modern SaaS stack" / "a standard SaaS stack with platform-level redundancy."

Evidence: doc4:4-13 — Rails 7 on Heroku, PostgreSQL, Redis. Standard components.

Gap: None on the "standard" characterization. (The "platform-level redundancy" portion is addressed in Finding 3.)

**Finding 20: Stripe credit card and recurring billing — [real]**

Claim (doc2:3 S6): "supports both credit card transactions and recurring billing."

Evidence: doc5:7-15 — Pro Plan and Enterprise Plan subscriptions processed through Stripe.

Gap: None.

**Finding 21: Third-party data enrichment API — [real, minor caveat]**

Claim (doc2:3 S7): "we integrate with a third-party data enrichment API that provides real-time product data."

Evidence: doc8:7-12 — DataEnrich.io, $400/mo, product matching, price comparison, category classification.

Gap: None on the claim. Caveat: 40% of features depend on it with no fallback (Finding 6).

**Finding 22: Revenue model — recurring subscription — [real]**

Claim (doc1:15): "Revenue Model: Recurring subscription (Monthly & Annual Enterprise)."

Evidence: doc5:7-15 — monthly Pro and Enterprise subscriptions; doc7:5-27 — annual enterprise contracts.

Gap: None.

**Finding 23: Revenue-positive — [unverifiable]**

Claim (doc1:9 S1): "revenue-positive."

Evidence: No cost data in the materials. MRR is $40,000 (doc5:5, doc5:19) but operating costs are not stated.

Gap: Cannot be verified from available materials.

**Finding 24: Low churn — [unverifiable]**

Claim (doc1:9 S4, doc1:14): "low churn" / "Churn: Low (stable base of recurring subscribers)."

Evidence: No churn data in the materials. doc6 shows 23 phantom accounts but no historical churn rate.

Gap: Cannot be verified from available materials.

**Finding 25: Minimal operational overhead — [unverifiable]**

Claim (doc1:9 S4): "minimal operational overhead."

Evidence: The claim is subjective. The materials show a small infrastructure footprint but also broken backups, no monitoring, and no CI/CD.

Gap: Cannot be verified from available materials.

**Finding 26: Proven product-market fit — [unverifiable]**

Claim (doc1:9 S5): "proven product-market fit."

Evidence: The materials show customers and revenue but no retention, NPS, or expansion data.

Gap: Cannot be verified from available materials.

**Finding 27: Well-documented codebase — [unverifiable]**

Claim (doc1:21, doc9:7): "Well-documented codebase."

Evidence: No documentation is included in the data room. doc3 shows 847 commits but no documentation artifacts.

Gap: Cannot be verified from available materials. (The test coverage portion of the same claim is Finding 4.)

**Finding 28: Available under varying load with no manual intervention — [unverifiable]**

Claim (doc2:3 S3): "the application remains available even under varying load conditions, with no manual intervention required."

Evidence: No load data or auto-scaling configuration in the materials. Single dyno (doc4:5).

Gap: Cannot be verified from available materials.

**Finding 29: Customers are engaged — [unverifiable]**

Claim (doc1:25): "the customers are engaged."

Evidence: doc6:15 shows last login dates for phantom accounts (60-120 days ago). No engagement data for the paying customer base.

Gap: Cannot be verified from available materials.

### Derived Findings

**Finding D1: Last recoverable backup date — [derived]**

Basis: doc4:18 — "Last Successful Backup: 2026-07-30"
       doc9:10 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. The last successful backup will be purged on 2026-08-29.

Consequence: After 2026-08-29, there will be no recoverable backup of the database. If a data loss event occurs after this date, the business cannot be restored. Combined with Finding 1 (backups failing for 21 days), the buyer is acquiring a business with no working backup and a finite window of recoverability.

Escalates: Finding 1

**Finding D2: Backup failure onset — [derived]**

Basis: doc4:17 — "Failures recorded for the last 21 days."
       doc4:18 — "Last Successful Backup: 2026-07-30"

Derivation: The last successful backup was 2026-07-30. Failures have been recorded for 21 days thereafter, placing the onset of failure at approximately 2026-07-31 and the document preparation date at approximately 2026-08-20.

Consequence: The backup system has been non-functional for at least 21 days. With no alerting (doc4:19), the seller may not be aware of the failure.

Escalates: Finding 1

### Coverage Statement

29 of 29 identified claims were examined. 4 are `[delta]`, 5 are `[partial]`, 13 are `[real]` or `[real, minor/operational caveat]`, and 7 are `[unverifiable]` due to absence of relevant data in the materials. All Priority 1 and Priority 2 claims were resolved. Priority 3 and 4 claims were checked; the 7 unverifiable claims are low-priority and their absence does not change the recommendation.

### What the Buyer Should Ask the Seller

1. Why have automated backups been failing for 21 days, and when will they be restored before closing?
2. How is the 99.9% uptime SLA in the enterprise contracts being met without any uptime monitoring?
3. What is the plan for the 23 phantom accounts with no payment in 90 days?
4. What is the 12-month churn rate?
5. What is the plan for the DataEnrich.io dependency (40% of features, no fallback, 90-day notice)?
6. Who will manage DNS and operational knowledge post-acquisition?

=== LIMITATIONS ===
1. Materials examined: doc1 through doc9 (data room), as of 2026-07-30 (most recent date in materials: doc4:18, doc3:21).
2. The seller was not consulted and has not confirmed this reading of their own claims.
3. Limited assurance engagement; coverage rests on 29 of 29 identified claims.

=== GAP MAP ===
**Target:** FlowMetrics — B2B e-commerce analytics SaaS, $480,000 asking price
**Recommendation:** Material

**Key Items:**
1. No working automated backups; last recoverable backup expires 2026-08-29. No alerting on failures.
2. 99.9% uptime SLA is unmeasured — no uptime monitor exists. Enterprise contracts carry SLA remedies the seller cannot trigger.
3. No platform redundancy — single dyno hosts both app and database. No failover.
4. Test coverage: 12 unit tests in utils, zero integration tests, zero payment-path tests. "Comprehensive" is false.
5. 120 "active" accounts: 23 have no payment in 90 days, 8 are free pilots, 15 have no signed agreement. Paying contracted customers: 97.

**Coverage:** 29 of 29 claims examined. 4 delta, 5 partial, 13 real/caveat, 7 unverifiable.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice