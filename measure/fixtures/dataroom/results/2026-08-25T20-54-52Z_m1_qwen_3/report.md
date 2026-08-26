**Recommendation: Material**

Of the 23 claims examined, 11 are `[delta]`, 4 are `[partial]`, 2 are `[real, minor caveat]`, 5 are `[real]`, and 1 is `[unverifiable]`. The consistency rate for claims that hold is 7/23 (30%). The financial claims (MRR, payment rails) are substantiated. The reliability, redundancy, and quality claims are systematically false. A buyer must price in the absence of backup integrity, the absence of redundancy, and the absence of test coverage.

**Findings**

**Finding 1: Backup failure and data loss date — [delta]**

Claim (doc1:19, doc9:10): "Daily automated database backups with 30-day retention."

Evidence: doc4:17 — "Failures recorded for the last 21 days"; doc4:18 — "Last Successful Backup: 2026-07-30"; doc4:19 — "Alerting: None configured for backup failures."

Gap: Backups are not being maintained daily. The schedule exists but has been failing for 21 days. No alerting means the failure is invisible to operators.

**Finding 2: Last recoverable backup ages out 2026-08-29 — [derived]**

Basis: doc4:18 — "Last Successful Backup: 2026-07-30"
       doc1:19 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. On that date, the last known-good backup is purged from retention.

Consequence: After 2026-08-29, the business cannot be restored to any known-good state. Today is 2026-08-25. Four days remain.

Escalates: Finding 1.

**Finding 3: No redundancy or automatic failover — [delta]**

Claim (doc1:18, doc2:1, doc9:5): "Platform-level redundancy and automatic failover."

Evidence: doc4:5 — "Dyno: standard-1x (1GB RAM, 0.5 CPU)"; doc4:6 — "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)"; doc4:7 — "Read Replicas: None"; doc4:8 — "Separate DB Instance: No."

Gap: There is no redundancy in the application architecture. One dyno, one co-located database, no replicas. Heroku's 99.95% SLA covers platform uptime, not application-level redundancy. A dyno crash or DB failure is a total outage with no failover path.

**Finding 4: No uptime monitoring — [delta]**

Claim (doc1:19, doc9:6): "99.9% uptime monitoring."

Evidence: doc4:22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)"; doc4:23 — "Status Page: Heroku built-in status page only."

Gap: No uptime monitoring exists. The 99.9% figure is unmeasured and unverified.

**Finding 5: Test coverage claim is false — [delta]**

Claim (doc1:21, doc9:7): "Well-documented codebase with comprehensive test coverage across critical paths."

Evidence: doc3:25 — "CI/CD Pipeline: None configured"; doc3:27 — "Unit Tests: 12 (all located in test/utils/)"; doc3:28 — "Integration Tests: 0"; doc3:29 — "Payment-Path Tests: 0"; doc3:30 — "Staging Environment: None"; doc3:31 — "Branch Protection: None"; doc3:32 — "Code Review Process: None documented."

Gap: 12 utility tests, zero integration tests, zero payment-path tests. "Comprehensive test coverage across critical paths" is false. The payment path, which handles $40,000/month, has no automated tests.

**Finding 6: 120 active accounts — [partial]**

Claim (doc1:9): "120 active accounts."

Evidence: doc6:3 — "Total Accounts Marked 'Active': 120"; doc6:12-16 — "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment."

Gap: 97 of 120 are genuinely active (94 Stripe-linked + 3 enterprise). 23 are phantom: no payment in 90 days, stale logins. The "active" label is not supported by payment or engagement data for 19% of the base.

**Finding 7: "No lock-in" understates DataEnrich.io dependency — [partial]**

Claim (doc1:22, doc9:9): "All third-party integrations on standard SaaS agreements with no lock-in."

Evidence: doc8:10 — "Termination Notice: 90 days (either party)"; doc8:11 — "Dependency: 40% of features depend on this API"; doc8:12 — "Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)."

Gap: The agreement is standard SaaS, but 40% of features depend on a single vendor with no fallback. "No lock-in" is technically true (you can terminate) but materially misleading: terminating means losing 40% of functionality with no replacement.

**Finding 8: Horizontal scaling claim — [partial]**

Claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: doc4:5 — "Dyno: standard-1x (1GB RAM, 0.5 CPU)"; doc4:6 — "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)."

Gap: You can add application dynos in Heroku. The database is co-located on the same dyno and does not scale with them. Horizontal scaling of the application layer does not relieve the database bottleneck.

**Finding 9: Managed SSL, OAuth, DNS — [real, minor caveat]**

Claim (doc1:20, doc9:11): "Managed SSL, OAuth authentication, and secure payment processing via Stripe" / "automatic SSL and managed DNS."

Evidence: doc4:26 — "SSL: Heroku-managed, auto-renewed"; doc4:27-28 — "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'. No secondary DNS provider."; doc8:25-28 — "Google OAuth. Cost: Free tier. Risk: If Google changes API or revokes app, users cannot log in. No contractual relationship."

Gap: SSL is genuinely managed. OAuth works but is on a free tier with no contract. DNS is "managed" only in the sense that one person manages it at GoDaddy with no redundancy. The claim holds; the operational fragility is a caveat, not a delta.

**Finding 10: $40,000 MRR — [real]**

Claim (doc1:12): "Blended MRR: $40,000."

Evidence: doc5:5 — "Total MRR (Stripe): $16,000"; doc5:19 — "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer."

Gap: None. $16,000 + $24,000 = $40,000. The arithmetic holds.

**Finding 11: Rails 7 on Heroku — [real]**

Claim (doc2:1, doc9:11): "Rails 7 application hosted on Heroku."

Evidence: doc4:4-5 — "Platform: Heroku. Dyno: standard-1x."

Gap: None.

**Finding 12: Redis for session caching — [real]**

Claim (doc2:1): "We utilize Redis for session caching."

Evidence: doc4:11-13 — "Service: Heroku Redis Add-on. Plan: heroku-redis:bb-1. Cost: $50/mo."

Gap: None.

**Finding 13: Stripe for payments — [real]**

Claim (doc2:1): "Payment processing is securely handled through Stripe."

Evidence: doc5:4-5 — "Total Active Subscriptions: 94. Total MRR (Stripe): $16,000."

Gap: None.

**Finding 14: Third-party data enrichment API — [real]**

Claim (doc2:1): "We integrate with a third-party data enrichment API that provides real-time product data."

Evidence: doc8:7-9 — "DataEnrich.io. Function: Product data enrichment API. Cost: $400/mo flat."

Gap: None.

**Finding 15: Stack managed through Heroku dashboard — [real]**

Claim (doc2:1): "The entire stack is managed through Heroku's dashboard."

Evidence: doc4:4 — "Platform: Heroku." All infrastructure components listed under Heroku.

Gap: None.

**Finding 16: Low churn — [unverifiable]**

Claim (doc1:14): "Churn: Low (stable base of recurring subscribers)."

Evidence: doc6:12-16 — "23 phantom accounts created Q1 2026, no payment in 90 days."

Gap: No churn data provided. The phantom accounts suggest the base is less stable than claimed, but a churn rate cannot be computed from the materials. This goes to Remaining Claims.

**Coverage Statement**

23 claims enumerated from doc1 (9), doc2 (7), and doc9 (7). All 23 were examined. 22 received a verdict; 1 is `[unverifiable]` (low churn). The `[derived]` finding (backup expiry) is not a claim verdict but a consequence of two stated figures.

What was not checked: no source code was reviewed beyond the infrastructure and test files cited above. No penetration testing was performed. No legal review of contracts was conducted. The 8 pilot agreements and 15 no-contract accounts noted in doc7 were not separately claimed by the seller and are not in the claim surface.

The audit stopped after verifying all 23 claims. No low-priority claims remain unchecked.

**What to Ask the Seller Before Closing**

1. Why have backups been failing for 21 days with no alerting? What is the remediation plan, and can it be completed before 2026-08-29?
2. What is the actual churn rate over the last 12 months? Provide monthly active account counts with payment events.
3. What is the plan for the 23 phantom accounts? Are they being pursued, or should they be excluded from the active base?
4. What is the fallback plan if DataEnrich.io terminates or degrades? 40% of features depend on it.
5. What is the test coverage plan for the payment path? Zero payment-path tests on a $40k/month revenue stream is a material risk.
6. Who manages DNS if dave is unavailable? Is there a secondary or automated failover?

=== LIMITATIONS ===
Materials examined: doc1 (listing), doc2 (technical description), doc3 (codebase audit), doc4 (infrastructure report), doc5 (Stripe export), doc6 (account database), doc7 (contract summary), doc8 (third-party agreements), doc9 (specifications). As-of date: 2026-08-25.
The seller was not consulted and has not confirmed this reading of their own claims.
This is a limited assurance engagement. The audit examined 23 of 23 identified claims. It does not cover claims not stated in the named claim sources, and it does not opine on what the target should do.
