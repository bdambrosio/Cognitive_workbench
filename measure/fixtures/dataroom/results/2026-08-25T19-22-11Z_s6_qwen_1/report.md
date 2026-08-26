**Recommendation: Material**

Of the 32 claims enumerated from the claim sources, 28 were examined individually. Seven carry a `[delta]` verdict and six carry a `[partial]` verdict. The deltas are not isolated: they cluster in the direction of overstating operational robustness (backups, redundancy, failover, monitoring, test coverage) and understating dependency risk (lock-in, account quality). The backup failure is an active emergency: the last recoverable backup expires on 2026-08-29, four days from the as-of date of these materials. The buyer must price these gaps explicitly.

---

**Finding 1: Daily automated backups with 30-day retention — [delta]**

Claim (doc1:19; doc2:5; doc9:6,10): "Daily automated database backups with 30-day retention."

Evidence: doc4:16–19 — "Schedule: Daily at 2:00 AM via heroku pg:backups schedule. Status: Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Gap: Backups have been failing for 21 consecutive days. The last successful backup is 2026-07-30. No alerting is configured to detect further failures. The claim of functioning daily backups is false.

**Finding 2: Last recoverable backup expires 2026-08-29 — [derived]**

Basis: doc4:18 — "Last Successful Backup: 2026-07-30"
       doc1:19; doc9:10 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. As of 2026-08-25, the last good backup ages out in 4 days.

Consequence: After 2026-08-29, no recoverable backup of the production database exists. Any data loss or corruption after that date is unrecoverable from backup. This is not a future risk; it is a date.

Escalates: Finding 1.

**Finding 3: Platform-level redundancy — [delta]**

Claim (doc1:18; doc2:3,5; doc9:5): "Platform-level redundancy."

Evidence: doc4:5–8 — "Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No."

Gap: There is one dyno, one database instance, and no replicas. No redundancy exists. The claim is false.

**Finding 4: Automatic failover — [delta]**

Claim (doc1:18; doc2:3; doc9:5): "Automatic failover."

Evidence: doc4:5–8 — single dyno, single DB on same dyno, no replicas, no separate DB instance.

Gap: With a single dyno and the database on the same dyno, there is no failover target. Heroku restarts crashed dynos, but a restart is not a failover; it is the same machine coming back. No failover mechanism exists. The claim is false.

**Finding 5: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "99.9% uptime monitoring."

Evidence: doc4:22–23 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Gap: No uptime monitoring is configured. The Heroku built-in status page reports Heroku's infrastructure, not this application's availability. The claim is false.

**Finding 6: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): "Comprehensive test coverage across critical paths."

Evidence: doc3:27–29 — "Unit Tests: 12 (all in test/utils/). Integration Tests: 0. Payment-Path Tests: 0."

Gap: Twelve unit tests in a utility directory, zero integration tests, zero payment-path tests. The payment path is the revenue-critical path. The claim is false.

**Finding 7: No proprietary lock-in — [delta]**

Claim (doc1:22; doc9:9): "No proprietary lock-in."

Evidence: doc8:11–12 — "Dependency: 40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)."

Gap: Forty percent of the product's features depend on a single third-party API (DataEnrich.io) with no fallback. If DataEnrich.io terminates the relationship (90-day notice per doc8:10), the product loses 40% of its functionality. This is significant lock-in. The claim is false.

**Finding 8: 120 active accounts — [delta]**

Claim (doc1:9,13): "120 active accounts."

Evidence: doc6:3 — "Total Accounts Marked 'Active': 120." doc6:12–16 — "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment."

Gap: 23 of the 120 accounts marked active have no payment events in 90 days and no logins in 60–120 days. True active accounts: 97 (94 Stripe subscriptions + 3 enterprise wire contracts). The claim overstates active accounts by 23.

**Finding 9: Stripe handles all payment processing — [partial]**

Claim (doc2:5): "Payment processing is securely handled through Stripe, which supports both credit card transactions and recurring billing."

Evidence: doc5:19 — "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer. These payments are not processed through Stripe and are not included in the $16,000 MRR figure above."

Gap: Stripe processes 94 subscriptions ($16,000 MRR). Three enterprise contracts ($24,000/mo, 60% of total MRR) are paid via wire transfer outside Stripe. The claim is true for the subscription tier and false for the enterprise tier.

**Finding 10: Technology stack is scalable — [partial]**

Claim (doc1:9): "a scalable technology stack."

Evidence: doc4:5–6 — single standard-1x dyno, DB on same dyno. doc9:8 — "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Gap: Heroku supports adding dynos, but the current deployment is a single dyno with the database co-located on it. No horizontal scaling is configured or demonstrated. The claim is aspirational, not demonstrated.

**Finding 11: Third-party integrations on standard SaaS agreements — [partial]**

Claim (doc1:22; doc9:9): "All third-party integrations are on standard SaaS agreements."

Evidence: doc8:25–28 — "Google OAuth. Function: Authentication. Cost: Free tier. Risk: If Google changes API or revokes app, users cannot log in. No contractual relationship."

Gap: Most integrations are on standard agreements, but Google OAuth has no contractual relationship at all, and DataEnrich.io has a 90-day termination notice with no fallback (doc8:10–12). The claim is mostly true with two specific exceptions.

**Finding 12: Heroku automatic scaling / horizontal scaling — [partial]**

Claim (doc2:3; doc9:8): "automatic scaling" / "The stack is designed to scale horizontally."

Evidence: doc4:5 — single standard-1x dyno. doc4:6 — DB on same dyno.

Gap: Heroku technically supports adding dynos, but the current configuration is a single dyno with the database co-located. No evidence of scaling being used or configured. The claim implies a capability that is not in use.

**Finding 13: Everything managed via Heroku dashboard — [partial]**

Claim (doc2:5): "The entire stack is managed through Heroku's dashboard."

Evidence: doc4:27–28 — "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'. No secondary DNS provider."

Gap: Most infrastructure is on Heroku, but DNS is managed via GoDaddy by a single individual. Not everything is managed via the Heroku dashboard.

**Finding 14: Managed DNS — [partial]**

Claim (doc9:11): "managed DNS."

Evidence: doc4:27–28 — GoDaddy, managed personally by one individual, no secondary provider. doc8:32 — "Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable."

Gap: DNS is managed, but by a single individual with no secondary. Single point of control. The claim is true in form but the operational context qualifies it.

**Finding 15: OAuth authentication — [real, minor caveat]**

Claim (doc1:20): "OAuth authentication."

Evidence: doc4:29 — "Google OAuth" listed in secrets. doc8:25–28 — Google OAuth for authentication, free tier, no contractual relationship.

Gap: OAuth is implemented via Google. No contractual relationship with Google; if Google revokes the app, users cannot log in. The claim holds; the caveat does not change the decision.

**Finding 16: Product intelligence — [real, minor caveat]**

Claim (doc1:9): "product intelligence."

Evidence: doc8:8 — "Product data enrichment API (product matching, price comparison, category classification)."

Gap: Product intelligence is provided via DataEnrich.io, but 40% of features depend on it with no fallback. The claim holds; the dependency is noted in Finding 7.

**Finding 17: Asking price is 12x blended MRR — [real, minor caveat]**

Claim (doc1:5): "$480,000 (12x blended MRR)."

Evidence: doc5:5 — "Total MRR (Stripe): $16,000." doc5:19 — 3 enterprise at $8,000/mo via wire = $24,000. $16,000 + $24,000 = $40,000. $480,000 / $40,000 = 12x.

Gap: The arithmetic is correct given the stated MRR. The MRR figure is supported (Finding 18). The caveat is that 60% of MRR is outside Stripe.

**Finding 18: Blended MRR is $40,000 — [real]**

Claim (doc1:9,12): "blended Monthly Recurring Revenue (MRR) stands at $40,000."

Evidence: doc5:5 — $16,000 Stripe MRR. doc5:19 — $24,000 wire. Total: $40,000.

Gap: None.

**Finding 19: Customers range from independent merchants to mid-market enterprise — [real]**

Claim (doc1:9): "ranging from independent merchants to mid-market enterprise retailers."

Evidence: doc5:9 — 91 Pro Plan accounts at $149/mo. doc7:3–5 — 3 enterprise contracts at $8,000/mo.

Gap: None.

**Finding 20: Revenue driven by mix of enterprise and month-to-month — [real]**

Claim (doc1:9): "driven by a mix of high-value enterprise contracts and a broad base of month-to-month subscribers."

Evidence: doc7:3–5 — 3 enterprise at $8,000/mo. doc5:4 — 94 active subscriptions.

Gap: None.

**Finding 21: Hosted on Heroku — [real]**

Claim (doc2:3): "hosted on Heroku."

Evidence: doc4:4 — "Platform: Heroku."

Gap: None.

**Finding 22: Database is PostgreSQL — [real]**

Claim (doc2:5): "Data persistence is handled by PostgreSQL."

Evidence: doc4:6 — "heroku-postgresql:standard-0."

Gap: None.

**Finding 23: Redis for session caching — [real]**

Claim (doc2:5): "We utilize Redis for session caching."

Evidence: doc4:11–12 — "Service: Heroku Redis Add-on. Plan: heroku-redis:bb-1."

Gap: None.

**Finding 24: Third-party data enrichment API — [real]**

Claim (doc2:5): "we integrate with a third-party data enrichment API."

Evidence: doc8:7–12 — DataEnrich.io, $400/mo, 40% of features depend on it.

Gap: None.

**Finding 25: Managed SSL — [real]**

Claim (doc1:20; doc9:11): "Managed SSL" / "automatic SSL."

Evidence: doc4:26 — "SSL: Heroku-managed, auto-renewed."

Gap: None.

**Finding 26: Operational overhead is minimal — [real, minor caveat]**

Claim (doc1:9): "minimal operational overhead."

Evidence: doc4:4–12 — managed services (Heroku, Redis, Stripe). doc3:25,30–32 — no CI/CD, no staging, no branch protection.

Gap: Infrastructure is largely managed, which does minimize ops overhead. The lack of CI/CD and staging increases deployment risk but does not change the fundamental claim.

**Finding 27: Revenue model is recurring subscription — [real, minor caveat]**

Claim (doc1:15): "Revenue Model: Recurring subscription (Monthly & Annual Enterprise)."

Evidence: doc7:29–34 — 94 month-to-month subscriptions. doc7:3–5 — 3 annual enterprise contracts. doc7:36–40 — 8 pilot agreements (free). doc7:42–45 — 15 no-contract accounts.

Gap: The revenue model is predominantly recurring. 8 pilot agreements are free with no payment obligation, and 15 accounts have no signed ToS on file. The claim holds; the caveats are noted.

**Finding 28: Product provides visibility into performance and pricing — [real, minor caveat]**

Claim (doc1:25): "visibility into product performance and pricing dynamics."

Evidence: doc8:8 — DataEnrich.io provides product matching, price comparison, category classification. doc7:11 — Acme Retail requires product matching, price comparison, real-time inventory sync.

Gap: The product provides these features via DataEnrich.io. The dependency is noted in Finding 7.

---

**Coverage statement**

32 claims were enumerated from the claim sources (doc1, doc2, doc9). 28 were examined individually. 4 claims were not examined: Claim 3 (real-time analytics latency — no evidence document addresses data freshness), Claim 9 (churn rate — no churn figure provided in any document), Claim 13 (Rails 7 version — no evidence document states the framework version), Claim 21 (code documentation quality — no evidence document addresses documentation). These are `[unverifiable]` — no evidence document addresses them. They are Tier 3–4 and their absence does not change the recommendation. All Tier 1 and Tier 2 claims were examined.

Consistency rate: of the 28 claims examined, 14 hold (8 `[real]`, 6 `[real, minor caveat]`), 6 are `[partial]`, 7 are `[delta]`, and 1 is `[derived]`. The consistency rate is 50%.

---

**What to ask the seller before closing**

1. Why have backups been failing for 21 days, and why was no alerting configured? When will the last good backup be replaced?
2. What is the plan for the 23 phantom accounts? Are they being pursued for payment, or are they to be written off?
3. What is the fallback if DataEnrich.io terminates the relationship on 90-day notice?
4. What is the deployment process in the absence of CI/CD and staging?
5. What is the contractual status of the 15 accounts with no signed ToS on file?
6. What is the plan if Google revokes the OAuth app?

---

=== LIMITATIONS ===

1. Materials examined: nine documents (doc1–doc9) provided in the data room, as of 2026-08-25. No source code was provided; findings rest on the documents as written.
2. The seller was not consulted and has not confirmed this reading of their own claims. Every finding interprets a claim without its author present to say what was meant.
3. This is a limited assurance engagement. The audit examined 28 of 32 claims. The conclusion applies only to the claims examined. It says nothing about the four unexamined claims or about any claim not in the claim sources.

---
