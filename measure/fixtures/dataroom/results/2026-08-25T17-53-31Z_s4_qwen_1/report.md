**Recommendation: Material**

Of the 28 claims examined, 9 are [delta] and 8 are [partial]. The target is not what the seller described in three specific, material ways: the backup system has been failing for 21 days with no alerting, the customer count is overstated by 23 accounts, and the uptime monitoring claim is false. These are not documentation drift; they are operational failures that change the risk profile of the acquisition. The buyer must price these in explicitly.

**Findings**

**Finding 1: Daily automated backups failing for 21 days — [delta]**

Claim (doc1:19; doc9:6; doc9:10): "Daily automated database backups with 30-day retention."

Evidence: doc4:16 — schedule is daily at 2AM. doc4:17 — failures recorded for the last 21 days. doc4:18 — last successful backup 2026-07-30. doc4:19 — no alerting on backup failures.

Gap: Backups are scheduled but not functioning. No mechanism exists to detect or alert on failure. The 30-day retention window is irrelevant if no new backup is being created.

**Finding 2: Last recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc1:19 — "30-day retention"
       doc4:18 — "Last Successful Backup: 2026-07-30"

Derivation: 2026-07-30 + 30 days = 2026-08-29. As of the as-of date of the materials (2026-08-25), the last recoverable backup will age out of the retention window in four days.

Consequence: After 2026-08-29, if no new backup succeeds, the business cannot be restored to any state after 2026-07-30. This is a hard deadline, not a risk to monitor.

Escalates: Finding 1.

**Finding 3: 120 active accounts — [delta]**

Claim (doc1:13): "Active Customers: 120."

Evidence: doc6:3 — 120 accounts marked active in CRM. doc6:12–16 — 23 accounts show no payment events in the last 90 days and no login activity in the last 60–120 days. doc6:6 — 94 accounts are Stripe-linked. doc6:7 — 3 accounts are wire-transfer. 94 + 3 = 97 genuinely active accounts.

Gap: 23 of the 120 accounts are not active. They are marked active in the CRM but show no payment or login activity. The active account count is 97, not 120.

**Finding 4: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "99.9% uptime monitoring."

Evidence: doc4:22–23 — no uptime monitor is configured. The only status visibility is Heroku's built-in status page, which reports platform health, not application-level uptime.

Gap: There is no uptime monitoring. The 99.9% figure appears in the enterprise contracts (doc7:8, doc7:16) as an SLA target, but no mechanism exists to measure or enforce it. The claim is false.

**Finding 5: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): "Well-documented codebase with comprehensive test coverage across critical paths."

Evidence: doc3:25–32 — 12 unit tests in test/utils/, 0 integration tests, 0 payment-path tests. No CI/CD pipeline. No staging environment. No branch protection. No code review process.

Gap: The payment path is a critical path and has zero tests. "Comprehensive test coverage across critical paths" is not supported by the materials.

**Finding 6: No proprietary lock-in — [partial]**

Claim (doc1:22; doc9:9): "All third-party integrations on standard SaaS agreements with no proprietary lock-in."

Evidence: doc8:7–12 — DataEnrich.io: 90-day termination notice, 40% of features depend on it, no fallback. doc8:25–28 — Google OAuth: no contractual relationship; if revoked, users cannot log in. doc4:27–28 — GoDaddy DNS managed personally by one individual, no secondary provider.

Gap: The agreements are standard SaaS, but the dependency structure creates practical lock-in. DataEnrich.io is a de facto single point of failure for 40% of features. The DNS is a single point of failure managed by one person. "No lock-in" is misleading.

**Finding 7: Platform-level redundancy and automatic failover — [partial]**

Claim (doc1:18; doc2:3; doc2:7; doc9:5): "Platform-level redundancy with automatic failover."

Evidence: doc4:5–8 — single standard-1x dyno. Database on the same dyno. No read replicas. No separate database instance.

Gap: Heroku provides dyno-level restart on crash, but the database is on the same dyno with no separate instance and no read replicas. The architecture is a single point of failure for the database. The claim overstates the redundancy.

**Finding 8: Low churn, stable base — [partial]**

Claim (doc1:14): "Churn: Low (stable base of recurring subscribers)."

Evidence: doc6:12–16 — 23 accounts with no payment in 90 days and no login in 60–120 days.

Gap: 23 accounts have gone dark. This contradicts "low churn, stable base." Historical churn data is not available in the materials to quantify the rate, but the 23 accounts are a specific, citable gap.

**Finding 9: No manual intervention required for routine operations — [partial]**

Claim (doc2:3): "No manual intervention required for routine operations."

Evidence: doc4:19 — no alerting on backup failures. doc4:28 — DNS managed personally by dave.

Gap: The 21-day backup failure went unnoticed because there is no alerting. Routine operations do require manual intervention when things break, and the current setup does not even detect the breakage.

**Finding 10: Blended MRR $40,000 — [real, minor caveat]**

Claim (doc1:12): "Blended MRR: $40,000."

Evidence: doc5:5 — Stripe MRR $16,000. doc5:19 — 3 enterprise companies at $8,000/month each via wire = $24,000. $16,000 + $24,000 = $40,000.

Gap: None on the figure. Caveat: $24,000 of the $40,000 (60%) is from 3 enterprise wire-transfer contracts, not from the SaaS subscription model. A buyer should know the concentration.

**Finding 11: Managed SSL and DNS — [real, operational caveat]**

Claim (doc1:20; doc9:11): "Automatic SSL and managed DNS."

Evidence: doc4:26 — SSL is Heroku-managed, auto-renewed. doc4:27–28 — DNS is GoDaddy, managed personally by dave, no secondary provider.

Gap: SSL is fully managed. DNS is managed but by a single individual with no secondary provider. Operational caveat: if dave is unavailable, DNS is at risk.

**Coverage statement**

28 claims were identified from the claim sources (doc1: 12, doc2: 8, doc9: 8). All 28 were examined. 9 are [delta], 8 are [partial], 6 are [real], 1 is [real, minor caveat], 2 are [real, operational caveat], and 2 are [unverifiable].

The 2 unverifiable claims are: "entire stack managed through Heroku's dashboard" (doc2:6) and "adding dynos is a one-click operation in the Heroku dashboard" (doc9:8). The materials confirm Heroku is the platform but do not document the dashboard management workflow or the specific dyno-scaling interface. These are platform capability claims that the data room does not settle. They are low-priority and do not affect the recommendation.

The audit stopped after examining all 28 claims. No claims remain unchecked.

**What the buyer should ask the seller before closing**

1. Why have the backups been failing for 21 days, and why was there no alerting? What is the plan to restore backup functionality before closing?
2. What is the status of the 23 accounts marked active but showing no payment or login activity? Are they being pursued for reactivation, or should the active count be restated?
3. What is the uptime monitoring plan? The enterprise contracts reference a 99.9% SLA, but no monitoring exists to measure it.
4. What is the test coverage plan for the payment path? Zero tests on a critical path is a material risk.
5. What is the fallback plan for DataEnrich.io if the 90-day termination notice is triggered? 40% of features depend on it.
6. Who manages DNS if dave is unavailable? Is there a secondary provider or a documented handover?

=== LIMITATIONS ===
Materials examined: data room documents doc1–doc9, as of 2026-08-25.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance engagement. Coverage: 28 of 28 claims examined. 2 claims are [unverifiable] from available materials.
