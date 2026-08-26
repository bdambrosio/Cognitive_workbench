**Recommendation: Material**

The data room's marketing claims (doc1, doc2, doc9) are contradicted by the seller's own factual disclosures (doc3–doc8) on five material points. The backup system has failed for 21 consecutive days with the last recoverable backup expiring on 2026-08-29. The claimed platform redundancy does not exist. Uptime monitoring is absent. Test coverage on critical paths is zero. The active-customer count is inflated by 19%. A buyer must price these gaps explicitly or walk.

---

**Finding 1: Last recoverable backup expires 2026-08-29 — [derived]**

Basis: doc9:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention."
       doc4:18 — "Last Successful Backup: 2026-07-30"

Derivation: 2026-07-30 + 30 days = 2026-08-29. As of the data room's as-of date (2026-08-25), the last recoverable backup ages out in 4 days.

Consequence: After 2026-08-29, the business has no recoverable database backup. Any data loss event after that date is unrecoverable. This is a Tier 1 consequence: data that cannot be recovered.

Escalates: Finding 2.

---

**Finding 2: Daily automated database backups — [delta]**

Claim (doc1:19; doc9:6, 10): "Daily automated database backups with 30-day retention."

Evidence: doc4:16–19 — Backups scheduled daily at 2:00 AM via `heroku pg:backups schedule`. Failures recorded for the last 21 days. Last successful backup: 2026-07-30. No alerting configured for backup failures.

Delta: The backups are not happening daily. They have failed for 21 consecutive days. The claim of "daily automated" backups is false for the current operational state.

---

**Finding 3: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18; doc9:5): "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: doc4:5–8 — Single standard-1x dyno (1 GB RAM, 0.5 CPU). Database (heroku-postgresql:standard-0) running on the same dyno as the application. No read replicas. No separate DB instance.

Delta: There is no redundancy. A single dyno with co-located application and database is a single point of failure. Heroku's process management restarts crashed processes but does not provide redundancy; if the host fails, both app and database are down. The claim is false for this configuration.

---

**Finding 4: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "We monitor uptime at 99.9%."

Evidence: doc4:22–23 — No uptime monitor configured. No Pingdom, UptimeRobot, or custom checks. Only Heroku's built-in status page.

Delta: The seller claims to monitor uptime at 99.9% but has no uptime monitoring tool. You cannot monitor what you are not measuring. The claim is false.

---

**Finding 5: Comprehensive test coverage across all critical paths — [delta]**

Claim (doc1:21; doc9:7): "The codebase is well-documented with comprehensive test coverage across all critical paths."

Evidence: doc3:24–30 — 12 unit tests, all in test/utils/. 0 integration tests. 0 payment-path tests. No CI/CD pipeline. No staging environment.

Delta: "Comprehensive test coverage across all critical paths" is false. There are 12 utility tests and zero tests on any critical path. The payment path — the most critical path in a SaaS business — has zero test coverage.

---

**Finding 6: 120 active customers — [partial]**

Claim (doc1:9, 13; doc6:3): "120 active customers."

Evidence: doc6:6–18 — CRM shows 120 total marked active. Breakdown: 94 Stripe-linked + 3 enterprise wire + 23 phantom/inactive. The 23 phantom accounts: created Q1 2026, no payment events in last 90 days, last login 60–120 days ago, no notes on file. All 120 marked 'active' in CRM.

Delta: Only 97 accounts are genuinely active (94 Stripe + 3 enterprise). 23 accounts are marked active but show no payment or login activity. A buyer relying on 120 active customers is overcounting by approximately 19%.

---

**Finding 7: Stack designed to scale horizontally — [partial]**

Claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: doc4:5–8 — Single standard-1x dyno. Database on the same dyno. No separate DB instance.

Delta: Adding application dynos is possible via Heroku's dashboard, but the database does not scale horizontally because it is co-located on the same dyno. The claim is misleading for this architecture.

---

**Finding 8: No lock-in on third-party integrations — [partial]**

Claim (doc1:22; doc9:9): "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: doc8:10–14 — DataEnrich.io: 40% of features depend on this API, 90-day termination notice, no fallback implemented. If the API is down or pricing changes, product degrades to basic mode. Google OAuth: no contractual relationship; if Google revokes the app, users cannot log in.

Delta: The agreements are standard SaaS, but "no lock-in" is false for DataEnrich.io (40% feature dependency, no fallback) and Google OAuth (authentication dependency with no contractual relationship).

---

**Finding 9: Pro-tier MRR arithmetic — [partial]**

Claim (doc5:9–11): "91 accounts at $149/mo, Total MRR: $13,549."

Evidence: 91 × $149 = $13,559. Doc5 states $13,549.

Delta: A $10 discrepancy. Possibly one account is on a different rate or there is a rounding issue, but the stated arithmetic does not match.

---

**Finding 10: OAuth authentication — [real, minor caveat]**

Claim (doc1:20; doc8:6): "OAuth authentication."

Evidence: doc8:6–8 — Google OAuth on free tier, no contractual relationship.

Delta: None. OAuth is implemented via Google. Caveat: free tier with no contractual relationship; if Google revokes the app, users cannot log in.

---

**Coverage statement**

67 claims identified across doc1 (marketing), doc2 (technical spec), doc9 (verbatim claims), and the factual disclosures in doc3–doc8. All 67 were examined. 5 deltas, 4 partials, 1 derived, 2 unverifiable, 1 real with minor caveat, 54 real.

The two unverifiable claims: "low churn" (no churn data in the data room) and "well-documented codebase" (codebase not included in the data room; absence of code review, branch protection, and CI/CD is consistent with poor documentation but cannot be confirmed from available materials).

**Limitation of this engagement:** The data room is a set of seller-provided documents. There is no independent codebase to inspect. Verification is cross-referencing the seller's marketing claims against the seller's own factual disclosures. This audit verifies internal consistency of the data room, not the data room against ground truth. A buyer should request access to the production environment, Heroku dashboard, Stripe dashboard, and CRM to confirm the factual disclosures are accurate.

---

**What the client should ask the seller before closing**

1. Why have backups failed for 21 consecutive days, and what is the remediation plan? When was the last successful backup, and can a current backup be produced on demand?
2. What is the plan for the 23 phantom accounts? Are they to be removed from the active count, and what is the true active-customer number?
3. What is the redundancy plan? Is there a separate database instance, or is the single-dyno configuration intentional and accepted as a risk?
4. What uptime monitoring is in place, and what is the actual uptime over the last 90 days?
5. What is the test coverage plan for the payment path, and when will integration tests be implemented?
6. What is the fallback plan if DataEnrich.io terminates or changes pricing?
7. Can the seller produce a current Stripe dashboard export and CRM export to confirm the figures in doc5 and doc6?

---

=== LIMITATIONS ===

Materials examined: nine seller-provided documents (doc1–doc9) from the flowmetrics data room, as-of 2026-08-25. The seller was not consulted and has not confirmed the auditor's reading of their own claims. This is limited assurance over the disclosed subset of the claim surface; the conclusion is expressed positively over that subset only. Of the 67 claims verified, 54 hold, 5 are deltas, 4 are partial, 1 is derived, 2 are unverifiable, and 1 carries a minor caveat.
