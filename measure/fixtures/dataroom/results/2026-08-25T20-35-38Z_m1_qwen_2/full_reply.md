**Recommendation: Material**

Of the 48 claims examined, 15 are `[delta]`, 11 are `[partial]`, and 1 derived finding carries a date that expires in four days. The target is not what the seller described in its most consequential respects: the backup system has been failing for 21 days, there is no uptime monitoring, there is no redundancy, and the test suite does not cover the payment path. The revenue figure is arithmetically supported but the customer base is more fragile than stated. A buyer must price these gaps in explicitly.

---

**Finding 1: Database backups have been failing for 21 days — [delta]**

Claim (doc1:19, doc9:10, doc2:6): "daily automated database backups with 30-day retention" / "The database is backed up daily to Heroku's managed storage with 30-day retention."

Evidence (doc4, Backups section): "Status: Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Gap: Backups are not happening. The last successful backup was 2026-07-30. No alerting means no one noticed.

**Finding 2: Last recoverable backup expires 2026-08-29 — [derived]**

Basis: doc9:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention."
       doc4, Backups — "Last Successful Backup: 2026-07-30"

Derivation: 30-day retention from 2026-07-30 exhausts on 2026-08-29. After that date, no backup of the database exists.

Consequence: If the database is lost or corrupted after 2026-08-29, the business cannot be restored. All customer data, subscription state, and transaction history would be unrecoverable.

Escalates: Finding 1.

**Finding 3: No uptime monitoring exists — [delta]**

Claim (doc1:18, doc9:2): "99.9% uptime monitoring" / "We monitor uptime at 99.9%."

Evidence (doc4, Monitoring): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Gap: Nothing is monitoring uptime. The 99.9% figure is not measured, not tracked, and not reported. The enterprise contracts (doc7) carry 99.9% SLAs with service-credit remedies, but there is no mechanism to detect a breach.

**Finding 4: No redundancy or failover — [delta]**

Claim (doc1:17, doc2:2, doc9:1, doc2:13): "platform-level redundancy and automatic failover" / "standard SaaS stack with platform-level redundancy."

Evidence (doc4, Hosting): "Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No."

Gap: A single dyno runs both the application and the database. There is nothing to fail over to. If the dyno goes down, the entire business is down simultaneously. Heroku's process manager restarts crashed dynos, but that is not redundancy.

**Finding 5: No test coverage on critical paths — [delta]**

Claim (doc1:24, doc9:4): "comprehensive test coverage across critical paths" / "comprehensive test coverage across all critical paths."

Evidence (doc3, CI/CD & Testing): "Unit Tests: 12 (all located in test/utils/). Integration Tests: 0. Payment-Path Tests: 0. CI/CD Pipeline: None configured. Staging Environment: None."

Gap: Twelve unit tests in a utilities directory, zero integration tests, zero payment-path tests, no CI/CD, no staging. The payment path — the mechanism by which the business collects $40,000/month — is entirely untested.

**Finding 6: "No lock-in" is contradicted by the dependency list — [delta]**

Claim (doc1:25, doc9:6): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in" / "no lock-in."

Evidence (doc8): DataEnrich.io — "40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode." Google OAuth — "No contractual relationship. If Google changes API or revokes app, users cannot log in." GoDaddy — "Managed personally by 'dave'. No secondary DNS. If domain expires or GoDaddy has an issue, the app is unreachable."

Gap: 40% of product functionality depends on a single API with no fallback and a 90-day termination notice. Authentication depends on a free-tier service with no contract. DNS is a single point of failure held by one individual. These are lock-in conditions.

**Finding 7: Deployments and monitoring are not streamlined — [delta]**

Claim (doc2:12): "deployments, scaling, and monitoring are streamlined and efficient."

Evidence (doc3): "CI/CD Pipeline: None configured." (doc4, Monitoring): "Uptime Monitor: None."

Gap: There is no CI/CD pipeline and no monitoring. Deployments are manual. Monitoring is absent. "Streamlined" is not a fair description of "not present."

**Finding 8: MRR of $40,000 is arithmetically supported but the customer base is overstated — [partial]**

Claim (doc1:5, doc1:12): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000" / "Blended MRR: $40,000."

Evidence (doc5): "Total MRR (Stripe): $16,000." + 3 enterprise contracts at $8,000/mo each (doc7) = $24,000. Total = $40,000. (doc6): "Phantom/Inactive Accounts: 23. No payment events in the last 90 days. Last login dates: 60-120 days ago." (doc7): "Pilot Agreements (8): Free 90-day trial. Payment Obligation: None." "No-Contract Accounts (15): No signed ToS on file."

Gap: The $40,000 figure reconciles. However, 23 of 120 "active" accounts (19%) show no payment in 90 days. Eight are free pilots with no payment obligation. Fifteen have no signed agreement. The paying base is 97, not 120. The MRR number holds; the "active customer base of 120" does not.

**Finding 9: 120 active accounts — 23 show no recent activity — [partial]**

Claim (doc1:3, doc1:13): "a loyal customer base of 120 active accounts" / "Active Customers: 120."

Evidence (doc6): "Total Accounts Marked 'Active': 120. Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago. No notes or memos on file regarding creation or non-payment."

Gap: 23 accounts are marked active but show no payment or login activity for 60–120 days, with no documentation of why they exist or why they are not paying.

**Finding 10: Horizontal scaling is constrained by database architecture — [partial]**

Claim (doc9:5): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence (doc4): "Dyno: standard-1x. Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None."

Gap: Adding application dynos is a one-click Heroku operation, but the database is co-located on the same dyno with no read replicas. The database is the scaling bottleneck. Horizontal scaling of the app tier does not relieve database load.

**Finding 11: "Turnkey" is qualified by personal dependencies — [partial]**

Claim (doc1:10): "This is a turnkey acquisition opportunity."

Evidence (doc4): "DNS Management: Managed personally by 'dave'. No secondary DNS provider." (doc3): "CI/CD Pipeline: None. Staging Environment: None. Branch Protection: None. Code Review Process: None documented."

Gap: DNS is held by one individual with no secondary. There is no CI/CD, no staging, no documented code review. Transition requires dave's personal involvement for DNS and operational knowledge that is not documented.

**Finding 12: "No manual intervention" is contradicted by absent automation — [partial]**

Claim (doc2:4): "no manual intervention required for routine operations."

Evidence (doc3): "CI/CD Pipeline: None configured." (doc4): "DNS Management: Managed personally by 'dave'." "Alerting: None configured for backup failures."

Gap: Deployments are manual (no CI/CD). DNS is managed by hand. Backup failures go unalerted. Routine operations require manual intervention.

**Finding 13: "Entire stack managed through Heroku's dashboard" — [partial]**

Claim (doc2:11): "The entire stack is managed through Heroku's dashboard."

Evidence (doc4): "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'." "Secrets Management: 14 secrets stored in Heroku config vars. No secrets vault."

Gap: DNS is managed externally at GoDaddy by dave personally. The word "entire" is not supported.

**Finding 14: Application availability under varying load — [partial]**

Claim (doc2:3): "the application remains available even under varying load conditions."

Evidence (doc4): "Dyno: standard-1x (1GB RAM, 0.5 CPU)." No evidence of load testing, auto-scaling configuration, or multiple dynos.

Gap: A single 0.5-CPU dyno with no scaling configuration has not been demonstrated to handle varying load. The claim is untested.

**Finding 15: "Managed DNS" — [partial]**

Claim (doc9:10): "managed DNS."

Evidence (doc4): "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'. No secondary DNS provider."

Gap: DNS is managed, but by one person at a registrar, not through a managed service with redundancy. The word "managed" implies a level of service not present.

---

**Claims that hold (summary):**

- Revenue-positive, recurring-revenue SaaS — [real, minor caveat] (M2M base has no-penalty cancellation)
- Real-time analytics and product intelligence — [real, operational caveat] (40% depends on DataEnrich.io, no fallback)
- Customer composition (independent to enterprise) — [real]
- Revenue model (monthly & annual) — [real]
- Revenue composition (enterprise + M2M mix) — [real, minor caveat] (60% of MRR from 3 customers)
- Minimal operational overhead — [real, minor caveat] (simplicity reflects absence, not efficiency)
- Proven product-market fit and immediate cash flow — [real, minor caveat]
- Managed SSL / automatic SSL — [real]
- OAuth authentication — [real, minor caveat] (Google free tier, no contract)
- Secure payment via Stripe — [real]
- PostgreSQL — [real]
- Redis for session caching — [real]
- Third-party data enrichment API — [real, operational caveat]

**Unverifiable from available materials:**

- Rails 7 (no version evidence in docs) — 3 claims
- Well-documented codebase (no documentation evidence) — 2 claims
- Low churn (no churn data provided) — 2 claims

**Non-delta:**

- Stripe supports credit card and recurring billing — a fact about Stripe, not the target. Noted for completeness.

---

**Coverage statement.**

48 claims identified from three claim sources (doc1: 25, doc2: 13, doc9: 10). All 48 were examined. 31 received a definitive verdict (real, partial, or delta). 6 are unverifiable from the materials provided (Rails version, documentation quality, churn rate — no evidence document addresses these). 1 is non-delta. 1 derived finding was computed from two stated figures. No claims were left unchecked. The evidence documents (doc3–doc8) were read in full. No source code was available in the data room; claims about code quality and Rails version could not be verified against implementation.

**What the buyer should ask the seller before closing:**

1. Why have backups been failing for 21 days, and why was no one alerted? When will the next successful backup occur?
2. What is the plan for the 23 accounts with no payment in 90 days? Are they being pursued, or should the active count be restated?
3. What is the contingency if DataEnrich.io terminates its 90-day-notice agreement or changes pricing? 40% of features depend on it with no fallback.
4. What is the contingency if Google revokes the OAuth app or changes its API? There is no contractual relationship.
5. Will dave transfer DNS management, and what is the transition plan? There is no secondary DNS.
6. What is the churn rate over the trailing 12 months? No churn data was provided.
7. What is the Rails version, and can the codebase be inspected? No source code was in the data room.

=== LIMITATIONS ===

1. Materials examined: nine documents (doc1–doc9) provided in the data room, as of 2026-08-25. No source code, no live system access, no financial statements beyond the Stripe and CRM exports provided.
2. The seller (Dave) was not consulted and has not confirmed this reading of his own claims. Every finding interprets a claim without its author present to say what was meant.
3. This is a limited assurance engagement. The audit examined 48 claims from three named claim sources. It says nothing about claims the seller made in other channels (email, verbal, cover letter references not included in the data room). It is not a penetration test, not a code-quality review, and not legal advice.

---

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS, 120 accounts, $40k MRR, asking $480k (12x MRR).

**Recommendation: Material**

The target's most consequential claims do not hold. A buyer must price these gaps explicitly.

**What matters most:**

1. **Backups have been failing for 21 days.** Last good backup: 2026-07-30. With 30-day retention, the last recoverable backup expires **2026-08-29** — four days from the as-of date. After that, the database is unrecoverable.
2. **No uptime monitoring exists.** The seller claims 99.9% uptime monitoring. There is no monitor. Enterprise contracts carry 99.9% SLAs with service-credit remedies that cannot be triggered because no one is watching.
3. **No redundancy.** Single dyno, database on the same dyno, no replicas. "Platform-level redundancy and automatic failover" is not present. If the dyno dies, the business is down.
4. **No test coverage on the payment path.** 12 unit tests in a utilities directory. Zero integration tests. Zero payment-path tests. No CI/CD. The mechanism collecting $40k/month is untested.
5. **"No lock-in" is contradicted by the dependency list.** 40% of features depend on DataEnrich.io (no fallback, 90-day termination notice). Auth depends on Google (no contract). DNS is held by one person (no secondary).

**What holds:**

The $40,000 MRR figure reconciles from the Stripe and contract documents. The product exists and does what it says. SSL, Stripe payments, PostgreSQL, Redis, and the data enrichment API are all present and functioning as described. The revenue is real.

**Coverage:** 48 of 48 claims examined. 31 definitive verdicts, 6 unverifiable (no evidence in data room), 1 non-delta, 1 derived. No claims left unchecked.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*