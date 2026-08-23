**AI-Readiness Audit — FlowMetrics**
**Prepared for:** Buyer's side
**Target:** FlowMetrics, B2B e-commerce analytics SaaS, offered at $480,000 (12× MRR)
**Data room:** 9 documents, all read in full
**Date:** 2026-08-23

---

## Recommendation: Material

Six deltas, one partial, and one derived finding with a live six-day expiry clock. The reliability and code-quality claims that form the core of the seller's technical narrative do not hold against the data room's own evidence. The MRR arithmetic is correct, but the account base is overstated by roughly 24%. A buyer at $480,000 is paying for a reliability story the data room contradicts on every axis it touches — monitoring, backup, redundancy, and testing. The gaps are individually fixable, but their collective pattern suggests the technical claims were written to a template rather than to the system actually in production. The buyer must price this in explicitly or walk.

**For a non-specialist reader:** The seller describes a well-run, well-monitored, well-tested software business. The data room shows a single-server application with no uptime checks, backups that stopped working three weeks ago with no one noticing, no automated testing on the payment path, and a customer list that includes 23 accounts that have not paid or logged in for months. The revenue number is real. The operational story behind it is not.

---

## Findings

**Finding 1: Backup expiry — [derived]**

Basis: doc4_infrastructure_config.md:18 — "Last Successful Backup: 2026-07-30"
       doc9_seller_s_technical_claims_verbatim.md:9 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. The last good backup falls out of Heroku's retention window on 2026-08-29, six days from the date of this report.

Consequence: If backup failures continue (no alerting is configured, doc4:19), the business cannot be restored to a state later than 2026-07-30. Any data loss or corruption event after that date is unrecoverable. For a buyer, this is not a maintenance issue — it is a point of no return that arrives during the diligence period itself.

Escalates: Finding 2.

**Finding 2: Backup system — [delta]**

Claim (doc9:9, doc2:3): "We maintain daily automated database backups" / "configured with daily scheduled backups to ensure data integrity and recoverability."

Evidence: doc4:15-19 — Schedule is daily at 2:00 AM, but "Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Delta: The backup schedule exists but has been failing for 21 consecutive days. No alerting means no one on the seller's side has been notified. The claim "we maintain daily automated database backups" is false in practice. The system is not maintaining backups.

**Finding 3: Uptime monitoring — [delta]**

Claim (doc9:7, doc1:6): "We monitor uptime at 99.9%."

Evidence: doc4:21-23 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Delta: No uptime monitor exists. The seller cannot be monitoring uptime at 99.9% because they are not monitoring uptime at all. The Heroku status page reports on Heroku's infrastructure, not on the application's availability, and it is not a monitoring tool the seller operates. The claim is false.

**Finding 4: Redundancy and failover — [delta]**

Claim (doc9:5, doc2:3, doc1:6): "The system has redundancy and automatic failover through Heroku's platform-level process management" / "platform-level redundancy and automatic failover."

Evidence: doc4:3-8 — Single standard-1x dyno (1 GB RAM, 0.5 CPU). Database (heroku-postgresql:standard-0) running on the same dyno as the application. No read replicas. No separate DB instance.

Delta: One dyno with a co-located database is not a redundant architecture. Heroku will restart a crashed dyno, which is process management, not architectural redundancy or failover. There is no secondary instance, no replica, no failover target. If the dyno and its database are simultaneously unavailable, there is nothing to fail over to. The claim conflates a platform's crash-recovery behaviour with the absence of single points of failure.

**Finding 5: Test coverage — [delta]**

Claim (doc9:8, doc1:6): "The codebase is well-documented with comprehensive test coverage across all critical paths."

Evidence: doc3:22-28 — 12 unit tests, all in test/utils/. 0 integration tests. 0 payment-path tests. No CI/CD pipeline. No staging environment. No branch protection. No code review process documented.

Delta: Twelve utility tests and zero tests on the payment path do not constitute "comprehensive test coverage across all critical paths." The payment path — the mechanism by which the business collects its entire $40,000 MRR — has no automated tests. The absence of CI/CD, staging, and branch protection means there is no gate between a code change and production. The claim is false.

**Finding 6: No lock-in — [delta]**

Claim (doc9:10, doc1:6): "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: doc8:7-12 — DataEnrich.io: 40% of features depend on this API. No fallback implemented. 90-day termination notice. doc8:29-32 — GoDaddy DNS managed personally by one individual, no secondary DNS provider.

Delta: A 40% feature dependency on a single external API with no fallback and a 90-day termination notice is lock-in by any operational definition. If DataEnrich raises pricing, changes its API, or terminates the relationship, the product degrades to "basic mode" (doc8:12) with no alternative. The GoDaddy dependency compounds the risk: a single person manages the DNS, and if that person is unavailable or the domain lapses, the application is unreachable. The claim is false.

**Finding 7: 120 active accounts — [partial]**

Claim (doc1:5): "A loyal customer base of 120 active accounts."

Evidence: doc6:3, 12-18 — 120 accounts marked active in CRM. 23 are phantom: created Q1 2026, no payment events in 90 days, last login 60–120 days ago, no notes on file. All 120 are still marked active.

Delta: 97 accounts are genuinely paying (94 Stripe-linked + 3 enterprise wire). 23 are not. The stated figure overstates the active base by approximately 24%. The CRM was not cleaned after those accounts went silent. This is a known-limitation issue rather than a broken promise, but it materially affects the revenue-per-account metric and the churn narrative.

**Finding 8: MRR $40,000 — [real]**

Claim (doc1:5): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000."

Evidence: doc5:5 — Stripe MRR: $16,000 (91 Pro × $149 = $13,549; 3 Enterprise Stripe × $817 = $2,451). doc5:19 and doc7:6, 14, 22 — 3 enterprise contracts at $8,000/mo each via wire transfer, not in Stripe. $16,000 + $24,000 = $40,000.

Delta: None. The arithmetic holds. The Stripe export and the contract summary are internally consistent and sum to the stated figure.

**Finding 9: Rails 7 stack — [real]**

Claim (doc9:11, doc2:3): "We use a modern Rails 7 stack."

Evidence: doc2:3 states Rails 7. doc4:4 confirms Heroku hosting, which is consistent with a Rails 7 deployment. No contradicting evidence in the data room.

Delta: None. Consistent across all available materials. (Note: I cannot open the application code to confirm the gem version; this is verified against the seller's own documentation and infrastructure description, not against the source.)

**Finding 10: Automatic SSL — [real]**

Claim (doc9:11): "automatic SSL."

Evidence: doc4:26 — "SSL: Heroku-managed, auto-renewed."

Delta: None.

**Finding 11: Managed DNS — [real, with a structural note]**

Claim (doc9:11): "managed DNS."

Evidence: doc4:27-28 — Provider: GoDaddy. Managed personally by 'dave'. No secondary DNS provider.

Delta: None on the claim itself — DNS is managed. The structural note: it is managed by a single individual with no secondary provider. This is a key-person risk that a buyer should plan for, and it is the same individual who accounts for 73% of commits (doc3:6). It does not change the verdict on the claim but does change the operational risk profile.

---

## Operational and security notes (not findings — no claim to test)

These are properties of the target that a buyer should know. They are not deltas because the seller did not claim otherwise.

**Secrets management.** 14 secrets (Stripe keys, DataEnrich API key, Twilio credentials, Google OAuth, DB URL, Redis URL, and others) are stored in Heroku config variables with no secrets vault (doc4:29). This is common at this scale but means credential rotation is a manual, dashboard-level operation with no audit trail.

**Authentication dependency.** Google OAuth is the sole authentication mechanism, on the free tier, with no contractual relationship (doc8:25-28). If Google revokes the app or changes the API, users cannot log in. There is no fallback authentication path.

**No CI/CD, no staging, no branch protection, no code review** (doc3:22-28). Any commit to the default branch can reach production. For a system processing $40,000/month in payments, this is a significant operational risk independent of the test-coverage delta above.

**Key-person concentration.** Dave: 73% of commits, sole DNS manager, sole operator of the backup schedule and the Heroku dashboard (doc3:6, doc4:28). Alice and Bob were one-time contractors. The operational knowledge is in one person.

---

## Coverage statement

**Claims identified:** ~30 across 9 documents.
**Claims individually verified:** 11 (Findings 1–11 above), covering all 7 verbatim technical claims in doc9 plus the financial and account claims in doc1.
**Not independently verified:**
- *Horizontal scalability* (doc9:6, doc2:3): "designed to scale horizontally" is a design claim that requires code access to verify (statelessness, connection pooling, session handling). The data room does not contain the source code. This is [unverifiable] from available materials.
- *Contract terms* (doc7): SLA percentages, termination clauses, and renewal dates are as stated by the seller. I have no independent access to the executed contracts. They are internally consistent with doc5 and doc6 but are taken at face value.
- *Dependency facts* (doc8): Costs, termination-notice periods, and risk assessments are as stated by the seller. No independent vendor confirmation.
- *Git history* (doc3): Commit counts and directory breakdowns are as summarised in the data room. I did not clone the repository.

**Why the remainder is or is not low-risk.** The unverified items are contract terms and dependency facts that the seller has stated, not technical claims about system behaviour. The technical claims — the ones a buyer would most want to be true — are the ones I verified, and they did not hold. The unverified items would, if they were deltas, be less expensive to a buyer than the six deltas already found. However, the contract terms (particularly the three enterprise SLAs at 99.9%/99.5%) are directly relevant to the uptime-monitoring delta: the seller is contractually obliged to meet an SLA they have no means of monitoring.

---

## What the client should ask the seller before closing

1. **Why have backups been failing for 21 days, and who was responsible for the alerting gap?** Request the Heroku backup log and the configuration that was supposed to trigger failure notifications.
2. **What is the current state of the last good backup as of today (2026-08-23)?** If 2026-07-30 is still the last good backup, the six-day window in Finding 1 is real. If a backup has succeeded since the data room was assembled, the finding is softened.
3. **Can you demonstrate the uptime monitoring you claim to operate?** If the answer is "Heroku's status page," that is not uptime monitoring of the application.
4. **What is the plan for the 23 phantom accounts?** Are they being pursued, written off, or simply not cleaned from the CRM? What is the actual active account count?
5. **What is the fallback plan if DataEnrich.io changes pricing or terminates the 90-day-notice agreement?** If the answer is "there isn't one," the lock-in finding stands and the buyer should price a 90-day migration risk.
6. **Can you walk through a production deployment?** Specifically: what prevents an unreviewed commit from reaching the payment path? (The data room says nothing.)
7. **What is Dave's role post-closing, and what is the key-person transition plan?** 73% of commits, sole DNS, sole operator.

---

*This report is a technical claims verification. It is not a penetration test, not a code-quality review, and not legal advice. The scope is stated claims versus observed implementation, with citations. All findings cite their source documents and line ranges. The audit recommends; the buyer decides.*

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS, $480K asking (12× MRR)

**Recommendation: MATERIAL**

Six deltas and one partial in the first eleven findings. The reliability and code-quality claims do not hold. The revenue number is real; the operational story behind it is not.

**Key items:**

⚠️ **Backups failing 21 days; last good backup expires in 6 days.** Daily backup schedule exists but has been failing since 2026-07-30. No alerting configured. The 30-day retention window closes 2026-08-29. After that date, the business cannot be restored.

⚠️ **No uptime monitoring despite 99.9% claim.** No Pingdom, no UptimeRobot, no custom checks. The Heroku status page monitors Heroku, not the application. The seller is contractually bound to 99.9% SLAs (three enterprise contracts) with no means of monitoring them.

⚠️ **Single dyno, co-located database, no redundancy.** "Platform-level redundancy and automatic failover" is a description of Heroku's crash-restart behaviour, not an architectural property. There is no second instance, no replica, no failover target.

⚠️ **23 of 120 "active" accounts are phantom.** No payment in 90 days, no login in 60–120 days, no notes on file. True active base: 97. Stated base overstates by ~24%.

⚠️ **40% feature dependency on DataEnrich.io, no fallback, 90-day termination notice.** "No lock-in" is false. If the API degrades or pricing changes, the product drops to basic mode with no alternative.

**Coverage:** 11 of ~30 claims individually verified, covering all 7 verbatim technical claims and the core financial/account claims. Remaining ~19 are contract terms, dependency facts, and design claims requiring code access or seller confirmation. The unverified items are less expensive to a buyer than the deltas already found, but the enterprise SLA terms are directly relevant to the uptime gap.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*