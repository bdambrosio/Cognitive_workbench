**AI-Readiness Audit — FlowMetrics**
Date: 2026-08-23
Auditor: Jill
Target: FlowMetrics — B2B E-Commerce Analytics Platform, offered by Dave
Claim surface: 191 micro-claims across 9 data-room documents

---

**Recommendation: Material**

Five deltas and one derived finding significantly change the risk profile of this acquisition. The stated backup, monitoring, test-coverage, account, and redundancy claims are contradicted by the seller's own data-room materials. The $480,000 asking price (12× MRR) is set against a revenue base that is 60% concentrated in three wire-transfer contracts with SLA obligations the current infrastructure cannot demonstrably meet.

---

**Findings (worst first)**

**Finding 1: Daily automated backups with 30-day retention — [delta]**

Claim (doc1, Technical Highlights – Reliability; doc9, line 6): "daily automated database backups with 30-day retention" / "The database is backed up daily to Heroku's managed storage with 30-day retention."

Evidence (doc4, Backups section): Schedule is daily at 2:00 AM via `heroku pg:backups schedule`. Status: "Failures recorded for the last 21 days." Last Successful Backup: 2026-07-30. Alerting: "None configured for backup failures."

Delta: Backups have been failing continuously for 21 days. The last successful backup is 24 days old as of this audit. No alerting means the failure was undetected. The claimed daily cadence is not occurring.

**Finding 2: 99.9% uptime monitoring — [delta]**

Claim (doc1, Technical Highlights – Reliability; doc9, line 2): "99.9% uptime monitoring" / "We monitor uptime at 99.9%."

Evidence (doc4, Monitoring section): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." Status Page: "Heroku built-in status page only."

Delta: No uptime monitoring of any kind exists. The 99.9% figure is not measured, tracked, or reported. The claim describes a capability that is not implemented.

**Finding 3: Comprehensive test coverage across all critical paths — [delta]**

Claim (doc1, Technical Highlights – Code Quality; doc9, line 3): "comprehensive test coverage across critical paths" / "comprehensive test coverage across all critical paths."

Evidence (doc3, CI/CD & Testing section): Unit Tests: 12 (all located in `test/utils/`). Integration Tests: 0. Payment-Path Tests: 0. CI/CD Pipeline: None configured. Staging Environment: None.

Delta: 12 unit tests in a single utility directory do not constitute comprehensive coverage. Zero integration tests and zero payment-path tests mean the Stripe billing flow, the wire-transfer reconciliation, and the DataEnrich.io API integration are entirely untested. The absence of CI/CD and staging means no test runs at all in any automated pipeline.

**Finding 4: 120 active accounts — [partial]**

Claim (doc1, About the Business / Key Financials): "120 active accounts" / "Active Customers: 120"

Evidence (doc6, CRM Export Summary): Total Accounts Marked 'Active': 120. Breakdown: 94 Stripe-linked, 3 enterprise (wire), 23 Phantom/Inactive. Phantom accounts: "Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60–120 days ago. No notes or memos on file regarding creation or non-payment."

Delta: 23 of the 120 accounts (19%) show no payment in 90 days, no meaningful login, and no documentation of their creation. They are marked active in the CRM with no supporting record. The functional active base is 97, not 120.

**Finding 5: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1, Technical Highlights – Scalable Architecture; doc9, line 1): "platform-level redundancy and automatic failover" / "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence (doc4, Hosting section): Dyno: `standard-1x` (1GB RAM, 0.5 CPU). Database: `heroku-postgresql:standard-0` (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No.

Delta: A single dyno with the database co-located on the same process has no redundancy and no failover path. If the dyno crashes, the application and the database are both unavailable simultaneously. Heroku's platform guarantees process restart, not data redundancy or zero-downtime failover. The claim conflates process management with redundancy.

**Finding 6: Backup retention window exhaustion — [derived]**

Basis: doc4, Backups section — "Last Successful Backup: 2026-07-30"
       doc1, Technical Highlights – Reliability; doc9, line 6 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. As of this audit (2026-08-23), the last recoverable backup state expires in 6 days. After that date, no backup exists within the stated retention window.

Consequence: If no backup succeeds between now and 2026-08-29, the business will have no recoverable data state at all. Combined with the absence of alerting (doc4: "None configured for backup failures"), there is no mechanism to detect or prevent this transition. A buyer closing after 2026-08-29 without a confirmed backup restoration would be acquiring a business with no demonstrated data-recovery capability.

Escalates: Finding 1

**Finding 7: Blended MRR $40,000 — [real, with a structural note]**

Claim (doc1, Key Financials): "Blended MRR: $40,000"

Evidence (doc5, Stripe Export Summary): Total Stripe MRR: $16,000 ($13,549 Pro + $2,451 Enterprise Stripe). doc7, Customer Contracts: 3 enterprise contracts at $8,000/mo each = $24,000/mo via wire transfer. $16,000 + $24,000 = $40,000.

Delta: None — the arithmetic is consistent.

Structural note: 60% of MRR ($24,000) is concentrated in three wire-transfer contracts with explicit SLA obligations (99.9% for Acme and GlobalMart, 99.5% for ShopStream) and termination-for-breach clauses (30-day notice). The remaining 40% ($16,000) is 94 month-to-month Stripe subscriptions with no SLA and no-penalty cancellation. The revenue is real but its risk profile is not what "recurring subscription" implies: the majority is annual-contract enterprise revenue with service-level obligations the current single-dyno, unmonitored, untested infrastructure is not demonstrably equipped to meet.

**Finding 8: Low churn, stable base — [real, operational caveat]**

Claim (doc1, Key Financials): "Churn: Low (stable base of recurring subscribers)"

Evidence (doc6): 23 phantom accounts created in Q1 2026 with no payment in 90 days. doc7: 94 month-to-month subscriptions with no-penalty cancellation at end of billing cycle. 8 pilot agreements expiring Q3 2026 with no payment obligation.

Delta: None on the Stripe base — the 94 subscriptions are active in Stripe. Operational caveat: the 23 phantom accounts represent acquisition-channel leakage that is not reflected in churn metrics if they were never billed. The 8 pilots expiring this quarter add conversion uncertainty. "Low churn" is accurate for the paying base but does not capture the top-of-funnel attrition.

**Finding 9: Scalable technology stack / one-click horizontal scaling — [real, minor caveat]**

Claim (doc9, line 4): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence (doc4): Single standard-1x dyno. Database co-located on same dyno.

Delta: None — Heroku does allow adding dynos. Minor caveat: the database is not horizontally scalable in this configuration (single standard-0, no replicas). Scaling the app tier does not scale the data tier. The claim is technically true for the app process but misleading about the full stack.

**Finding 10: Low vendor lock-in / standard SaaS agreements — [real, minor caveat]**

Claim (doc1, Technical Highlights – Low Vendor Lock-in; doc9, line 5): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."

Evidence (doc8, External Dependency List): DataEnrich.io — 40% of features depend on this API, 90-day termination notice, no fallback implemented. Google OAuth — no contractual relationship. GoDaddy DNS — managed personally by dave, no secondary.

Delta: None — the agreements are standard SaaS. Minor caveat: "no lock-in" is contradicted in practice by the DataEnrich.io dependency. 40% of product features require that API, and no fallback is implemented. If DataEnrich.io raises pricing, changes terms, or goes out of business, the product degrades to "basic mode (no enrichment)" per the seller's own dependency list. That is functional lock-in even if the contract is standard.

**Finding 11: Managed DNS / no single point of failure — [non-delta]**

Claim (doc9, line 7): "We use a modern Rails 7 stack with automatic SSL and managed DNS."

Evidence (doc4, Security & DNS): DNS Provider: GoDaddy. DNS Management: "Managed personally by 'dave'. No secondary DNS provider."

Delta: No claim is made that DNS is redundant. The seller states "managed DNS," which is accurate — dave manages it. However, the absence of a secondary DNS provider and the personal (not organizational) management create a key-person and single-point-of-failure risk that no document claims away. Noted for completeness, not a finding against a stated claim.

**Finding 12: Key-person concentration — [non-delta]**

Claim: No document claims team redundancy or institutional knowledge distribution.

Evidence (doc3): dave: 618 commits (73%). Last commit: 2026-07-30. Commits in last 30 days: 0. doc4: DNS managed personally by dave. doc8: GoDaddy DNS risk — "Managed personally by 'dave'."

Delta: No claim to verify. Noted because the buyer should be aware: 73% of code is attributable to one person who has not committed in 30 days, and the DNS is in his personal account. This is an operational risk with no corresponding seller claim to contradict, so it is a due-diligence item rather than a finding.

**Finding 13: 15 accounts with no signed ToS — [non-delta]**

Claim: No document claims all accounts are under contract.

Evidence (doc7, No-Contract Accounts): 15 individual users on Pro plan, paid via Stripe, "No signed ToS on file (acceptance not recorded)."

Delta: No claim to verify. Noted because 15 of 94 Stripe accounts (16%) lack a recorded legal agreement. This is a minor legal-hygiene gap, not a contradiction of a stated claim.

---

**Coverage statement**

191 micro-claims were enumerated across the nine data-room documents. 13 findings are reported above, covering all five priority tiers: safety-critical (backup, uptime), architectural (redundancy, test coverage, MRR composition), operational (account base, churn, scaling), and micro-claims (vendor lock-in, DNS, key-person, no-contract accounts). Claims not individually verified include: the specific feature descriptions ("real-time inventory sync," "category classification," "API access" required by enterprise contracts) — these cannot be verified without access to the running application or codebase, and the data room does not include source code. The git history is a summary provided by the seller, not a raw log. The Stripe and CRM exports are seller-provided summaries, not raw API pulls. These limitations mean the audit verifies internal consistency and claim-vs-observed-state within the data room, but does not independently confirm that the running system behaves as the documents describe. For a $480,000 acquisition, the buyer should request: (a) raw Stripe and CRM API exports, (b) access to the Heroku dashboard to confirm dyno, backup, and monitoring state, (c) a code repository clone for test-suite verification, and (d) a backup-restore test performed before closing.

---

**What the client should ask the seller before closing**

1. Restore the most recent backup (2026-07-30) to a staging instance and confirm data integrity. If this fails, the backup claim is not merely stale — it is non-functional.
2. Explain the 21-day backup failure: what changed, why no alert fired, and what the remediation plan is. Will the buyer inherit a working backup or a broken one?
3. Account for the 23 phantom accounts: how were they created, why is there no documentation, and should they be excluded from the active-customer count in the valuation?
4. Confirm whether the three enterprise contracts' SLA obligations (99.9%/99.5%) have ever been measured or breached. If uptime has never been monitored, has any SLA remedy been triggered?
5. What is the plan for the DataEnrich.io dependency if pricing changes or the service is discontinued? Has a fallback been scoped?
6. Is dave available for a transition period? What is the plan for DNS ownership transfer (currently personal account)?
7. The 8 pilot agreements expiring Q3 2026: what is the conversion rate historically, and is any revenue from them included in the $40,000 MRR?

---

**Executive synthesis**

The business is real: the revenue is arithmetically consistent, the customers are paying, and the product exists. But the reliability and quality claims in the listing are not supported by the seller's own data room. Backups have been failing for three weeks with no one noticing, there is no uptime monitoring despite a 99.9% claim, the test suite is 12 utility functions, and the "redundant" infrastructure is one process with the database inside it. The $480,000 price is set on a revenue base where 60% carries SLA obligations this infrastructure is not demonstrably equipped to meet. A buyer should not close without a confirmed backup restore, a working monitoring setup, and a clear answer on the 23 unexplained accounts.

---

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS, 120 accounts, $40k MRR, asking $480k

**Recommendation: Material**

| # | Item | Note |
|---|------|------|
| 1 | Backups failing 21 days; last good backup expires 2026-08-29 | No alerting. After Aug 29, no recoverable state exists. |
| 2 | 99.9% uptime claimed; no monitor exists | SLA contracts (3 × $8k/mo) obligate 99.9%/99.5%. Unmeasured. |
| 3 | "Comprehensive test coverage" claimed; 12 unit tests, 0 integration | Payment path, API integration, and billing flow are untested. |
| 4 | 120 active accounts claimed; 23 are phantom | No payment 90 days, no login, no documentation. Functional base: 97. |
| 5 | "Platform-level redundancy" claimed; single dyno, DB co-located | No replicas, no separate DB, no failover path. |
| 6 | MRR $40k verified ($16k Stripe + $24k wire) | Arithmetically consistent. 60% in 3 annual contracts with SLA. |

**Coverage:** 191 claims enumerated; 13 findings reported across all 4 priority tiers. Source code and running system not available in data room — claims verified against seller-provided documents only.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*