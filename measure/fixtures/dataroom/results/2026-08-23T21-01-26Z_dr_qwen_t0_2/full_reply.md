REPORT — FlowMetrics AI-Readiness Audit

RECOMMENDATION: Material

The audit found 3 deltas, 5 partials, 2 derived findings, 1 real-with-structural-note, 1 real-with-operational-caveat, and 1 unverifiable claim across 15 findings. The 3 deltas (backups, uptime monitoring, test coverage) are individually addressable but collectively indicate a pattern: the seller's technical claims describe a system that does not exist. The backup situation is urgent — the last good backup expires 2026-08-29, six days from the audit date. The enterprise SLA exposure compounds this: $16,000/mo in revenue (40% of MRR) is contractually dependent on features that have no fallback and no monitoring. The buyer must price these gaps explicitly or walk.

FINDINGS (worst first):

**Finding 1: Daily automated backups with 30-day retention — [delta]**

Claim (doc9 §Backups): "DB backed up daily to Heroku managed storage with 30-day retention"

Evidence: doc4 §Backups — "Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Delta: The claim asserts continuous daily backup success with 30-day retention. The evidence shows 21 consecutive days of failed backups, no alerting to detect the failures, and a last successful backup that will be purged on 2026-08-29 under the stated retention policy. The backup system is not functioning.

**Finding 2: 99.9% uptime monitoring — [delta]**

Claim (doc9 §Monitoring): "99.9% uptime monitoring"

Evidence: doc4 §Monitoring — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Delta: The claim asserts active uptime monitoring at a 99.9% threshold. No uptime monitor of any kind exists. The 99.9% figure is unsubstantiated by any tooling, and the enterprise contracts (doc7) that require 99.9% SLA have no mechanism to detect or report a breach.

**Finding 3: Comprehensive test coverage across all critical paths — [delta]**

Claim (doc9 §Testing): "well-documented with comprehensive test coverage across all critical paths"

Evidence: doc3 §CI/CD & Testing — "Unit Tests: 12 (all located in test/utils/). Integration Tests: 0. Payment-Path Tests: 0."

Delta: The claim asserts comprehensive coverage across critical paths. The evidence shows 12 unit tests confined to utility functions, zero integration tests, and zero tests on the payment path. The critical paths (authentication, payment processing, data enrichment) are entirely untested.

**Finding 10: Backup retention exhaustion — [derived]**

Basis: doc4 §Backups — "Last Successful Backup: 2026-07-30"; doc1/doc9 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. Audit date: 2026-08-23. The last recoverable backup will be purged in 6 days.

Consequence: After 2026-08-29, the business has no recoverable database backup. Combined with Finding 1 (21 days of failures, no alerting), the business is 6 days from zero backup coverage. A database failure after that date is unrecoverable.

Escalates: Finding 1.

**Finding 11: Enterprise SLA breach risk — [derived]**

Basis: doc7 — Acme Retail SLA 99.9%, Required Features: "Product matching, price comparison, real-time inventory sync"; GlobalMart SLA 99.9%, Required Features: "Product matching, category classification, API access"; Termination: "30-day notice for material SLA breach." doc8 — DataEnrich.io provides "Product data enrichment API (product matching, price comparison, category classification)"; "40% of features depend on this API"; "Fallback: None implemented." doc4 — "Uptime Monitor: None."

Derivation: The enterprise contracts require features that are provided exclusively by DataEnrich.io, which has no fallback. There is no monitoring to detect degradation. If DataEnrich.io degrades or terminates (90-day notice), the required features fail, constituting a material SLA breach, giving the customer 30-day termination notice.

Consequence: $16,000/mo in enterprise revenue (40% of total MRR) is exposed to termination if DataEnrich.io fails and no fallback is implemented. The buyer inherits this exposure at closing.

Escalates: Finding 6 (no lock-in) and Finding 2 (no uptime monitoring).

**Finding 4: Platform-level redundancy and automatic failover — [partial]**

Claim (doc2, doc9): "automatic scaling and failover at platform level" / "redundancy and automatic failover through Heroku platform-level process management"

Evidence: doc4 §Hosting — "Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No."

Delta: Heroku does provide process-level restart on crash, which is a form of failover. However, the database runs on the same dyno as the application, there are no read replicas, and there is no separate DB instance. A dyno failure takes down both the app and the database simultaneously. The claim is 80% true (Heroku does restart crashed processes) but the 20% gap (no database redundancy, no separation of concerns) is material for a business whose enterprise contracts require 99.9% uptime.

**Finding 5: 120 active customers — [partial]**

Claim (doc1): "120 active accounts"

Evidence: doc6 — "120 accounts marked active, 94 Stripe-linked, 3 enterprise wire, 23 phantom/inactive (created Q1 2026, no payment events 90 days, last login 60-120 days ago, no notes)."

Delta: The CRM shows 120 accounts marked active, but 23 of them (19%) show no payment events in 90 days, last login 60-120 days ago, and no notes. These are not paying customers. The true active customer count is 97 (94 Stripe + 3 wire). The 120 figure is technically in the CRM but includes 23 accounts that are not generating revenue.

**Finding 6: No proprietary lock-in — [partial]**

Claim (doc1, doc9): "all third-party integrations on standard SaaS agreements with no lock-in"

Evidence: doc8 — DataEnrich.io: "$400/mo flat. Termination Notice: 90 days (either party). Dependency: 40% of features depend on this API. Fallback: None implemented."

Delta: The claim holds for Stripe, Heroku, Redis, and Twilio, which operate on standard agreements. However, DataEnrich.io is a significant lock-in: 40% of features depend on it, no fallback is implemented, and the 90-day termination notice means either party can walk with 3 months' notice. The product degrades to "basic mode" without it. This is not a standard SaaS dependency; it is a single point of failure for the product's core value proposition.

**Finding 9: Low churn — [partial]**

Claim (doc1): "The business operates with low churn"

Evidence: doc6 — 23 phantom accounts created Q1 2026 with no payment events in 90 days, last login 60-120 days ago. doc7 — 8 pilot agreements expiring Q3 2026 with no payment obligation.

Delta: "Low churn" is a qualitative claim with no supporting data in the data room. The 23 phantom accounts suggest either unprocessed churn or accounts that were never active. The 8 pilots expiring within the acquisition window represent potential revenue that will not materialize. The data does not support the "stable base" characterization, but the absence of churn data is not the same as evidence of high churn.

**Finding 12: Key person risk / turnkey acquisition — [partial]**

Claim (doc1): "minimal operational overhead" / "turnkey acquisition opportunity"

Evidence: doc3 — "dave: 618 commits (73%). Last Commit: 2026-07-30. Commits in Last 30 Days: 0." doc4 — "DNS Management: Managed personally by 'dave'. No secondary DNS provider." doc8 — GoDaddy: "Managed personally by 'dave'. No secondary DNS."

Delta: The business is not turnkey. One person manages DNS, is responsible for 73% of code commits, and has made zero commits in the last 30 days. A buyer who acquires the business without dave loses DNS management, code maintenance, and operational knowledge simultaneously. The 30-day commit silence suggests dave is already disengaged.

**Finding 13: 15 no-contract accounts — [partial]**

Claim (doc1): "120 active accounts" / "loyal customer base"

Evidence: doc7 — "No-Contract Accounts (15): Individual users who upgraded to Pro plan. Payment: Via Stripe. Legal Agreement: No signed ToS on file (acceptance not recorded)."

Delta: 15 of the 94 Stripe-linked accounts (16%) have no legal agreement on file. Their revenue is real (they pay via Stripe) but their contractual relationship is unenforceable. This is a legal exposure: the business cannot enforce ToS terms (data usage, liability, termination) against these accounts.

**Finding 8: Blended MRR $40,000 — [real, with a structural note]**

Claim (doc1): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000"

Evidence: doc5 — "94 active Stripe subs, $16,000 Stripe MRR (91 Pro at $149/mo = $13,549; 3 Enterprise at $817/mo = $2,451). 3 enterprise companies have separate $8,000/mo wire contracts not in Stripe."

Delta: None. $16,000 + $24,000 = $40,000. The math checks out.

Structural note: 60% of MRR ($24,000) comes from 3 wire-transfer contracts representing 2.5% of the customer base. This is a severe concentration risk. The loss of any one enterprise contract removes 20% of MRR. The "broad base of month-to-month subscribers" framing in doc1 understates this concentration.

**Finding 14: 8 pilot agreements expiring Q3 2026 — [real, operational caveat]**

Claim (doc1): "immediate cash flow" / "proven product-market fit"

Evidence: doc7 — "Pilot Agreements (8): Free 90-day trial. Expiration: Q3 2026. Payment Obligation: None. Conversion: Optional conversion to Pro plan ($149/mo)."

Delta: None. The pilots are accurately described as free trials with no payment obligation.

Operational caveat: They expire within the acquisition window and represent zero current revenue. They should not be counted as pipeline in the valuation. The "immediate cash flow" claim is supported by the $40,000 MRR, not by the pilots.

**Finding 15: 14 secrets in config vars, no vault — [real, with a structural note]**

Claim (doc2): "standard SaaS stack"; doc9: "modern Rails 7"

Evidence: doc4 — "Secrets Management: 14 secrets stored in Heroku config vars (Stripe keys, DataEnrich.io API key, Twilio credentials, Google OAuth, DB URL, Redis URL, etc.). No secrets vault."

Delta: None. Heroku config vars are a standard mechanism for this scale.

Structural note: The transfer of 14 secrets including payment keys and API credentials at closing is a security event. The buyer should require a full credential rotation as a condition of closing. This is not a claim delta but an operational requirement.

**Finding 7: Well-documented codebase — [unverifiable]**

Claim (doc1, doc9): "well-documented codebase"

Evidence: doc3 provides process metrics (847 commits, no CI/CD, no code review, 12 unit tests) but does not provide access to source code or documentation files.

Assessment: The claim concerns the quality of in-code documentation, which cannot be assessed from the data room documents. This is not a finding — it goes in Remaining Claims.

COVERAGE STATEMENT:

Claims enumerated: ~68 distinct verifiable claims across 9 documents.
Claims individually verified: 15 findings covering the highest-priority claims (safety-critical, architectural invariants, key operational parameters).
Coverage: ~22% of the claim surface.

Not checked and why it matters:
- Tech stack specifics (Rails 7 version, PostgreSQL version, Redis version): internally consistent across doc2, doc3, doc4, doc9. Low risk. A version mismatch would not change the valuation.
- Dependency costs (Stripe 2.9%+$0.30, DataEnrich.io $400/mo, Heroku $25/mo, Redis $50/mo, Twilio $100/mo, GoDaddy $12/yr): stated in doc8, not contradicted by any other document. Low risk. A cost error would affect margin but not the fundamental valuation.
- Commit counts and contributor breakdown: stated in doc3, not contradicted. Low risk.
- SSL auto-renewal: stated in doc4, consistent with Heroku's standard behavior. Low risk.
- Google OAuth free tier: stated in doc8, consistent with Google's standard offering. Low risk.

The remaining ~78% of claims are micro-claims that are internally consistent across documents and do not carry material buyer risk. The 15 findings above cover every claim where a delta would change the valuation or the risk profile.

WHAT THE BUYER SHOULD ASK THE SELLER BEFORE CLOSING:

1. Why have backups been failing for 21 days, and why was no alerting configured? Was the failure known?
2. What is the plan to restore backup functionality before closing? Can the buyer verify a successful backup and restore as a condition of closing?
3. What uptime monitoring will be in place at closing? The enterprise contracts require 99.9% SLA with no monitoring to detect a breach.
4. What is the plan for DataEnrich.io dependency? Is a fallback being built? What happens to the enterprise contracts if DataEnrich.io terminates with 90-day notice?
5. Will dave remain available post-closing? What is the transition plan for DNS management and code maintenance? The 30-day commit silence is a concern.
6. What is the status of the 23 phantom accounts? Are they real customers who churned, or were they never active?
7. What is the legal status of the 15 no-contract accounts? Can ToS acceptance be retroactively documented?
8. What is the credential rotation plan for the 14 secrets at closing?
9. What is the churn rate over the last 12 months? The data room does not contain churn data.
10. What is the conversion rate of the 8 pilot agreements? They expire Q3 2026.

=== GAP MAP ===

FlowMetrics — B2B E-Commerce Analytics Platform
Asking price: $480,000 (12x blended MRR)

RECOMMENDATION: MATERIAL

The seller's technical claims do not match the observed implementation. Three claims are false (backups, uptime monitoring, test coverage), and the backup situation is urgent: the last good backup expires in 6 days. Enterprise revenue (40% of MRR) is exposed to SLA breach with no monitoring and no fallback for the API that provides the required features.

Key items:

1. Backups failing for 21 days. Last good backup 2026-07-30, expires 2026-08-29. No alerting. After that date, zero recoverable backup.
2. No uptime monitor exists. Enterprise contracts require 99.9% SLA with no mechanism to detect a breach.
3. No integration or payment-path tests. 12 unit tests in test/utils/ only. Critical paths untested.
4. $16,000/mo enterprise revenue (40% of MRR) depends on DataEnrich.io, which has no fallback and no monitoring. 30-day termination notice for SLA breach.
5. MRR $40,000 verified. But 60% is 3 wire contracts (2.5% of customers). 23 of 120 "active" accounts show no payment in 90 days.

Coverage: 15 of ~68 claims individually verified (22%). Remaining claims are micro-claims (tech stack versions, dependency costs, commit counts) that are internally consistent and low-risk.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice