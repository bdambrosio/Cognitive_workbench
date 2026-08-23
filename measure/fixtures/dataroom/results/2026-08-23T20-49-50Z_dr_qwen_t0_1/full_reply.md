# AI-Readiness Audit — FlowMetrics

**Date:** 2026-08-23
**Target:** FlowMetrics — B2B E-Commerce Analytics Platform
**Seller:** Dave
**Asking Price:** $480,000 (12× blended MRR)
**Auditor:** Jill

---

## 1. Recommendation

**Material.**

Five claims are false as stated. Two derived consequences carry time pressure and contractual exposure. The revenue figure is real, but the operational and architectural claims that support the "turnkey, robust, low-overhead" framing do not hold. The buyer must price the gaps explicitly or walk.

---

## 2. Findings (worst first)

**Finding 1: Daily automated database backups with 30-day retention — [delta]**

Claim (doc1:19; doc9:10): "daily automated database backups with 30-day retention to Heroku's managed storage."

Evidence (doc4:16–19): Schedule is daily at 2:00 AM via `heroku pg:backups schedule`. "Failures recorded for the last 21 days." "Last Successful Backup: 2026-07-30." "Alerting: None configured for backup failures."

Delta: The backups are not functioning. The last successful backup was 24 days before this audit. The 30-day retention claim is moot if no new backups are being created. No one is alerted when they fail.

---

**Finding 2: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "We monitor uptime at 99.9%."

Evidence (doc4:22–23): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." "Status Page: Heroku built-in status page only."

Delta: There is no uptime monitoring. The 99.9% figure is not measured by any tool the seller operates. The enterprise contracts (doc7:8,16) promise 99.9% SLA with service-credit remedies, but the seller has no mechanism to detect, report, or quantify an SLA breach.

---

**Finding 3: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3; doc9:5): "platform-level redundancy and automatic failover through Heroku's process management."

Evidence (doc4:5–8): Single `standard-1x` dyno (1 GB RAM, 0.5 CPU). Database `heroku-postgresql:standard-0` running on the same dyno as the application. "Read Replicas: None." "Separate DB Instance: No."

Delta: A single dyno with the database co-located is a single point of failure. Heroku's process management restarts a crashed dyno; that is recovery, not redundancy. There is no second instance to fail over to. The claim describes a capability the infrastructure does not have.

---

**Finding 4: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): "Well-documented codebase with comprehensive test coverage across critical paths."

Evidence (doc3:27–29): "Unit Tests: 12 (all located in `test/utils/`)." "Integration Tests: 0." "Payment-Path Tests: 0."

Delta: Twelve unit tests in a utilities directory is not comprehensive test coverage. Zero integration tests. Zero payment-path tests for a business whose primary revenue mechanism is payment processing. The payment path is the critical path, and it has no tests. This is the absence of the thing the claim describes.

---

**Finding 5: No manual intervention required for routine operations — [delta]**

Claim (doc2:3): "no manual intervention required for routine operations."

Evidence (doc4:17–19): Backups failing for 21 days with no alerting. (doc4:28): DNS managed personally by 'dave', no secondary provider. (doc3:21–22): Last commit 2026-07-30; 0 commits in last 30 days.

Delta: A critical failure (backup system) went undetected for three weeks because no one was monitoring. DNS requires personal management by a single individual. The application has had no development activity in a month. Routine operations do require manual intervention, and the current state shows that intervention is not being applied.

---

**Finding 6: Backup coverage expires 2026-08-29 — [derived]**

Basis: doc9:10 — "30-day retention" (verbatim). doc4:18 — "Last Successful Backup: 2026-07-30" (verbatim).

Derivation: 30 days from 2026-07-30 is 2026-08-29. Today is 2026-08-23. Six days of backup coverage remain. Backups have been failing since approximately 2026-08-02 (21 days before this audit). If failures continue, no backup will exist after 2026-08-29.

Consequence: After 2026-08-29, the business cannot be restored from any backup. Any data loss, corruption, or deletion after that date is permanent. A buyer closing in the next six days inherits a business with a six-day window of recoverability, after which the data state is unrecoverable.

Escalates: Finding 1.

---

**Finding 7: Enterprise SLAs unenforceable without monitoring — [derived]**

Basis: doc7:8 — Acme Retail "SLA: 99.9%" (verbatim). doc7:16 — GlobalMart "SLA: 99.9%" (verbatim). doc7:9,17 — "Service credit of 10% of monthly fee per 0.1% below SLA." doc4:22 — "Uptime Monitor: None" (verbatim).

Derivation: The seller is contractually bound to 99.9% uptime for two enterprise customers ($16,000/mo combined, 40% of MRR) with service-credit remedies. Without monitoring, the seller cannot detect a breach, calculate the credit owed, or prove compliance. The SLA is unenforceable in practice because the seller cannot measure the metric they contracted to.

Consequence: The seller is contractually exposed to service-credit claims they cannot detect or quantify. A buyer inherits this exposure. The $16,000/mo in enterprise contracts carry SLA obligations the seller has no mechanism to fulfill or defend.

Escalates: Finding 2.

---

**Finding 8: 120 active accounts — [partial]**

Claim (doc1:9,13): "120 active accounts."

Evidence (doc6:3,12–16,18): 120 accounts marked 'active' in CRM. 23 are "Phantom/Inactive Accounts": created Q1 2026, no payment events in last 90 days, last login 60–120 days ago, no notes on file. "All 120 accounts are currently marked as 'active' in the CRM."

Delta: 23 of 120 (19%) show no payment in 90 days and no login in 60–120 days. They are marked active but are not functionally active. The number 120 is in the system; the qualifier "active" does not hold for 19% of the base. Per-account revenue is $333 (using 120) vs. $412 (using the 97 genuinely active accounts).

---

**Finding 9: No proprietary lock-in — [partial]**

Claim (doc1:22; doc9:9): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."

Evidence (doc8:11–12): DataEnrich.io — "40% of features depend on this API." "Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode." (doc8:28): Google OAuth — "No contractual relationship." (doc8:29–31): GoDaddy DNS — "Managed personally by 'dave'. No secondary DNS."

Delta: The data formats are standard, but the functional dependency is severe. 40% of features depend on a single third-party API with no fallback. If DataEnrich.io changes pricing or goes down, the product degrades. That is vendor lock-in in a practical sense. Google OAuth has no contractual relationship at all. The claim of "no lock-in" is contradicted by the dependency structure.

---

**Finding 10: Horizontal scaling / one-click dyno addition — [partial]**

Claim (doc9:8; doc1:18): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence (doc4:5–6): Single `standard-1x` dyno. Database on the same dyno as the application. (doc4:7–8): No read replicas, no separate DB instance.

Delta: Heroku does make adding dynos a one-click operation at the platform level. But the database is co-located on the same dyno. You cannot horizontally scale the application tier without first separating the database. The mechanism exists; the current architecture blocks it.

---

**Finding 11: Turnkey acquisition opportunity — [partial]**

Claim (doc1:9): "This is a turnkey acquisition opportunity."

Evidence (doc4:28): DNS managed personally by 'dave', no secondary. (doc4:17–19): Backups failing 21 days, no alerting. (doc3:21–22): 0 commits in 30 days.

Delta: "Turnkey" implies the business operates without the seller's personal involvement. DNS is managed personally by one individual. Backups are failing and no one is monitoring. The business is not turnkey; it requires the seller's personal attention and lacks the monitoring that would make it self-sustaining.

---

**Finding 12: Robust, defensible product — [partial]**

Claim (doc1:25): "We have built a robust, defensible product."

Evidence (doc4:5–6): Single dyno, DB co-located. (doc3:27–29): 12 unit tests, 0 integration, 0 payment-path. (doc8:11–12): 40% of features depend on DataEnrich.io, no fallback.

Delta: "Robust" is contradicted by the single-dyno architecture, the failing backups, and the absence of testing on the payment path. "Defensible" is contradicted by the 40% dependency on a third-party API with no fallback — the enrichment features are not proprietary, and a competitor could replicate the non-enrichment features.

---

**Finding 13: Minimal operational overhead — [partial]**

Claim (doc1:9): "minimal operational overhead."

Evidence (doc4:28): DNS managed personally. (doc4:17–19): Backups failing, no alerting. (doc3:21–22): 0 commits in 30 days.

Delta: The infrastructure is simple (one dyno, one DB, one Redis). But "minimal overhead" implies it runs itself. It does not: backups are failing and undetected, DNS requires personal management, and the application has had no development activity in a month. The overhead is not minimal; it is unmanaged.

---

**Finding 14: Customers are engaged — [partial]**

Claim (doc1:25): "the customers are engaged."

Evidence (doc6:14–15): 23 accounts with no payment in 90 days, last login 60–120 days ago.

Delta: 97 of 120 accounts (81%) show engagement. 23 (19%) do not. The claim is true for the majority but not for all.

---

**Finding 15: Revenue is recurring — [partial]**

Claim (doc1:25): "the revenue is recurring."

Evidence (doc7:29–34): 94 month-to-month, auto-renewal, no SLA. (doc7:36–40): 8 pilot agreements, free 90-day trial, no payment obligation, expiring Q3 2026. (doc7:42–45): 15 no-contract accounts, no signed ToS on file.

Delta: The revenue is subscription-based and recurring for the 97 accounts with active Stripe or wire contracts. The 23 others (8 pilots with no payment obligation, 15 with no signed ToS) have a weak or absent contractual foundation. The "recurring" nature is real for 81% of the base, not 100%.

---

**Finding 16: PostgreSQL with daily scheduled backups — [partial]**

Claim (doc2:5): "PostgreSQL, which is configured with daily scheduled backups to ensure data integrity and recoverability."

Evidence (doc4:6): `heroku-postgresql:standard-0`. (doc4:16): Schedule daily at 2:00 AM. (doc4:17): Failures last 21 days.

Delta: The schedule exists. The backups are not succeeding. The claim is true in configuration, false in execution. Overlaps with Finding 1 but is a distinct claim in doc2.

---

**Finding 17: Blended MRR $40,000 — [real]**

Claim (doc1:9,12): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000."

Evidence (doc5:5): "Total MRR (Stripe): $16,000." (doc5:19): "The 3 enterprise companies (Acme Retail, GlobalMart, ShopStream) have separate contracts for $8,000/mo each, paid via wire transfer. These payments are not processed through Stripe and are not included in the $16,000 MRR figure above."

Delta: None. $16,000 + $24,000 = $40,000. The 23 phantom accounts are not in the Stripe count (doc6:6: 94 Stripe-linked + 3 enterprise + 23 phantom = 120), so they do not inflate the MRR figure. The revenue is real as stated.

---

**Finding 18: Managed SSL — [real]**

Claim (doc1:20; doc9:11): "Managed SSL."

Evidence (doc4:26): "SSL: Heroku-managed, auto-renewed."

Delta: None.

---

**Finding 19: OAuth authentication — [real, minor caveat]**

Claim (doc1:20): "OAuth authentication."

Evidence (doc8:25–28): Google OAuth, free tier. "If Google changes API or revokes app, users cannot log in. No contractual relationship."

Delta: None. Caveat: no contractual relationship with Google. Low-probability risk.

---

**Finding 20: Secure payment processing via Stripe — [real]**

Claim (doc1:20; doc2:5): "secure payment processing via Stripe."

Evidence (doc5:4–5): 94 active subscriptions, $16,000 MRR. (doc8:3–6): Stripe, 2.9% + $0.30/txn, standard merchant agreement.

Delta: None.

---

**Finding 21: Redis for session caching — [real]**

Claim (doc2:5): "We utilize Redis for session caching."

Evidence (doc4:11–13): Heroku Redis Add-on, `bb-1`, $50/mo. (doc8:17–20): "If down, app is slow but functional."

Delta: None.

---

**Finding 22: Third-party data enrichment API — [real, minor caveat]**

Claim (doc2:5): "we integrate with a third-party data enrichment API that provides real-time product data."

Evidence (doc8:7–12): DataEnrich.io, $400/mo, 40% of features depend on it.

Delta: None. Caveat: see Finding 9 for the lock-in risk.

---

**Finding 23: Revenue-positive — [real, minor caveat]**

Claim (doc1:9): "revenue-positive."

Evidence: MRR $40,000. Known recurring costs (doc8): DataEnrich $400, Heroku $25, Redis $50, Twilio $100, GoDaddy $12/yr, Stripe 2.9%+$0.30/txn. Total known recurring ≈ $587/mo + transaction fees.

Delta: None on known costs. Caveat: no labor costs, no COGS, no other overhead in the data room. Full profitability cannot be confirmed.

---

**Finding 24: Immediate cash flow — [real, minor caveat]**

Claim (doc1:9): "immediate cash flow."

Evidence: MRR $40,000, revenue-positive on known costs.

Delta: None. Caveat: 8 pilot agreements expire Q3 2026 with no payment obligation; 15 accounts have no signed ToS. The cash flow is real but the contractual foundation for 23 of 120 accounts is weak.

---

## 3. Remaining Claims (not findings)

| Claim | Verdict | Reason |
|---|---|---|
| "Low churn" (doc1:9,14) | [unverifiable] | No churn metrics, cohort data, or cancellation records in the data room. |
| "Well-documented codebase" (doc1:21) | [unverifiable] | No source code or documentation artifacts in the data room. |
| "Modern Rails 7 stack" (doc1:18; doc9:11) | [unverifiable] | Seller statement only. No dependency manifest or code to confirm version. |
| "Proven product-market fit" (doc1:9) | [unverifiable] | Qualitative judgement. Revenue exists, but 23 phantom accounts and 8 free pilots complicate the picture. |
| "12× blended MRR" asking price (doc1:5) | [non-delta] | $480,000 / $40,000 = 12×. Arithmetic is correct. Whether 12× is a fair multiple is the buyer's judgement. |

---

## 4. Coverage Statement

**37 claims enumerated. 29 dispositioned as findings. 5 in Remaining Claims. 3 are overlaps already covered by a finding above.**

Effective coverage of distinct assertions: **29/32 ≈ 91%.**

What was not checked and why it matters:
- **Source code and documentation** are not in the data room. Claims about code quality, documentation, and the Rails version cannot be verified. A buyer should request repository access before closing.
- **Churn data** is absent. The "low churn" claim is untested. A buyer should request 12-month cancellation and cohort data.
- **Labor and COGS** are absent. "Revenue-positive" is verified only against known infrastructure costs. Full profitability is unconfirmed.
- **The 23 phantom accounts** could not be individually identified or their status confirmed with the seller. The data room shows they exist and are marked active; it does not explain why.

---

## 5. What the buyer should ask the seller before closing

1. **Backup recovery:** What is the plan to restore the backup system before 2026-08-29? Can you provide a verified backup from the last 7 days, and evidence that the schedule is now succeeding?
2. **Monitoring:** Why is there no uptime monitoring? Will you install a monitoring tool (Pingdom, UptimeRobot, or equivalent) before closing, at your cost?
3. **DNS ownership:** Who controls the GoDaddy account? Will you transfer it to the buyer or a shared account before closing? What is the domain expiry date?
4. **DataEnrich.io:** What is the current termination notice period? Do you have a fallback plan or a second enrichment vendor identified? What would it cost to replace this dependency?
5. **Phantom accounts:** Why are 23 accounts marked active with no payment in 90 days? Can you provide a list of these accounts, the reason for their status, and whether they are expected to convert or should be deactivated?
6. **SLA exposure:** Have there been any SLA breaches in the last 12 months for Acme Retail, GlobalMart, or ShopStream? If so, were service credits paid? Can you provide uptime logs for the last 12 months?
7. **Enterprise contract renewal:** GlobalMart renews 2026-11-01 (approximately 10 weeks from this audit). What is the renewal status? Is there a non-renewal risk?
8. **No-contract accounts:** The 15 accounts with no signed ToS — can you provide evidence of acceptance (click-through logs, email confirmations)? If not, what is the legal standing of those subscriptions?

---

## 6. Executive Synthesis

The revenue is real. The $40,000 MRR checks out arithmetically from the Stripe export and the enterprise wire contracts. The business is generating cash.

The operations are not. The backup system has been failing for three weeks and no one noticed. There is no uptime monitoring despite contractual SLAs that promise 99.9% with service-credit remedies. The "redundancy" is a single server with the database on it. The "comprehensive test coverage" is twelve utility tests and zero tests on the payment path. The "no manual intervention" claim is contradicted by a three-week undetected failure and a DNS account managed personally by the seller.

The most urgent issue is time: the last successful backup was 2026-07-30. With 30-day retention, all backup coverage expires on 2026-08-29 — six days from this audit. After that date, the business has no recoverable data state.

The $480,000 asking price is 12× the verified MRR. The MRR is real. The operational claims that make the business "turnkey" and "robust" are not. A buyer at 12× is paying for a business that runs itself. This business does not run itself. It requires the seller's personal attention, and it is currently failing in ways the seller has not detected.

The gaps are addressable — backup restoration, monitoring installation, DNS transfer, and test coverage can be done in a sprint or two. But they are not done, and the backup expiry creates a hard deadline. The buyer should either require the seller to fix the critical items before closing, or price the remediation into the deal.


=== GAP MAP ===

**FlowMetrics** — B2B E-Commerce Analytics Platform, $480K asking (12× MRR)

**Recommendation: Material**

| # | Item | Note |
|---|---|---|
| 1 | Backup coverage expires 2026-08-29 | Last successful backup 2026-07-30; 30-day retention; 6 days of recoverability remain. After that date, no backup exists. |
| 2 | No uptime monitoring; contractual SLAs unenforceable | Two enterprise contracts ($16K/mo, 40% of MRR) promise 99.9% SLA with service credits. No monitoring tool exists to detect or quantify a breach. |
| 3 | Single point of failure | One dyno, database co-located. No redundancy, no failover, no read replicas. The "platform-level redundancy" claim is false. |
| 4 | No payment-path tests | 12 unit tests in `test/utils/`. Zero integration tests. Zero payment-path tests. The primary revenue mechanism is untested. |
| 5 | 19% of "active" accounts are phantom | 23 of 120 accounts show no payment in 90 days, no login in 60–120 days. Marked active in CRM. MRR is unaffected; account count is inflated. |

**Coverage:** 29 of 32 distinct claims verified. 4 unverifiable (no code, no churn data, no dependency manifest in data room). 1 non-delta (valuation multiple).

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*