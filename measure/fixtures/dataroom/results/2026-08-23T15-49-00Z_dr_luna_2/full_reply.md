# FlowMetrics buyer-side due-diligence report

## 1. Recommendation: Do not close on the current evidence

I recommend **pause / do not close pending material verification and remediation**. I cannot recommend “Clear” or “Clear with caveats”: the data room contains a direct operational contradiction, failed backups, questionable customer-status reporting, and no source code or implementation artifacts. The financial figures reconcile arithmetically, but collection and entitlement are not independently established.

The claim surface is **140 explicit claims across nine documents**. I reviewed the complete data-room corpus. It contains documents only—no source code, tests, deployment manifests, or other implementation artifacts—so code-level verification is unavailable.

## 2. Findings, worst first

### Finding 1: Backup protection is not functioning — [not verified / material delta]

**Claim (doc9_seller_s_technical_claims_verbatim.md):** “We monitor uptime at 99.9% and maintain daily automated database backups.” The same document also states: “The database is backed up daily to Heroku's managed storage with 30-day retention.”

**Evidence (doc4_infrastructure_config.md:15–19):**
- “**Schedule:** Daily at 2:00 AM via `heroku pg:backups schedule`”
- “**Status:** Failures recorded for the last 21 days.”
- “**Last Successful Backup:** 2026-07-30”
- “**Alerting:** None configured for backup failures.”

**Delta:** A configured daily schedule is not equivalent to successful daily backups. The evidence records 21 consecutive days of failures, a last successful backup of 2026-07-30, and no failure alerting. The stated 30-day retention also does not cure the absence of current successful backups.

This is documentary evidence, not an implementation-file finding. The absence of source artifacts prevents independent verification of the backup job, retention configuration, restore process, or actual recoverability.

### Finding 2: The claimed uptime monitoring does not exist — [false / material delta]

**Claim (doc9: technical claims):** “We monitor uptime at 99.9% and maintain daily automated database backups.”

**Evidence (doc4:21–23):**
- “**Uptime Monitor:** None (No Pingdom, UptimeRobot, or custom checks).”
- “**Status Page:** Heroku built-in status page only.”

**Delta:** The data-room infrastructure record directly contradicts the claim that uptime is monitored at 99.9%. Heroku’s platform status page is not evidence of an application-level uptime monitor or a measured 99.9% application availability rate. The customer contracts also contain 99.9% SLA commitments for Acme Retail and GlobalMart, increasing the commercial significance of this gap (doc7).

### Finding 3: Customer count is overstated or ambiguously defined — [materially qualified]

**Claim (doc1:9, 12–13):** “We have built a loyal customer base of 120 active accounts.” The listing reports “**Blended MRR:** $40,000” and “**Active Customers:** 120.”

**Settling evidence (doc6):**
- “Total Accounts Marked Active: 120”
- “Stripe-Linked Accounts: 94”
- “Wire Transfer Enterprise Accounts: 3”
- “Phantom/Inactive Accounts: 23”
- “All 120 accounts are currently marked as ‘active’ in the CRM.”

The CRM summary says the 23 phantom/inactive accounts had “no payment events in the last 90 days,” last login dates “60–120 days ago,” and no explanatory notes or memos.

**Delta:** The 120 figure is a CRM-status count, not an established count of paying or currently active customers. On the available evidence, 94 Stripe-linked accounts plus three wire-transfer enterprise accounts are identifiable as revenue-linked; 23 records are expressly described as phantom/inactive. Dave must define “active” and provide evidence for each remaining account.

### Finding 4: MRR reconciles mathematically but not evidentially — [unresolved]

**Claim (doc1:9, 12):** “Our blended Monthly Recurring Revenue (MRR) stands at $40,000.”

**Settling evidence:**
- Stripe (doc5): “**Total Active Subscriptions:** 94” and “**Total MRR (Stripe):** $16,000.”
- Contracts (doc7): Acme Retail, GlobalMart, and ShopStream each have a contract value of “$8,000 per month.”

The three contracts therefore add $24,000 per month to the $16,000 Stripe total, which equals the stated $40,000. This is an arithmetic reconciliation, not proof of realized revenue.

**Delta:** The wire-transfer portion requires proof of execution, current status, invoices, bank receipts, and renewals. The contract renewal dates are Acme Retail—2027-03-15, GlobalMart—2026-11-01, and ShopStream—2027-06-20. The listing’s “low churn” claim is not independently established by the supplied summaries.

### Finding 5: Enterprise revenue is concentrated and SLA exposure is material — [operational caveat]

**Claim (doc1):** Revenue is driven by high-value enterprise contracts and a broad subscriber base.

**Evidence (doc7):** Three identified enterprise contracts each have a value of “$8,000 per month,” together representing $24,000 monthly. Acme Retail and GlobalMart each carry a 99.9% SLA; ShopStream carries a 99.5% SLA. The remedy is a service credit of 10% of the monthly fee per 0.1% below the applicable SLA.

**Delta:** The three wire contracts represent 60% of the stated $40,000 MRR. The absence of uptime monitoring and the backup failures create direct risk against the contractual SLA commitments. Renewal and termination rights also make customer retention and near-term revenue durability important closing conditions.

### Finding 6: “Comprehensive test coverage” is unsupported — [not verified / material coverage gap]

**Claim (doc9):** “The codebase is well-documented with comprehensive test coverage across all critical paths.”

**Evidence available:** doc3’s summary records 12 unit tests, all in `test/utils`, with zero integration or payment-path tests. No source code or test files are present in the data room.

**Delta:** The available summary does not support “comprehensive” coverage, particularly for integration and payment paths. I could not independently inspect or run the tests.

### Finding 7: External dependency has no fallback — [structural note]

**Claim (doc9):** “All third-party integrations are on standard SaaS agreements with no lock-in.”

**Evidence (doc8:7–12):** DataEnrich.io supplies product matching, price comparison, and category classification; it costs “$400/mo,” requires “90 days” termination notice, and “40% of features depend on this API.” “No fallback is implemented”; if unavailable or repriced, “the product degrades to basic mode without enrichment.”

**Delta:** A standard agreement does not eliminate operational or product dependency. Forty percent feature dependence, no fallback, and a 90-day notice period are material qualifications to “no lock-in.”

## 3. Coverage and limitations

I reviewed all nine supplied documents and compared their claims internally. I did not inspect source code, run tests, inspect production systems, verify Heroku or Stripe directly, test backup restoration, validate bank receipts, confirm customer identities, or independently measure uptime. Those omissions matter because the method is intended to compare claims with implementation; this data room does not contain the implementation evidence needed to complete that comparison. The recommendation should therefore be treated as a diligence stop, not as a finding that every untested claim is false.

## 4. Questions and closing conditions for Dave

1. Provide source code, repository access, deployment configuration, CI history, and the complete test suite.
2. Provide backup logs from 2026-07-30 onward, explain the 21-day failure period, fix alerting, and demonstrate a successful restore into an isolated environment.
3. Provide uptime-monitoring history and reconcile the 99.9% seller statement with the absence of an application monitor.
4. Define “active account” and provide a row-level customer schedule identifying the 120 CRM records, payment status, last payment, last login, contract, and cancellation status.
5. Provide executed contracts, invoices, bank statements or remittance evidence, and current renewal status for Acme Retail, GlobalMart, and ShopStream.
6. Explain the 23 phantom/inactive records and why they remain marked active.
7. Provide churn, cohort, refund, failed-payment, and MRR-recognition reports for the last 12 months.
8. Explain SLA measurement and service-credit exposure for the three enterprise customers.
9. Provide DataEnrich.io contract terms, usage limits, pricing history, and a fallback or migration plan.

Until these items are resolved, I would not rely on the stated customer count, uptime posture, backup protection, or full MRR as closing facts.

=== GAP MAP ===

**Target:** FlowMetrics — B2B e-commerce analytics SaaS offered for sale by Dave.

**Recommendation:** Pause; do not close pending material verification and remediation.

- Backups have failed for 21 days, with no failure alerting or demonstrated restore.
- Claimed uptime monitoring is not present.
- The 120-account figure includes 23 phantom/inactive CRM records; only 94 Stripe-linked and three wire-transfer accounts are revenue-linked in the supplied summaries.
- The $40,000 MRR arithmetic reconciles, but the $24,000 wire-transfer component still needs collection and contract verification.
- No source code or implementation artifacts were supplied; comprehensive testing and production behavior remain unverified.

**Coverage:** All nine data-room documents reviewed; implementation, production, payment, bank, and restore verification not performed because the required artifacts and access were unavailable.