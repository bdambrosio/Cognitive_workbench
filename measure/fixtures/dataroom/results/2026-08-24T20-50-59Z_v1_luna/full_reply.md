Recommendation: Material

Executive synthesis

The seller describes platform redundancy, automatic failover, 99.9% uptime monitoring, daily backups with 30-day retention, and no proprietary vendor lock-in. The reviewed infrastructure shows one standard-1x dyno, a same-dyno PostgreSQL deployment, no replicas or separate database, no uptime monitor beyond Heroku’s status page, and 21 days of backup failures with no alerting. A key product-data dependency covers 40% of features and has no implemented fallback. Customer and revenue records also contain classification and reconciliation issues. Coverage is roughly 77 claims or figures across nine documents. Priority operational and commercial claims were checked; implementation-level claims and some product claims remain unverified or only documented.

**Finding 1: Backup continuity is materially inconsistent with the seller’s reliability claim — [delta]**  
Claim (doc1, exact line): “Reliability: 99.9% uptime monitoring and daily automated database backups with 30-day retention.”  
Evidence (doc4:15-19): Backups are scheduled daily, but failures were recorded for the last 21 days; the last successful backup was 2026-07-30; no alerting is configured.  
Delta: The schedule exists, but successful recoverable backups and failure detection are not evidenced. The stated 30-day retention could not be verified from the cited infrastructure record.

**Finding 2: The stated redundancy and automatic failover are not supported by the reviewed topology — [delta]**  
Claim (doc1, exact line): “Scalable Architecture: Built on a modern Rails 7 stack with platform-level redundancy and automatic failover.”  
Evidence (doc4:3-8): The application uses one standard-1x dyno; PostgreSQL runs on the same dyno; there are no read replicas and no separate database instance.  
Delta: The documented deployment has a single application/database hosting arrangement with no documented replica or failover path. I cannot verify platform-level redundancy from these records.

**Finding 3: Uptime monitoring is not evidenced — [delta]**  
Claim (doc1, exact line): “Reliability: 99.9% uptime monitoring and daily automated database backups with 30-day retention.”  
Evidence (doc4:21-23): No uptime monitor is configured; the Heroku built-in status page is the only status source.  
Delta: A provider status page is not evidence of customer-facing uptime monitoring. The 99.9% monitoring claim is unsupported by the reviewed configuration.

**Finding 4: A material product dependency has no implemented fallback — [real, operational caveat]**  
Claim (doc1, exact line): “Low Vendor Lock-in: All third-party integrations are on standard SaaS agreements with no proprietary lock-in.”  
Evidence (doc8:7-12): DataEnrich supplies product matching, price comparison, and category classification; 40% of features depend on it; termination requires 90 days’ notice; no fallback is implemented, and failure or price changes degrade the product to basic mode.  
Delta: The agreement may be standard, but operational dependency remains material. The seller’s broad no-lock-in characterization does not capture the absence of a fallback or the 40% feature dependency.

**Finding 5: CRM account status is internally inconsistent with activity evidence — [partial]**  
Claim (doc1, exact line): “We have built a loyal customer base of 120 active accounts.”  
Evidence (doc6:3 and 12-18): 120 accounts are marked active; 23 are described as phantom/inactive, have had no payment events for 90 days, last logged in 60-120 days ago, and remain marked active.  
Delta: The total of 120 is arithmetically consistent with 94 Stripe-linked accounts, 3 enterprise accounts, and 23 phantom/inactive accounts. The inconsistency is classification: all 120 carry an active status despite the documented inactivity indicators. This does not establish that 23 accounts are definitively noncustomers.

**Finding 6: Reported Pro-plan MRR is arithmetically incorrect — [delta]**  
Claim (doc5, exact line): “Count: 91 accounts; Price: $149/mo; Total MRR: $13,549.”  
Evidence (doc5:7-11): The cited count is 91 and the cited price is $149 per month.  
Delta: 91 × $149 = $13,559, not $13,549. The stated figure is understated by $10 per month.

**Finding 7: Blended MRR reconciles only through the enterprise contracts — [derived]**  
Claim (doc1, exact line): “Our blended Monthly Recurring Revenue (MRR) stands at $40,000.”  
Evidence (doc7:3-27): Three enterprise contracts are each $8,000 per month, totaling $24,000 per month. Evidence (doc5:19): Stripe MRR is stated as $16,000 per month, excluding those three contracts.  
Delta: $24,000 + $16,000 = $40,000 per month, matching the stated blended MRR. This is a derived reconciliation, not a normal claim finding. It does not resolve the $10 Pro-plan arithmetic error or validate the underlying Stripe figure.

**Finding 8: SSL appears managed, but DNS is a single-person operational dependency — [real, operational caveat]**  
Claim (doc1, exact line): No explicit seller claim was provided in the cited material.  
Evidence (doc4:25-28): SSL is Heroku-managed and auto-renewed; GoDaddy DNS is personally managed by Dave; no secondary DNS provider is configured.  
Delta: SSL renewal is documented as automated. DNS administration lacks documented redundancy, creating an operational continuity risk if access or availability is lost.

Remaining Claims

- Testing, CI/CD, staging, branch protection, and code review remain documented gaps rather than claim findings. The available record states no CI/CD, 12 unit tests limited to test/utils, zero integration or payment-path tests, no staging, no branch protection, and no documented code review (doc3:24-32).
- OAuth, outbound data handling, and customer-data/product behavior are unverifiable because source code was not available. Documented Google OAuth risk and GoDaddy DNS risk appear in doc8:25-32, but the implementation could not be independently verified.
- The seller’s low-churn, scalable-stack, minimal-overhead, product-market-fit, and turnkey assertions were not established by the cited evidence.
- The 30-day backup retention, automatic failover implementation, and actual uptime performance remain unverified.

Buyer questions

1. What caused the 21-day backup failure period, and can the seller produce a successful restore test and retention configuration?
2. What component provides failover, given the single dyno, same-dyno database, no replicas, and no separate database?
3. What were the uptime measurements and incidents supporting the 99.9% figure?
4. What is the transition plan and commercial exposure for DataEnrich, including a fallback implementation?
5. Which of the 23 phantom/inactive accounts should be excluded from active-customer reporting, and what supports their continued inclusion?
6. Can the seller reconcile the $10 Pro-plan error and provide source exports for the stated $16,000 Stripe MRR?
7. Who besides Dave can administer GoDaddy DNS, and is a recovery process documented?
8. Can the seller provide source access or evidence for OAuth, outbound integrations, payment-path tests, CI/CD, staging, and code review?

technical claims verification · not a pen-test, not legal advice

=== GAP MAP ===

Flowmetrics — Recommendation: Material

- Revenue arithmetic and enterprise-contract reconciliation are directionally clear, but account-status records need cleanup.
- Backup failures, absent alerting, limited monitoring, and single-host topology create operational exposure.
- DataEnrich supports a substantial portion of the product without a fallback mode.
- SSL renewal is automated; DNS administration lacks documented redundancy.
- Product, OAuth, testing, and implementation claims remain partly unverified or only documented.

Coverage: roughly 77 claims or figures across nine documents; priority operational and commercial claims checked, with implementation-level claims and some product claims unverified or only documented.
Full report with citations available on request.