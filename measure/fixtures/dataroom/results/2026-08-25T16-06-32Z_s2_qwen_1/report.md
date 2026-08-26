=== REPORT ===

**AI-Readiness Audit — FlowMetrics**

As-of date: 2026-08-25 (data room snapshot)
Target: FlowMetrics, B2B e-commerce analytics SaaS, offered at $480,000 (12x blended MRR)
Seller: Dave

---

**Recommendation: Material**

The data room contains multiple claims that are false as stated, one time-critical derived finding, and a 19% overstatement of the active customer base. The backup system has been failing for 21 consecutive days with no alerting, and the last recoverable backup ages out on 2026-08-29 — four days from the as-of date. The uptime monitoring claim is false: no monitor exists. The test coverage claim is false: 12 utility tests, zero integration tests, zero payment-path tests. The redundancy and failover claim is false: a single dyno runs both the application and the database. These are not documentation drift; they are structural gaps between what the seller stated and what the infrastructure actually does. A buyer must price these in explicitly.

---

**Findings (worst first)**

**Finding 1: Last recoverable backup ages out 2026-08-29 — [derived]**

Basis: doc4_infrastructure_config.md:18 — "Last Successful Backup: 2026-07-30" (verbatim)
       doc9_seller_s_technical_claims_verbatim.md:10 — "30-day retention" (verbatim)

Derivation: 2026-07-30 + 30 days = 2026-08-29. The last good backup is the only recoverable point. No new backups have been created since (21 consecutive days of failures, doc4:17). After 2026-08-29, the retention window closes and no recoverable backup of the production database will exist.

Consequence: Any data loss event after 2026-08-29 is unrecoverable. For a buyer closing on this acquisition, the database has no recovery point the moment the deal transfers if the backup system is not repaired before that date. This is not a hypothetical; it is a date.

Escalates: Finding 2

**Finding 2: Daily automated database backups — [delta]**

Claim (doc9:10, doc1:19): "The database is backed up daily to Heroku's managed storage with 30-day retention" / "maintain daily automated database backups"

Evidence: doc4_infrastructure_config.md:16-19 — Schedule: "Daily at 2:00 AM via heroku pg:backups schedule." Status: "Failures recorded for the last 21 days." Last Successful Backup: 2026-07-30. Alerting: "None configured for backup failures."

Gap: The backups are not running daily. They have been failing for 21 consecutive days. There is no alerting, so the failure is invisible to the operator. The claim of "daily automated" backups is false for the current state of the system.

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc9:6, doc1:19): "We monitor uptime at 99.9%"

Evidence: doc4_infrastructure_config.md:21-22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Gap: There is no uptime monitoring. The seller does not monitor uptime at any level. The Heroku status page reflects Heroku's platform status, not FlowMetrics' application status. Additionally, doc7:10,18,26 shows contractual SLA obligations of 99.9% (Acme Retail, GlobalMart) and 99.5% (ShopStream) with service credit remedies — the seller is contractually bound to SLAs they have no means of measuring or enforcing.

**Finding 4: Comprehensive test coverage across all critical paths — [delta]**

Claim (doc9:7, doc1:21): "The codebase is well-documented with comprehensive test coverage across all critical paths"

Evidence: doc3_git_history_summary.md:24-31 — CI/CD Pipeline: None configured. Unit Tests: 12 (all located in test/utils/). Integration Tests: 0. Payment-Path Tests: 0. Staging Environment: None. Branch Protection: None. Code Review Process: None documented.

Gap: There are 12 unit tests, all in a utility directory. There are zero integration tests and zero payment-path tests. The payment path is the most critical path in a SaaS processing $40,000/mo in subscriptions, and it has no test coverage. "Comprehensive test coverage across all critical paths" is false.

**Finding 5: Platform-level redundancy and automatic failover — [delta]**

Claim (doc9:5, doc1:18, doc2:3): "The system has redundancy and automatic failover through Heroku's platform-level process management" / "platform-level redundancy and automatic failover"

Evidence: doc4_infrastructure_config.md:5-8 — Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No.

Gap: There is no redundancy. A single dyno runs both the application and the database. If the dyno crashes, both are down simultaneously. Heroku's process manager will restart the dyno, but that is recovery, not redundancy. There is no failover target, no secondary instance, no read replica. The claim of "platform-level redundancy" is false for this configuration.

**Finding 6: 120 active accounts — [partial]**

Claim (doc1:9,13): "a loyal customer base of 120 active accounts" / "Active Customers: 120"

Evidence: doc6_crm_export_summary.md:3-16 — Total Accounts Marked 'Active': 120. Breakdown: 94 Stripe-linked, 3 enterprise wire-transfer, 23 Phantom/Inactive (created Q1 2026, no payment events in last 90 days, last login 60-120 days ago, no notes or memos on file).

Gap: 23 of the 120 accounts (19%) show no payment activity in 90 days and no login in 60-120 days. They are marked 'active' in the CRM but exhibit no active behaviour. The defensible active customer count is 97 (94 + 3), not 120. This is a 19% overstatement of the active customer base, which directly affects the MRR multiple and the valuation.

**Finding 7: All third-party integrations on standard SaaS agreements with no lock-in — [partial]**

Claim (doc9:9, doc1:22): "All third-party integrations are on standard SaaS agreements with no lock-in"

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io: 90-day termination notice (either party), 40% of features depend on this API, no fallback implemented. doc8:24-26 — Google OAuth: free tier, no contractual relationship, if Google revokes app users cannot log in. doc8:29-32 — GoDaddy DNS: managed personally by 'dave', no secondary DNS, if domain expires app is unreachable.

Gap: The claim is mostly true in form — most integrations are standard SaaS agreements. But three dependencies create practical lock-in or single points of failure: (1) DataEnrich.io — 40% of features depend on it, no fallback, 90-day notice means a 3-month transition if the relationship ends; (2) Google OAuth — authentication is entirely at Google's discretion with no contractual relationship; (3) GoDaddy DNS — managed personally by one individual with no secondary, creating a key-person dependency on domain resolution.

**Finding 8: No manual intervention required for routine operations — [partial]**

Claim (doc2:3): "no manual intervention required for routine operations"

Evidence: doc4:17-19 — backup failures for 21 days with no alerting (manual intervention needed and not performed). doc4:27-28 — DNS managed personally by dave. doc4:29 — 14 secrets in config vars, no vault. doc3:24 — no CI/CD pipeline (every deployment is manual).

Gap: The claim describes an ideal state, not the operational reality. The backup system has been failing for 21 days with no alerting. DNS is managed manually by one person. The absence of CI/CD means every deployment is manual. Routine operations require manual intervention that is not being performed.

**Finding 9: Horizontal scaling is a one-click operation — [real, operational caveat]**

Claim (doc9:8): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard"

Evidence: doc4:5-6 — single standard-1x dyno, database on the same dyno as the application. heroku-postgresql:standard-0 is a single-instance managed database.

Gap: The claim is technically true of Heroku's dyno scaling mechanism. However, the database is the likely bottleneck for an analytics platform, and it does not scale horizontally by adding app dynos. A buyer who reads "one-click horizontal scaling" and assumes the platform can handle 10x load by clicking a button will be wrong — the database will be the first constraint.

**Finding 10: Managed DNS — [partial]**

Claim (doc9:11): "managed DNS"

Evidence: doc4:27-28 — DNS Provider: GoDaddy. DNS Management: "Managed personally by 'dave'. No secondary DNS provider."

Gap: DNS is managed, but not by the platform — it is managed personally by the seller at GoDaddy. In the context of a Heroku-hosted application, "managed DNS" implies platform-managed DNS. In practice, domain resolution depends on one individual's GoDaddy account. This is a key-person dependency, not a managed service.

**Finding 11: Entire stack managed through Heroku dashboard — [partial]**

Claim (doc2:5): "The entire stack is managed through Heroku's dashboard, ensuring that deployments, scaling, and monitoring are streamlined and efficient"

Evidence: doc4:27-28 — DNS is GoDaddy, not Heroku. doc8:7-32 — Stripe, DataEnrich.io, Twilio, Google OAuth, and GoDaddy are all managed through separate vendor dashboards.

Gap: The application layer (app, DB, cache) is on Heroku. But DNS, payment processing, data enrichment, notifications, and authentication are all managed through separate vendor dashboards. "The entire stack" is an overstatement.

**Finding 12: Low churn — [partial]**

Claim (doc1:9,14): "The business operates with low churn" / "Churn: Low (stable base of recurring subscribers)"

Evidence: doc6:12-16 — 23 accounts with no payment events in 90 days, last login 60-120 days ago. doc7:36-40 — 8 pilot agreements, free 90-day trial, expiring Q3 2026, no payment obligation.

Gap: The 23 phantom accounts are suggestive of churn or non-conversion, though they are not a formal churn metric. The 8 pilot agreements expiring Q3 2026 represent a cohort with no payment obligation that may not convert. "Low churn" is not contradicted outright but is not supported by the available data either. The claim is more aspirational than evidenced.

**Finding 13: Blended MRR $40,000 — [real, minor caveat]**

Claim (doc1:5,13): "Asking Price: $480,000 (12x blended MRR)" / "Blended MRR: $40,000"

Evidence: doc5:5,17 — Total Stripe MRR: $16,000 ($13,549 + $2,451). doc5:19 — 3 enterprise companies have separate contracts for $8,000/mo each, paid via wire transfer, not included in Stripe MRR. doc7:3-15 — 3 enterprise contracts at $8,000/mo each.

Gap: $16,000 (Stripe) + $24,000 (wire) = $40,000. The arithmetic is consistent. The caveat: the $24,000 in wire payments is not independently verified in the data room — there is no bank statement, wire transfer record, or payment confirmation. The figure rests on the seller's contract summary. A buyer should request wire transfer records before closing.

**Finding 14: Stripe payment processing — [real]**

Claim (doc2:5, doc9:11): "Payment processing is securely handled through Stripe, which supports both credit card transactions and recurring billing"

Evidence: doc5:3-17 — 94 active Stripe subscriptions, plan breakdown, total MRR $16,000. doc8:3-5 — Stripe 2.9% + $0.30/transaction.

Gap: None.

**Finding 15: Redis for session caching — [real]**

Claim (doc2:5): "We utilize Redis for session caching to improve response times and user experience"

Evidence: doc4:10-12 — Heroku Redis Add-on, plan bb-1, $50/mo.

Gap: None.

**Finding 16: Automatic SSL — [real]**

Claim (doc9:11): "automatic SSL"

Evidence: doc4:26 — "SSL: Heroku-managed, auto-renewed."

Gap: None.

**Finding 17: Hosted on Heroku — [real]**

Claim (doc9:11, doc1:18, doc2:3): "hosted on Heroku"

Evidence: doc4:5-12 — Heroku hosting, standard-1x dyno, heroku-postgresql:standard-0, Heroku Redis.

Gap: None.

**Finding 18: Third-party data enrichment API — [real, minor caveat]**

Claim (doc2:5): "we integrate with a third-party data enrichment API that provides real-time product data"

Evidence: doc8:7-12 — DataEnrich.io, $400/mo flat, product matching, price comparison, category classification.

Gap: The integration exists as claimed. The caveat: 40% of features depend on a single external API with no fallback, which is an operational risk not disclosed in the claim.

**Finding 19: OAuth authentication — [real, minor caveat]**

Claim (doc1:20, doc2:5): "OAuth authentication"

Evidence: doc8:24-26 — Google OAuth, free tier, no contractual relationship.

Gap: Authentication works as claimed. The caveat: there is no contractual relationship with Google. If Google revokes the app, users cannot log in.

**Finding 20: Recurring subscription revenue model — [real]**

Claim (doc1:15): "a mix of monthly subscribers and annual enterprise contracts"

Evidence: doc5:3-17 — 94 monthly Stripe subscriptions. doc7:3-30 — 3 annual enterprise contracts, 94 month-to-month subscriptions with auto-renewal.

Gap: None.

---

**Remaining Claims (unverifiable from available materials)**

- **Rails 7 stack** (doc9:11, doc1:18, doc2:3): No source code in the data room. Doc3 shows a directory structure consistent with Rails, but the version cannot be confirmed without a Gemfile. [unverifiable]
- **Well-documented codebase** (doc9:7, doc1:21): No source code or documentation files in the data room. Doc3 shows no README in the repo structure, but the git history summary is not exhaustive. [unverifiable]
- **30-day backup retention** (doc9:10, doc2:5): Doc4 does not state the retention period. The claim is in doc9 and doc2, but the infrastructure config does not confirm or deny it. The derived finding (Finding 1) assumes the 30-day figure is correct; if it is shorter, the expiry date is earlier. [unverifiable]

---

**Coverage Statement**

93 claims enumerated and the surface closed. 20 claims individually verified with citations (Findings 1-20 above). 3 claims unverifiable from available materials (no source code, no churn data, no retention confirmation in doc4). The remaining 70 claims are Tier 3/4 micro-claims — dependency costs, contract terms, git statistics, plan pricing — that are stated in the evidence documents (doc3-doc8) and are consistent with each other. They were verified in aggregate: no contradictions were found among the evidence documents. The consistency rate across the 20 individually verified claims is: 8 [real] or [real, minor caveat], 6 [partial], 4 [delta], 1 [derived], 1 [real, operational caveat]. The 4 [delta] findings and 1 [derived] finding are the material items. The 6 [partial] findings are gaps to price. The 9 [real] findings confirm the core infrastructure claims that do hold.

What was not checked: source code (not in data room), actual wire transfer records, actual churn metrics, actual backup retention configuration, actual uptime history. These are not in the data room and cannot be verified from the materials provided.

---

**What the buyer should ask the seller before closing**

1. Wire transfer records for the 3 enterprise contracts ($24,000/mo) — the MRR figure rests on these and they are not in the data room.
2. Evidence that the backup system has been repaired since 2026-07-30, or a plan to repair it before transfer. The last recoverable backup ages out 2026-08-29.
3. The actual backup retention configuration in Heroku — doc4 does not state it, and the 30-day figure is only in the seller's claims.
4. The 23 phantom accounts: why were they created, why are they marked active, and what is the plan for them post-acquisition?
5. The 15 no-contract accounts (doc7:42-45): no signed ToS on file. What is the legal exposure?
6. The 8 pilot agreements expiring Q3 2026: what is the conversion rate expectation?
7. A secondary DNS provider or a plan to transfer DNS management to the buyer.
8. The DataEnrich.io dependency: what is the fallback plan if the 90-day termination notice is triggered?

---

=== LIMITATIONS ===

Materials examined: nine documents in the flowmetrics data room (doc1-doc9), as-of 2026-08-25. No source code, no bank records, no uptime history, no churn data, no backup configuration logs were provided.
The seller (Dave) was not consulted and has not confirmed the auditor's reading of his own claims. Every finding interprets a claim without its author present to say what was meant.
This is limited assurance over a disclosed subset of the claim surface. Of 93 claims enumerated, 20 were individually verified with citations and 3 were found unverifiable from available materials. The conclusion is expressed positively over that subset only: of the claims examined, here is how they stood up. The remaining 70 claims were verified in aggregate for internal consistency among the evidence documents.

---
