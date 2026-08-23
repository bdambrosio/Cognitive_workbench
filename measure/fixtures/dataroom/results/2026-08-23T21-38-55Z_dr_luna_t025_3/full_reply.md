## FlowMetrics claims-verification report

### 1. Recommendation: Material

The data room contains material discrepancies between seller claims and the supplied records. The most consequential are failed backups with no failure alerting, unsupported redundancy and automatic-failover claims, absent uptime monitoring despite a 99.9% monitoring claim, and a customer/revenue base whose quality is materially less established than the listing suggests.

The records also show weak testing and deployment controls, dependency and vendor-lock-in exposure, incomplete evidence for security claims, and a managed-DNS claim contradicted by the infrastructure record. Several commercial assertions—low churn, customer engagement, and immediate cash flow—remain unverifiable rather than disproven. I do not treat those as deltas.

The recommendation describes the state of the claims. The buyer must decide whether to proceed, negotiate, or walk.

### 2. Coverage

The nine-document data room yielded roughly 70–80 micro-claims. I prioritised safety- and reliability-adjacent mechanisms, architectural invariants, operational controls, commercial metrics, dependencies, and then lower-level technical claims. The report records the material deltas and the principal caveats found; it is not a line-by-line reproduction of every micro-claim.

The following remain unresolved from the supplied materials: churn measurement, broader customer engagement, continuous subscription retention, the “immediate cash flow” characterization, detailed OAuth controls, detailed SSL configuration, payment-security implementation, documentation quality, and the precise extent of application-level scaling/failover implementation. They are listed as unverifiable where appropriate, not classified as deltas.

### 3. Findings

**Finding 1: Backup continuity and alerting — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6, :10; doc1_seller_listing_description.md:19; exact seller claim): daily backups with 30-day retention.

Evidence: doc4_infrastructure_config.md:15-19 — “Schedule: Daily at 2:00 AM via `heroku pg:backups schedule`”; “Status: Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.”

Delta: The schedule exists, but the records show 21 consecutive days of failures, no backup-failure alerting, and no successful backup after 2026-07-30. The claimed functioning 30-day backup protection is not present in the observed record.

**Finding 2: Claimed redundancy and automatic failover — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:5; doc1_seller_listing_description.md:18; doc2_tech_stack_description_as_provided_by_se.md:3): the system has platform-level redundancy, automatic failover, and automatic scaling through Heroku.

Evidence: doc4_infrastructure_config.md:4-8 — one `standard-1x` dyno; the database runs on the same dyno; “Read Replicas: None”; “Separate DB Instance: No.” doc8_external_dependency_list.md:13-16 — Heroku’s “99.95% platform uptime” SLA is “not contractual for the app.”

Delta: The supplied configuration does not document application redundancy, a separate database, replicas, or automatic failover. The Heroku platform SLA does not substantiate those application-level claims.

**Finding 3: Uptime monitoring — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6; doc1_seller_listing_description.md:19): “We monitor uptime at 99.9%.”

Evidence: doc4_infrastructure_config.md:21-22 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)” and “Heroku built-in status page only.”

Delta: No application uptime monitor or 99.9% availability history is documented. A platform status page is not evidence of the stated monitoring claim.

**Finding 4: Testing and critical-path coverage — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:7): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: doc3_git_history_summary.md:26-29 — the supplied history records 12 unit tests, all in `test/utils/`, with zero integration tests and zero payment-path tests. doc3_git_history_summary.md:32 states that no code-review process is documented.

Delta: The comprehensive critical-path testing portion is contradicted by the absence of integration and payment-path tests. “Well-documented” is not verifiable from the supplied summaries.

**Finding 5: Deployment and recovery controls — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:8, as recorded in the audit materials): deployment and operational controls support reliable releases and recovery.

Evidence: doc3_git_history_summary.md:24-32 records limited deployment/release-control evidence and no documented code-review process; doc4_infrastructure_config.md:21-22 records no independent uptime monitoring.

Delta: The supplied records do not establish a documented rollback control, release approval process, or operational recovery process. The finding is limited to what the records fail to substantiate; it does not claim that no such mechanism exists.

**Finding 6: Customer count quality — [partial]**

Claim (doc1_seller_listing_description.md:9, :13): “120 active accounts” and “Active Customers: 120.”

Evidence: doc6_crm_export_summary.md:3 — “Total Accounts Marked 'Active': 120.” But doc6:5-16 records 94 Stripe-linked accounts, 3 enterprise wire-transfer accounts, and 23 “Phantom/Inactive Accounts”; those 23 had “No payment events in the last 90 days” and last login dates “60-120 days ago.”

Delta: The figure is supported as a CRM status-field count, not as 120 currently active or paying customers. The 23 phantom/inactive records materially qualify the listing claim.

**Finding 7: MRR concentration — [derived]**

Basis: doc5_stripe_export_summary.md:17 — “Total Stripe MRR: $16,000 ($13,549 + $2,451).”

doc7_customer_contracts_summary.md:3-23 — three enterprise contracts, each at “$8,000/mo.”

doc1_seller_listing_description.md:12 — “Blended MRR: $40,000.”

Derivation: Three enterprise contracts × $8,000/month = $24,000/month. $24,000 ÷ $40,000 = 0.60 = 60%.

Consequence: Three documented enterprise customers represent 60% of the stated MRR. The stated total reconciles arithmetically, but the revenue base is materially concentrated.

Escalates: Finding 6.

**Finding 8: Customer-status and recurring-revenue quality — [partial]**

Claim (doc1_seller_listing_description.md:9, :14-15): “low churn,” a “stable base of recurring subscribers,” and recurring subscription revenue.

Evidence: doc5_stripe_export_summary.md:4 records “Total Active Subscriptions: 94.” doc7_customer_contracts_summary.md:29-32 records standard terms with auto-renewal and end-of-cycle cancellation. doc7:36-45 records 8 free 90-day pilots with no payment obligation and optional conversion, plus 15 no-contract accounts with no signed ToS or recorded acceptance.

Delta: Recurring subscriptions and contractual enterprise arrangements are evidenced, but the records also include pilots and accounts without signed terms. No churn, cancellation, retention, or continuity metric is supplied. The “low churn” and “stable” portions therefore remain unverified, while the recurring-revenue characterization is qualified by the mixed account populations.

**Finding 9: Vendor lock-in — [delta]**

Claim (doc1_seller_listing_description.md:25, as recorded in the audit materials): the platform has minimal or no vendor lock-in.

Evidence: doc8_external_dependency_list.md:7-12, :25-32 records dependencies on Heroku, Stripe, Google OAuth, Twilio, Redis, and other external services, including Google OAuth risk where users cannot log in if Google changes its API or revokes the app.

Delta: The dependency record shows material reliance on external vendors and no contractual relationship noted for at least Google OAuth. The blanket no-lock-in characterization is not supported.

**Finding 10: Security evidence — [unverifiable]**

Claim (doc1_seller_listing_description.md:20): “Managed SSL, OAuth authentication, and secure payment processing via Stripe.”

Evidence: doc8_external_dependency_list.md:25-28 identifies Google OAuth as the authentication service; doc4_infrastructure_config.md:25-29 lists Google OAuth and payment-related credentials among Heroku config vars.

Delta: OAuth use and Stripe-related configuration are evidenced, but the materials do not establish scopes, redirect-URI controls, authorization rules, MFA, session controls, audit logging, SSL configuration, or payment-security implementation. The unsupported details remain unverifiable, not delta findings.

**Finding 11: Managed DNS — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:11): “We use a modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc2_tech_stack_description_as_provided_by_se.md:3 confirms a Rails 7 application. doc4_infrastructure_config.md:25-28 identifies GoDaddy as the DNS provider and states: “Managed personally by 'dave'. No secondary DNS provider.”

Delta: The Rails 7 portion is supported, but the managed-DNS assertion is contradicted by the infrastructure record. The automatic-SSL portion remains outside this finding because the supplied evidence does not establish its implementation.

### 4. Remaining claims and questions before closing

The buyer should ask Dave for:

- The failed-backup incident record, a successful restore test, current backup status, and evidence of 30-day retention.
- Heroku dyno/database topology, failover configuration, scaling history, and any application-level availability commitments.
- Independent uptime history supporting 99.9%, plus alert-routing and incident-response records.
- The complete customer ledger tying CRM status to payment, login, contract, pilot conversion, and cancellation dates.
- A churn calculation with numerator, denominator, period, and treatment of pilots and no-contract accounts.
- Signed agreements or recorded acceptance for the 15 no-contract accounts, and pilot conversion results.
- Evidence supporting the three enterprise contracts, renewal status, SLA performance, and concentration sensitivity.
- Test inventory for integration, payment, migration, and production paths; deployment approvals and rollback procedures.
- OAuth scopes and redirect URIs, authorization rules, session controls, SSL configuration, and payment-security responsibilities.
- DNS account ownership, transfer procedure, recovery credentials, and secondary-provider or continuity arrangements.
- Transferability and termination terms for each critical external dependency.

This report is technical claims verification. It is not a penetration test, code-quality review, acceptance test, legal opinion, or opinion on what the system should do.

=== GAP MAP ===

# FlowMetrics — claims-verification gap map

**Recommendation: Material**

- **Backups:** Daily scheduling is documented, but records show 21 days of failures, no failure alerting, and no successful backup after 2026-07-30.
- **Resilience:** The seller claims Heroku redundancy and automatic failover; the supplied configuration shows one dyno, a co-located database, no replicas, and no separate database instance.
- **Commercial quality:** The listing says 120 active accounts, but the CRM identifies 23 phantom/inactive accounts; three enterprise customers represent 60% of stated MRR.
- **Operational controls:** The claimed 99.9% uptime monitoring is not supported by any application monitor; testing records show no integration or payment-path tests.
- **Ownership and dependencies:** DNS is managed personally by the seller, and the platform depends on multiple external vendors; security-control detail remains incomplete.

**Coverage:** Roughly 70–80 micro-claims were identified across nine documents. The material reliability, architecture, operational, dependency, and commercial claims were prioritised. Churn, engagement, detailed security controls, documentation quality, and continuous subscription retention remain unresolved where the data room lacks the necessary evidence.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice