# Technical Claims Audit Report — FlowMetrics

**As-of date for materials: 2026-08-25**

## Recommendation: Material

Of the **18 claims examined individually out of 62 identified**, the materials show several material gaps affecting revenue representation, recoverability, availability, dependency concentration, and operational control. The remaining 44 claims were not individually tested because they are predominantly lower-priority product, marketing, and implementation-detail claims; this is limited assurance, not a conclusion about those claims.

## Findings

**Finding 1: Stated MRR and active-customer base are materially overstated — [delta]**

Claim (doc1_seller_listing_description.md:9, 12-13): “Our blended Monthly Recurring Revenue (MRR) stands at $40,000” and “Active Customers: 120.”

Evidence: doc5_stripe_export_summary.md:3-19 — Stripe reports “Total Active Subscriptions: 94” and “Total MRR (Stripe): $16,000”; three enterprise companies have separate $8,000/month wire-transfer contracts not included in Stripe. doc6_crm_export_summary.md:3-18 — the CRM marks 120 accounts active, but identifies 23 as “Phantom/Inactive Accounts” with no payment events in the last 90 days and last logins 60–120 days ago.

Gap: The stated $40,000 MRR is not supported by the Stripe and contract records supplied. The 120-account figure includes 23 accounts described by the evidence as phantom/inactive. The three wire contracts explain $24,000 of revenue outside Stripe, but do not reconcile the stated $40,000 figure: $16,000 + $24,000 = $40,000, while the customer count remains materially misrepresented.

**Finding 2: Recoverability is materially impaired and the last recoverable backup has a defined expiry — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6, 10): “We ... maintain daily automated database backups” and “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:15-19 — backups are scheduled daily, but “Failures recorded for the last 21 days,” the “Last Successful Backup” was “2026-07-30,” and “None configured” for backup-failure alerting.

Gap: The schedule exists, but successful daily recoverability is not evidenced. Backup failures persisted for 21 days without alerting.

**Finding 3: Last successful backup reaches the stated retention boundary on 2026-08-29 — [derived]**

Basis: doc4_infrastructure_config.md:18 — “Last Successful Backup: 2026-07-30.”
       doc9_seller_s_technical_claims_verbatim.md:10 — “30-day retention.”

Derivation: 2026-07-30 plus 30 days equals 2026-08-29. On 2026-08-29, absent a newer successful backup, the last known recoverable backup ages beyond the stated 30-day retention window.

Consequence: After 2026-08-29, the materials no longer evidence a recoverable database state within the seller's stated retention period. This is a time-sensitive consequence of the backup failure, not a separate seller claim.

Escalates: Finding 2.

**Finding 4: Redundancy and automatic failover are not supported by the deployment configuration — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:5): “The system has redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4_infrastructure_config.md:3-9 — the application uses one `standard-1x` dyno; PostgreSQL is “running on the same dyno as the application”; “Read Replicas: None”; and “Separate DB Instance: No.”

Gap: The supplied configuration shows a single application/database placement with no read replicas or separate database instance. It does not establish the claimed redundancy or automatic failover for the application and data path.

**Finding 5: The claimed 99.9% uptime monitoring is absent — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:6): “We monitor uptime at 99.9%.”

Evidence: doc4_infrastructure_config.md:21-23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)” and only the Heroku built-in status page is listed.

Gap: No application-level uptime monitor or measurement record was supplied. The Heroku status page is not evidence that FlowMetrics monitored or achieved 99.9% uptime.

**Finding 6: Comprehensive critical-path test coverage is not supported — [delta]**

Claim (doc9_seller_s_technical_claims_verbatim.md:7): “The codebase is well-documented with comprehensive test coverage across all critical paths.”

Evidence: doc3_git_history_summary.md:24-32 — no CI/CD pipeline; 12 unit tests, all in `test/utils/`; zero integration tests; zero payment-path tests; no staging environment; no branch protection; and no documented code-review process.

Gap: The evidence contradicts “comprehensive” coverage for integration and payment paths and does not establish coverage of critical paths.

**Finding 7: The DataEnrich dependency has no fallback and affects 40% of features — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:9): “All third-party integrations are on standard SaaS agreements with no lock-in.”

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supports product matching, price comparison, and category classification; “40% of features depend on this API”; “Fallback: None implemented”; and if it is unavailable or pricing changes, the product degrades to basic mode.

Gap: The agreement is described as standard with 90 days' termination notice, but the implementation has material functional dependence and no fallback. “No lock-in” is therefore incomplete as an operational description.

**Finding 8: DNS control is concentrated in the seller personally — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:11): “We use a modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4_infrastructure_config.md:25-29 — SSL is Heroku-managed and auto-renewed, but DNS is GoDaddy, “Managed personally by 'dave',” with “No secondary DNS provider.” doc8_external_dependency_list.md:29-32 — if the domain expires or GoDaddy has an issue, “the app is unreachable.”

Gap: Automatic SSL is evidenced, but DNS is not independently managed or redundant; access and availability depend on the seller's personal control.

## Claims that held or had non-material caveats

The materials support the existence of a recurring SaaS business, three enterprise contracts at $8,000/month each, Heroku hosting, PostgreSQL, Redis, Stripe payment processing, and Heroku-managed SSL. These observations do not cure the material gaps above. The evidence also indicates that Redis failure leaves the application functional but slower (doc8_external_dependency_list.md:17-20), and that Twilio failure affects notifications rather than core functionality (doc8_external_dependency_list.md:21-24).

## Coverage

The frozen claim surface contains 62 claims: doc1 contains 29, doc2 contains 19, and doc9 contains 14. I examined 18 claims individually, prioritising business-ending claims, architectural invariants, and operational parameters. I stopped with 44 claims remaining, principally lower-risk product, marketing, and implementation-detail assertions that were not necessary to resolve the material revenue, recoverability, availability, dependency, and control risks. The unchecked claims remain outside this conclusion.

## Questions for the seller before closing

1. Provide bank statements, invoices, and signed contracts reconciling the stated $40,000 MRR, including the three wire-transfer customers.
2. Explain why 23 CRM accounts with no recent payment events or logins are marked active, and provide the customer-count definition used in the listing.
3. Provide successful backup logs, restore-test results, and the current backup status; specifically explain the 21-day failure period and the state as of 2026-08-29.
4. Demonstrate the actual failover topology and recovery procedure for both the application and PostgreSQL data.
5. Provide historical uptime measurements supporting 99.9%, rather than a Heroku platform-status reference.
6. Provide test inventory and results for payment, integration, authentication, and other stated critical paths.
7. Provide the DataEnrich agreement, transferability terms, rate-change provisions, and a plan for service interruption or termination.
8. Confirm transfer of GoDaddy, Heroku, Stripe, OAuth, Twilio, DataEnrich, repository, and backup ownership and credentials without continuing personal dependency on Dave.

## Proposed method-file edits for review

- Add a mandatory reconciliation worksheet for claims involving multiple revenue or customer systems before assigning a verdict.
- Require every derived date to be recalculated against the report's stated as-of date and labelled with its expiry date.
- For availability claims, distinguish platform SLA/status information from application-level monitoring evidence.
- Treat personal ownership of DNS, credentials, or recovery controls as an architectural-control claim requiring explicit transfer evidence.

=== LIMITATIONS ===
Materials examined: doc1–doc9 in the FlowMetrics data room; materials as of 2026-08-25.
The seller was not consulted and has not confirmed your reading of their own claims.
This is a limited-assurance engagement resting on 18 individually examined claims out of the frozen 62-claim surface; it is not a full audit, penetration test, code-quality review, legal opinion, or acceptance test.

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS platform

**Recommendation: Material**

- **MRR and customer count:** Listing states $40,000 MRR and 120 active customers; evidence reconciles the revenue only by adding three wire contracts and identifies 23 phantom/inactive CRM accounts.
- **Data recoverability:** Daily backup claim is undermined by 21 days of failures, no alerting, and no successful backup after 2026-07-30; the stated 30-day window reaches its boundary on 2026-08-29.
- **Availability architecture:** Claimed redundancy and failover are not supported by a single dyno/database placement with no replicas or separate DB instance.
- **Uptime and testing:** No uptime monitor exists, and the repository has no integration or payment-path tests despite the comprehensive-coverage claim.
- **Dependency and control risk:** DataEnrich supports 40% of features with no fallback; DNS is personally managed by Dave with no secondary provider.

**Coverage:** 18 of 62 claims checked individually, prioritising business-ending, architectural, and operational claims; 44 lower-priority claims remain unchecked.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*