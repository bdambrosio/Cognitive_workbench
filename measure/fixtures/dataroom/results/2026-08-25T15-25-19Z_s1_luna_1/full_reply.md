Recommendation: Material

FlowMetrics has material deltas in backup operation, uptime monitoring, critical-path testing, and external control-point dependence. The supplied infrastructure evidence does not demonstrate the claimed redundancy and automatic failover. The stated $40,000 MRR reconciles to the supplied Stripe and wire-payment figures. Fifteen claims were individually verified out of a frozen surface of 273. Coverage is limited; unexamined claims may contain additional material gaps.

**Finding 1: Backup retention and daily backup claims — [delta]**

Claim (doc1_seller_listing_description.md:18-20): FlowMetrics performs daily automated backups with 30-day retention.

Evidence: doc4_infrastructure_config.md:15-19 — backup failures have lasted 21 days; the last successful backup was 2026-07-30; no backup-failure alerting exists.

Delta: The evidence contradicts the operational claim. A 21-day failure period is not consistent with daily automated backups operating as stated. The evidence does not support the claimed 30-day retained recovery position at the audit date.

**Finding 2: Redundancy and automatic failover — [partial]**

Claim (doc1_seller_listing_description.md:18-22; doc9_seller_s_technical_claims_verbatim.md:5-11): The system has redundancy and automatic failover.

Evidence: doc4_infrastructure_config.md:3-8 — one standard-1x Heroku dyno runs the application, with PostgreSQL on the same dyno; there are no read replicas or separate database infrastructure.

Delta: The observed deployment has a single application and database hosting point. The supplied infrastructure evidence does not demonstrate the claimed redundancy or automatic failover. This is a material evidence gap; the observed topology alone does not establish that no failover mechanism exists elsewhere.

**Finding 3: Uptime monitoring — [delta]**

Claim (doc1_seller_listing_description.md:18-20; doc9_seller_s_technical_claims_verbatim.md:5-6,10): FlowMetrics provides 99.9% uptime monitoring.

Evidence: doc4_infrastructure_config.md:21-23 — no uptime monitor exists.

Delta: The claimed monitoring is absent in the examined infrastructure.

**Finding 4: Comprehensive critical-path test coverage — [delta]**

Claim (doc1_seller_listing_description.md:18-22): FlowMetrics has comprehensive critical-path test coverage.

Evidence: doc3_git_history_summary.md:24-32 — there are 12 unit tests, confined to `test/utils`; there are zero integration or payment-path tests.

Delta: The evidence does not support comprehensive critical-path coverage.

**Finding 5: No vendor lock-in — [delta]**

Claim (doc1_seller_listing_description.md:18-22; doc9_seller_s_technical_claims_verbatim.md:5-11): FlowMetrics has no vendor lock-in.

Evidence: doc8_external_dependency_list.md:7-12 — DataEnrich.io supports 40% of features and has no fallback. doc8_external_dependency_list.md:25-28 — loss or revocation of Google OAuth prevents login, and there is no contractual relationship. doc8_external_dependency_list.md:29-32 — GoDaddy is the sole DNS provider; outage or expiry makes the application unreachable.

Delta: Material operation depends on external control points without the stated fallback or secondary-provider protections. The claim is false as stated.

**Finding 6: 120 active accounts — [partial]**

Claim (doc1_seller_listing_description.md:9,12-13): FlowMetrics has 120 active accounts.

Evidence: doc6_crm_export_summary.md:3-18 — the CRM records 120 accounts: 94 Stripe-linked, 3 wire-transfer enterprise accounts, and 23 accounts with no payment events in 90 days.

Delta: The count of 120 is present, but 23 accounts have no payment events in 90 days on the supplied evidence. The count does not establish 120 active paying accounts.

**Finding 7: MRR reconciliation — [real]**

Claim (doc1_seller_listing_description.md:9,12-13): FlowMetrics has $40,000 blended MRR.

Evidence: doc5_stripe_export_summary.md:3-5 — 94 active Stripe subscriptions produce $16,000 MRR. doc5_stripe_export_summary.md:12-19 — three enterprise companies pay $8,000 per month each by wire. doc6_crm_export_summary.md:3-18 — the CRM records the same 94 Stripe-linked and 3 wire-transfer enterprise accounts.

Delta: None. $16,000 + (3 × $8,000) = $40,000.

**Finding 8: Backup expiry date — [derived]**

Basis: doc4_infrastructure_config.md:17-19 — “last successful backup was 2026-07-30” and “failures have lasted 21 days.”  
doc1_seller_listing_description.md:18-20 — “30-day retention.”

Derivation: 2026-07-30 + 30 days = 2026-08-29.

Consequence: On 2026-08-29, the last successful backup reaches the stated 30-day retention boundary. If no later successful backup exists, that recovery point expires on that date.

Escalates: Finding 1.

Coverage statement: The frozen claim surface contains exactly 273 claims across nine documents, as of 2026-08-25. Fifteen claims were individually verified. The remaining 258 claims were not individually verified and are not treated as deltas. They include claims about scaling, OAuth, Stripe, contracts, SLAs, pilots, account terms, and other matters. Coverage is limited and does not support a conclusion about the full surface.

Remaining Claims: The materials do not settle the horizontal-scaling claim, the full OAuth and Stripe implementation claims, detailed SLA and remedy obligations, eight free pilots expiring in Q3 2026, or 15 accounts without recorded Terms-of-Service acceptance. These remain unverified rather than deltas.

Questions for the seller:

1. What backup completed after 2026-07-30? Provide restore-test evidence.
2. Explain the 21-day failure without alerting.
3. Identify and evidence the redundancy and failover mechanism.
4. Support the horizontal-scaling and redundancy claims.
5. Identify tested fallbacks for DataEnrich.io, Google OAuth, and GoDaddy DNS.
6. Explain the 23 accounts without payment events.
7. Provide integration, payment-path, staging, and deployment-control evidence.

=== LIMITATIONS ===
Materials examined: nine FlowMetrics audit documents and verified infrastructure, account, contract, and Git evidence; as of 2026-08-25.
The seller was not consulted and has not confirmed the auditor's reading of their own claims.
This is limited assurance over a disclosed subset of the claim surface: 15 of 273 frozen claims were individually verified.

=== GAP MAP ===

Target: FlowMetrics — subscription software business with Stripe, wire-transfer enterprise accounts, Heroku infrastructure, and third-party dependencies.

Recommendation: Material

Key items:
- Backup recovery is not supported; the last successful backup was 2026-07-30, with the stated 30-day retention boundary expiring on 2026-08-29 if no later successful backup exists.
- The supplied infrastructure evidence does not demonstrate claimed redundancy or automatic failover: one dyno hosts the application and PostgreSQL.
- No uptime monitor was found despite the 99.9% monitoring claim.
- DataEnrich.io supports 40% of features without fallback, with Google OAuth and GoDaddy as additional control points.
- Testing evidence is limited to 12 utility unit tests, with no integration or payment-path tests; the stated $40,000 MRR reconciles, while 23 of 120 CRM accounts show no payment events in 90 days.

Coverage: 15 of 273 frozen claims individually verified as of 2026-08-25; the remaining 258 were not individually verified and are not treated as deltas.

Technical claims verification · not a pen-test, not legal advice.