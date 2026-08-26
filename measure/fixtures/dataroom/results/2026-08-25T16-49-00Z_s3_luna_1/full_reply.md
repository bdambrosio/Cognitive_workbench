Recommendation: **Material**

FlowMetrics’ stated claims are materially contradicted by the implementation in backup recovery, redundancy, testing, uptime monitoring, payment security, and dependency posture. The materials support 15 assessed claims out of a frozen surface of 286. The remaining 271 claims were not examined. This is limited assurance over the disclosed subset only.

**Finding 1: Backup operation — [delta]**

Claim (D1:L19; D9:L6/L10): Backups operate with 30-day retention.

Evidence: D4:L16-L19 — backup failures continued for 21 days; the last successful backup was 2026-07-30; no alerting is implemented.

Gap: The documented backup operation and retention claim are not supported. Failures can continue without notification.

**Finding 2: Last recoverable backup expires — [derived]**

Basis: D4:L18 — last successful backup: 2026-07-30  
       D1:L19; D9:L6/L10 — stated retention: 30 days

Derivation: 2026-07-30 + 30 days = 2026-08-29.

Consequence: The last recoverable backup ages out on 2026-08-29 if no successful backup is created before then. After that date, the stated retention window contains no successful backup.

Escalates: Finding 1

**Finding 3: Redundancy and failover — [delta]**

Claim (D1:L18-L22; D9:L5-L10): FlowMetrics has redundant infrastructure and failover capability.

Evidence: D4:L6-L8 — the database shares the application dyno; there are no database replicas and no separate database.

Gap: The observed deployment does not provide the stated separate or redundant database architecture or demonstrated failover.

**Finding 4: Comprehensive testing — [delta]**

Claim (D1:L21; D9:L7): The system has comprehensive automated testing.

Evidence: D3:L20-L32 — tests cover utilities only; there are zero integration or payment-path tests, no staging environment, no CI/CD, no branch protection, and no recorded review control.

Gap: The evidence does not support comprehensive testing. Critical paths are outside the tested surface.

**Finding 5: Uptime monitoring — [delta]**

Claim (D1:L19; D9:L6): Uptime is monitored.

Evidence: D4:L21-L23 — no uptime monitoring or alerting is implemented.

Gap: The claimed monitoring is absent.

**Finding 6: Payment security — [partial]**

Claim (D1:L20; D2:L3-L6): Payment handling is secure and production-ready.

Evidence: D4:L25-L29 — 14 secrets are stored without a vault; D3:L27-L29 — payment paths have zero tests.

Gap: Managed SSL and payment records do not establish secure payment handling. Secret management and payment-path verification are material gaps.

**Finding 7: No lock-in — [partial]**

Claim (D1:L22; D9:L9): The business has no vendor lock-in.

Evidence: D8:L7-L12 — DataEnrich supplies 40% of the relevant data and no fallback provider is documented.

Gap: The dependency has material concentration risk and no demonstrated fallback. The no-lock-in claim is only partially supported.

**Finding 8: Active-customer representation — [partial]**

Claim (D1:L12-L15): The stated customer base represents active customers.

Evidence: D6:L3-L18 — 23 accounts are identified as phantom or inactive; D7:L29-L45 — eight accounts are free pilots and fifteen accounts lack recorded signed Terms of Service acceptance.

Gap: The customer listing contains inactive or unconfirmed accounts. Low churn is unverifiable, not contradicted by these materials. The evidence does not establish that all customer relationships are pilots.

**Finding 9: Google OAuth — [real, operational caveat]**

Claim (D8:L25-L28): Google OAuth is supported.

Evidence: D8:L25-L28 — the materials record Google OAuth support.

Gap: None. The OAuth implementation details were not independently verified.

**Finding 10: GoDaddy DNS — [real, operational caveat]**

Claim (D8:L29-L32): GoDaddy DNS is used.

Evidence: D8:L29-L32 — the materials identify GoDaddy DNS.

Gap: None. DNS configuration and operational resilience were not independently verified.

**Finding 11: Rails 7 and Heroku — [real]**

Claim (D2:L3-L6): FlowMetrics uses Rails 7 on Heroku.

Evidence: D2:L3-L6 — the stated stack is Rails 7 and Heroku.

Gap: None.

**Finding 12: Managed SSL — [real, minor caveat]**

Claim (D4:L25-L29): SSL is managed.

Evidence: D4:L25-L29 — managed SSL is recorded.

Gap: None on the claim examined. Certificate renewal and deployment controls were not separately tested.

**Finding 13: Stripe records — [real]**

Claim (D5:L4-L5; D5:L17-L19): Stripe records support the stated payment information.

Evidence: D5:L4-L5; D5:L17-L19 — Stripe records contain the stated payment data.

Gap: None.

**Finding 14: Enterprise contracts — [real, minor caveat]**

Claim (D7:L3-L27): Enterprise contracts exist on the stated terms.

Evidence: D7:L3-L27 — the materials contain the listed enterprise contract records.

Gap: None on existence. Contractual performance and feature delivery were not independently verified.

**Finding 15: MRR arithmetic — [real, operational caveat]**

Claim (D5:L4-L5; D1:L12-L15): The stated MRR is $40,000, based on $16,000 + 3 × $8,000.

Evidence: D5:L4-L5 — $16,000 plus three $8,000 contracts equals $40,000; D1:L12-L15 — stated MRR is $40,000; D1:L5 — asking price is $480,000.

Gap: None in the arithmetic. Operational reconciliation of complete MRR, churn, and account status remains incomplete.

Coverage: The frozen claim surface is 286 claims: 188 in D1-D6, 46 in D7, 36 in D8, and 16 in D9. Fifteen claims were assessed and 271 were unexamined. The prior total of 279 was an arithmetic error and is corrected here. Unexamined claims include scaling, automatic failover beyond the architectural contradiction, OAuth implementation, secure Stripe implementation beyond the stated caveat, PostgreSQL and Redis implementation, DataEnrich implementation, contractual feature delivery, SLA performance, documentation, complete MRR and churn reconciliation, product-market fit, engagement, and no-manual-intervention claims. These remain unverifiable from the available materials, not deltas. The assessed sample includes multiple material failures, so the remainder cannot be treated as low-risk.

Before closing, the buyer should ask the seller to:

- Produce a successful backup and restore record after 2026-07-30, with retention and alerting evidence.
- Demonstrate database redundancy, failover, recovery objectives, and separation from the application dyno.
- Provide integration and payment-path test results, deployment controls, staging evidence, and review records.
- Identify a DataEnrich replacement or quantify the cost and timing of migration.
- Reconcile active customers, pilots, terms of service, MRR, churn, and delivered contractual features.
- Explain the 14 secrets held in Heroku configuration variables and provide evidence supporting secure payment implementation.

=== LIMITATIONS ===
Materials examined: nine FlowMetrics data-room documents (D1-D9), as of 2026-08-25.
The seller was not consulted and has not confirmed the auditor's reading of their own claims.
This is limited assurance over the disclosed subset only: 15 of 286 claims were assessed, and 271 were unexamined.

=== GAP MAP ===
FlowMetrics — SaaS data and payment platform

Recommendation: **Material**

Key items:
- **Backup recovery:** 21 consecutive failure days, with no alerting; the last successful backup reaches the stated retention boundary on 2026-08-29.
- **Redundancy:** The database shares the application dyno, with no replicas or separate database; the documented failover claim does not hold.
- **Testing:** Utility-only tests leave integration and payment paths untested.
- **Payment and vendor risk:** 14 secrets are stored without a vault, and DataEnrich supplies 40% of relevant data without fallback.
- **Customer records:** 23 accounts are identified as phantom or inactive; eight are free pilots and fifteen lack recorded signed Terms of Service acceptance.

Coverage: 15 of 286 claims assessed; 271 unexamined. The assessed findings include material contradictions, so the remainder is not low-risk by default.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice