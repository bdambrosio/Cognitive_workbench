**FlowMetrics Audit: Final Report**

**Engagement Metadata**
*   **Target:** FlowMetrics (SaaS Platform)
*   **Date:** August 23, 2026
*   **Auditor:** Jill
*   **Scope:** Verification of operational, financial, and technical claims against data room documents (Docs 1–9).

### §9 Recommendation: Material

The recommendation is **Material**. This is not a case of isolated clerical errors; it is a systemic pattern of operational neglect that contradicts the core value proposition of the asset.

Ten findings are classified as [delta], meaning the evidence directly contradicts the seller's claims. The operational infrastructure is in a state of active failure: backups are failing without alerting, there is no uptime monitoring despite contractual SLAs, and the "redundant" architecture is a single point of failure. The scalability claim is architecturally false because the database is co-located with the application on a single dyno. The "no lock-in" claim is contradicted by a 40% dependency on a single vendor (DataEnrich) with no fallback mechanism.

The narrative of a "reliable, scalable, well-documented, turnkey" business is unsupported. The asset is a fragile, single-person operation with unmonitored critical failures. Valuation must reflect the cost of remediating these structural gaps, not the price of a mature, stable SaaS.

### Findings (Worst-First)

**1. Backup Failure — [delta]**

Claim (Doc 9): "The database is backed up daily to Heroku's managed storage with 30-day retention."

Evidence: Doc 4 — "Failures recorded for the last 21 days." "Last Successful Backup: 2026-07-30." "Alerting: None configured for backup failures."

Delta: The last successful backup is 24 days old. The 30-day retention window is being consumed by 21 consecutive days of failure. The claim of "daily" backups is false for the past three weeks.

**2. No Uptime Monitoring vs. SLA Obligations — [delta]**

Claim (Doc 9): "We monitor uptime at 99.9%."

Evidence: Doc 4 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)." Doc 7 — Three enterprise contracts with 99.9% or 99.5% SLAs and service credit remedies.

Delta: The seller claims to monitor uptime at the exact level required by their contracts, but no monitoring infrastructure exists. The company cannot prove SLA compliance or detect breaches.

**3. Test Coverage Misrepresentation — [delta]**

Claim (Doc 9): "The codebase is well-documented with comprehensive test coverage across all critical paths."

Evidence: Doc 3 — "Unit Tests: 12 (all located in test/utils/), Integration Tests: 0, Payment-Path Tests: 0."

Delta: 12 unit tests in a utility directory, zero integration tests, zero payment-path tests. "Comprehensive" and "all critical paths" are contradicted by the test inventory.

**4. Single Dyno vs. Redundancy Claim — [delta]**

Claim (Doc 9): "The system has redundancy and automatic failover through Heroku's platform-level process management."

Evidence: Doc 4 — "Dyno: standard-1x (1GB RAM, 0.5 CPU)." "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)." "Read Replicas: None." "Separate DB Instance: No."

Delta: A single dyno with the database on it is not redundancy. Heroku's "failover" for a single dyno is process restart, not multi-instance redundancy. The claim is false.

**5. 120 Active Customers Inflated — [partial]**

Claim (Doc 1): "We have built a loyal customer base of 120 active accounts."

Evidence: Doc 6 — "Phantom/Inactive Accounts: 23. Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60-120 days ago."

Delta: True active base is ~97 paying accounts. The 23 phantom accounts inflate the user count metric by ~19%.

**6. No CI/CD — [delta]**

Claim (Doc 1, Doc 9): Implied modern development practice; "modern Rails 7 stack."

Evidence: Doc 3 — No CI/CD pipeline configuration, no automated deployment references.

Delta: Manual deployment increases risk of human error and downtime. No automated safety net exists.

**7. No Staging Environment — [delta]**

Claim (Doc 1, Doc 9): Implied safe deployment environment.

Evidence: Doc 3 — No staging environment referenced in git history or configuration.

Delta: Code is deployed directly to production. No buffer for testing.

**8. No Branch Protection — [delta]**

Claim (Doc 1, Doc 9): Implied secure code management.

Evidence: Doc 3 — No branch protection rules in git configuration.

Delta: Any developer can push directly to main. High risk of accidental production breakage.

**9. Key Person Risk: Dave — [real, with a structural note]**

Claim (Doc 1): "Turnkey acquisition opportunity... minimal operational overhead."

Evidence: Doc 4 — "DNS Management: Managed personally by 'dave'." Doc 8 — DNS, deployment, and vendor management all attributed to one individual.

Delta: The business is not turnkey; it is a Dave-dependent operation. No documentation or redundancy for critical processes.

**10. MRR Arithmetic — [real, minor caveat]**

Claim (Doc 1): "Our blended Monthly Recurring Revenue (MRR) stands at $40,000."

Evidence: Doc 5 — Stripe MRR: 91 Pro × $149 = $13,549 (actual: $13,559); 3 Enterprise × $817 = $2,451. Doc 7 — 3 enterprise contracts × $8,000 = $24,000. True total: $40,010.

Delta: $10 discrepancy in Doc 5's own arithmetic (91 × 149 = 13,559, not 13,549). The claimed $40,000 is within $10 of the true $40,010. Materially accurate.

**11. DataEnrich.io 40% Dependency, No Fallback — [delta]**

Claim (Doc 9): "All third-party integrations are on standard SaaS agreements with no lock-in."

Evidence: Doc 8 — "Dependency: 40% of features depend on this API." "Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)." "Termination Notice: 90 days (either party)."

Delta: A 40% feature dependency with no implemented fallback and a 90-day termination notice constitutes operational lock-in. The claim of "no lock-in" is contradicted by the stated dependency and lack of fallback.

**12. Scalability Claim — [delta]**

Claim (Doc 9): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence: Doc 4 — "Database: heroku-postgresql:standard-0 (running on the same dyno as the application)." "Read Replicas: None." "Separate DB Instance: No."

Delta: Adding application dynos does not scale the database, which is co-located on the same dyno with no separate instance or read replicas. Horizontal scaling of the application tier is blocked by the database architecture.

**13. 15 No-Contract Accounts — [partial]**

Claim (Doc 7): 15 individual users on Pro plan, payment via Stripe.

Evidence: Doc 7 — "No signed ToS on file (acceptance not recorded)."

Delta: Paying customers operate without a recorded legal agreement. Revenue is real; contractual enforceability is not established.

**14. Churn Claim — [unverifiable]**

Claim (Doc 1): "The business operates with low churn, a stable base of recurring subscribers."

Evidence: Doc 6 — 23 phantom/inactive accounts. Doc 7 — 8 free pilots expiring Q3 2026. No cohort data or retention metrics in any document.

Delta: Actual churn rate cannot be computed from the data provided. The claim is neither supported nor refuted.

**15. Revenue-Positive Claim — [unverifiable]**

Claim (Doc 1): "FlowMetrics is a revenue-positive, recurring-revenue SaaS platform."

Evidence: No P&L or income statement in the data room. Visible costs from Doc 8: ~$587/mo. Implied revenue ~$40,000/mo.

Delta: Revenue-positive status cannot be verified without a complete P&L. Total cost structure (personnel, marketing, other) is unknown.

**16. GoDaddy Single-Person DNS — [real, with a structural note]**

Claim (Doc 4): "DNS Management: Managed personally by 'dave'. No secondary DNS provider."

Evidence: Doc 8 — "If domain expires or GoDaddy has an issue, the app is unreachable."

Delta: Key-person risk on critical infrastructure. No redundancy exists.

**17. "Turnkey" / "Minimal Operational Overhead" — [delta]**

Claim (Doc 1): "This is a turnkey acquisition opportunity... with minimal operational overhead."

Evidence: Doc 4 — 21 days backup failures, no alerting, no uptime monitor. Doc 3 — No CI/CD, no staging, no branch protection. Doc 8 — Single-person DNS, single dyno.

Delta: Operational overhead is not minimal. Backup failures are unmonitored, no uptime monitoring exists, no CI/CD, no staging, single-point-of-failure DNS. The claims are contradicted by the infrastructure state.

**18. 8 Free Pilots Expiring Q3 2026 — [real, operational caveat]**

Claim (Doc 7): 8 pilot agreements, free 90-day trial, expiring Q3 2026.

Evidence: Doc 7 — "Payment Obligation: None." "Conversion: Optional conversion to Pro plan ($149/mo)."

Delta: MRR is not inflated (they are free). Near-term pipeline includes 8 zero-commitment accounts expiring this quarter with no documented conversion process.

**19. Google OAuth No-Contract — [non-delta]**

Claim (Doc 8): "If Google changes API or revokes app, users cannot log in. No contractual relationship."

Evidence: Doc 8 — Risk acknowledged in the dependency list.

Delta: None. Standard SaaS authentication risk. No seller claim is contradicted.

**20. Heroku SLA vs. App SLA — [real, operational caveat]**

Claim (Doc 8): "SLA: 99.95% platform uptime (not contractual for the app)."

Evidence: Doc 7 — Three enterprise contracts with 99.9%/99.5% SLAs. Doc 4 — No uptime monitoring.

Delta: The platform provides 99.95% uptime, but this is not a contractual SLA for the application. The company has contractual SLAs with customers but no mechanism to verify compliance.

### Coverage Statement

**Coverage:** 20 of ~50 claims individually verified (40%). The remaining ~30 are micro-claims (specific pricing, SSL auto-renewal, Rails version) or require materials not in the data room (source code, P&L, cohort data) — assessed low-risk given the severity distribution of what was found.

**What was NOT checked:**
*   Specific dollar amounts for minor expenses (e.g., exact Stripe fee breakdown per transaction).
*   SSL certificate auto-renewal status (Doc 4 states it is Heroku-managed; not independently verified).
*   Specific Rails version compatibility (requires source code).
*   The "real-time" nature of data processing (requires code review).
*   "Proven product-market fit" (requires market analysis and churn data).
*   Full personnel and marketing costs (requires P&L).

**Why it matters:**
The absence of source code prevents verification of technical claims like "real-time" processing. The absence of full P&Ls prevents verification of net income. This gap limits the audit to structural and operational claims, not deep technical or financial performance. The severity distribution of the 20 findings (10 deltas, 2 partial, 2 unverifiable, 4 real with caveats, 1 non-delta, 1 real minor caveat) indicates that the remaining micro-claims are unlikely to change the Material recommendation.

### Questions for the Seller Before Closing

1.  **Backup Restoration Test:** Provide a timestamped log of a successful backup restoration in the last 30 days. If none exists, what is the plan to remediate the 21-day failure streak?
2.  **Database Scaling Plan:** How will you scale the database if user load increases? What is the cost and timeline to move to a separate DB instance?
3.  **DataEnrich Fallback:** What is the specific technical plan to implement a fallback? If you cannot build it, what is the cost of negotiating a better contract or finding an alternative vendor?
4.  **Contractual Enforcement:** For the 15 Pro accounts without signed ToS, what is the legal strategy for enforcing payment terms? Will you require retroactive signing?
5.  **Key Person Mitigation:** If Dave leaves, who takes over DNS, code deployment, and vendor management? Is there documentation for these processes?
6.  **Churn Data:** Provide the last 12 months of cohort retention data. Without this, the "low churn" claim is a guess.
7.  **Full P&L:** Provide a complete P&L for the last 12 months, including all personnel, marketing, and software costs.

### Executive Synthesis

FlowMetrics is presented as a stable, scalable SaaS, but the audit reveals a fragile, single-person operation. The core infrastructure is failing: backups are broken, there is no monitoring to detect outages, and the architecture cannot scale beyond its current limits. The "no lock-in" claim is false due to a heavy dependency on one vendor with no backup plan. While the revenue appears real, the operational risks are high. The buyer should not pay for a "turnkey" business; they are buying a project that requires immediate, significant investment in infrastructure remediation and process building. The valuation should reflect this remediation cost.
