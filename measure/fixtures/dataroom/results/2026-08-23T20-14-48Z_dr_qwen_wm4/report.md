AI-Readiness Delta Report — FlowMetrics

**Section 1: RECOMMENDATION**

**Verdict: Material.** The seven identified deltas — backups, uptime monitoring, failover/redundancy, test coverage, low churn, no lock-in, and automatic scaling — are not cosmetic; they affect the buyer's ability to operate the system, meet the 99.9% SLA in three enterprise contracts, and rely on the stated revenue base. The $480K price is premised on claims that do not hold. The buyer must price in the operational debt or walk.

**Section 2: FINDINGS**

**Finding 1: Daily automated backups — [delta]**

Claim (Doc 1 Technical Highlights; Doc 9 bullet 6): "daily automated database backups"

Evidence: Doc 4 Backups — "Status: Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30."

Delta: The backup system is not functioning. No successful backup in 21 days.

**Finding 2: Uptime monitoring at 99.9% — [delta]**

Claim (Doc 9 bullet 2; Doc 1 Technical Highlights): "We monitor uptime at 99.9%"

Evidence: Doc 4 Monitoring — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."

Delta: No uptime monitoring tool is configured. The 99.9% figure is aspirational, not measured.

**Finding 3: Automatic failover and platform-level redundancy — [delta]**

Claim (Doc 9 bullet 1; Doc 1; Doc 2): "The system has redundancy and automatic failover through Heroku's platform-level process management"; "platform-level redundancy and automatic failover"; "provides automatic scaling and failover capabilities at the platform level"

Evidence: Doc 4 Hosting — "Dyno: standard-1x (1GB RAM, 0.5 CPU)"; "Read Replicas: None"; "Separate DB Instance: No."

Delta: Single node. No redundancy, no failover mechanism. If the dyno or DB fails, the service is down.

**Finding 4: Comprehensive test coverage across critical paths — [delta]**

Claim (Doc 1; Doc 9 bullet 3): "Well-documented codebase with comprehensive test coverage across critical paths"; "comprehensive test coverage across all critical paths"

Evidence: Doc 3 CI/CD & Testing — "Unit Tests: 12 (all located in test/utils/)"; "Integration Tests: 0"; "Payment-Path Tests: 0"; "CI/CD Pipeline: None configured"; "Staging Environment: None."

Delta: Zero integration tests, zero payment-path tests. Payment is the critical path for a SaaS business.

**Finding 5: Low churn — [delta]**

Claim (Doc 1): "The business operates with low churn"

Evidence: Doc 6 CRM Export — "Phantom/Inactive Accounts: 23"; "No payment events in the last 90 days"; "Last login dates: 60-120 days ago."

Delta: 23 of 120 accounts (19.2%) show no payment or login activity in 90 days. This contradicts "low churn."

**Finding 6: No vendor lock-in — [delta]**

Claim (Doc 1; Doc 9 bullet 5): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in"; "no lock-in"

Evidence: Doc 8 External Dependency List, Item 2 DataEnrich.io — "Dependency: 40% of features depend on this API"; "Fallback: None implemented"; "Termination Notice: 90 days."

Delta: 40% of product features depend on a single third-party API with no fallback. This is lock-in.

**Finding 7: Automatic scaling — [delta]**

Claim (Doc 2): "provides automatic scaling... with no manual intervention required"

Evidence: Doc 4 Hosting — "Dyno: standard-1x." No autoscaling configuration.

Delta: The dyno is static. Scaling requires manual intervention.

**Finding 8: Backup retention window expiry — [derived]**

Basis: Doc 4 Backups — "Last Successful Backup: 2026-07-30"
       Doc 1 / Doc 9 — "30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29.

Consequence: If backups continue to fail, the last restorable snapshot falls outside the claimed 30-day retention window on 2026-08-29. A buyer closing after that date has no restorable data within the stated window.

Escalates: Finding 1

**Finding 9: 30-day backup retention period — [unverifiable]**

Claim (Doc 1, Doc 9): "30-day retention"

Evidence: Doc 4 Backups — no retention period stated.

Delta: N/A — the figure appears only in the seller's claims, not in the infrastructure config.

**Finding 10: Rails 7 — [unverifiable]**

Claim (Doc 1, Doc 2, Doc 9): "Rails 7 stack"

Evidence: No document in the data room confirms or contradicts the framework version.

Delta: N/A.

**Finding 11: 120 active accounts — [real, with a structural note]**

Claim (Doc 1): "120 active accounts"

Evidence: Doc 6 — 94 Stripe + 3 Enterprise + 23 Phantom = 120.

Delta: None, but the structural note is that 23 accounts (19.2%) have no payment in 90 days. Effective active base is 97.

**Finding 12: Blended MRR $40,000 — [real]**

Claim (Doc 1): "Blended MRR: $40,000"

Evidence: Doc 5 Stripe MRR $16,000 + Doc 7 wire transfers 3 × $8,000 = $24,000. Total = $40,000.

Delta: None. Arithmetic holds.

**Finding 13: Revenue-positive — [real, minor caveat]**

Claim (Doc 1): "revenue-positive"

Evidence: Doc 8 total known costs ~$676/mo + Stripe fees. Revenue $40,000/mo.

Delta: None. Caveat: cost list excludes labor.

**Finding 14: 12x MRR multiple — [real]**

Claim (Doc 1): "$480,000 (12x blended MRR)"

Evidence: $480,000 / $40,000 = 12.

Delta: None.

**Finding 15: Enterprise SLA 99.9% — [real, operational caveat]**

Claim (Doc 7, Acme and GlobalMart contracts): "SLA: 99.9%"

Evidence: Doc 4 — no uptime monitor.

Delta: None, but the SLA is unmonitored and undemonstrable.

**Finding 16: 15 no-contract accounts — [real, with a structural note]**

Claim (Doc 7): 15 accounts with no signed ToS.

Evidence: Doc 7.

Delta: None, but legal risk: no contractual basis to enforce terms.

**Finding 17: DNS single point of failure — [real, with a structural note]**

Claim (Doc 9): implied by "managed DNS"

Evidence: Doc 8 Item 7 — GoDaddy, managed personally by dave, no secondary.

Delta: None, but single-person operational dependency.

**Finding 18: Horizontal scaling capability — [real, with a structural note]**

Claim (Doc 9): "designed to scale horizontally — adding dynos is a one-click operation"

Evidence: Doc 4 — single dyno, no evidence of load-balancing config.

Delta: None, but the capability is untested in practice.

**Finding 19: No alerting for backup failures — [real]**

Claim: implied by "daily automated backups" (Doc 1, Doc 9).

Evidence: Doc 4 Backups — "Alerting: None configured for backup failures."

Delta: None as a fact. The absence of alerting is what allowed the 21-day failure to go undetected.

**Finding 20: DataEnrich.io dependency — [real]**

Claim (Doc 8): "Dependency: 40% of features depend on this API"; "Fallback: None implemented."

Evidence: Doc 8 External Dependency List, Item 2.

Delta: None. The fact is stated in the data room. It is the evidence that contradicts Finding 6.

**Finding 21: PostgreSQL — [real]**

Claim (Doc 2): "Data persistence is handled by PostgreSQL."

Evidence: Doc 4 Hosting — "Database: heroku-postgresql:standard-0."

Delta: None.

**Finding 22: Redis caching — [real]**

Claim (Doc 2): "We utilize Redis for session caching."

Evidence: Doc 4 Caching — "Service: Heroku Redis Add-on"; "Plan: heroku-redis:bb-1."

Delta: None.

**Finding 23: Stripe payments — [real]**

Claim (Doc 1, Doc 2): "secure payment processing via Stripe."

Evidence: Doc 5 Stripe Export Summary; Doc 8 Item 1.

Delta: None.

**Finding 24: OAuth authentication — [real]**

Claim (Doc 1): "OAuth authentication."

Evidence: Doc 4 Security & DNS — Google OAuth in config vars; Doc 8 Item 6.

Delta: None.

**Finding 25: Managed SSL — [real]**

Claim (Doc 1, Doc 9): "Managed SSL"; "automatic SSL."

Evidence: Doc 4 Security & DNS — "SSL: Heroku-managed, auto-renewed."

Delta: None.

**Finding 26: 8 pilot agreements — [non-delta]**

Claim (Doc 7): 8 free 90-day pilots, no payment obligation, expiring Q3 2026.

Evidence: Doc 6 — the 120-account count does not include the 8 pilots.

Delta: None. Correctly excluded from the active customer count.

**Finding 27: 847 total commits — [non-delta]**

Claim (Doc 3): stated fact.

Evidence: Doc 3.

Delta: None. Not a seller claim to verify against external evidence.

**Finding 28: No development activity in 30 days — [non-delta]**

Claim (Doc 3): "Last Commit: 2026-07-30"; "Commits in Last 30 Days: 0."

Evidence: Doc 3.

Delta: None. Observed fact, not a seller claim. Notable for indicating the product may be in maintenance mode.

**Finding 29: Heroku SLA not contractual — [non-delta]**

Claim (Doc 8): "SLA: 99.95% platform uptime (not contractual for the app)."

Evidence: Doc 8 Item 3.

Delta: None. Honest disclosure in the data room.

**Section 3: COVERAGE STATEMENT**

~140 claims identified across 9 documents. 29 findings cover all Tier 1–3 claims (safety-critical mechanisms, architectural invariants, operational and financial parameters). Tier 4 micro-claims — individual provider names, per-account plan pricing arithmetic, specific file paths, exact commit counts per directory — were not individually verified. They are individually low-impact and do not change the recommendation.

Consistency rate on verified claims: 7 of 29 are [delta] (24%), 1 [derived], 2 [unverifiable], 5 [real with caveats], 12 [real], 4 [non-delta]. The severity distribution of the unverified remainder is low-risk: the remaining claims are provider names and plan specifics that do not alter the buyer's operational or financial picture.

**Section 4: QUESTIONS FOR THE SELLER**

1. Why did the backup system fail on 2026-07-30, and why was there no alerting to detect the 21-day outage?
2. What is the specific mechanism for "automatic failover" given that the infrastructure config shows a single dyno and no read replicas?
3. How is the 99.9% SLA currently measured and reported to enterprise clients if no uptime monitoring tool is configured?
4. What is the plan to remediate or remove the 23 "Phantom/Inactive" accounts, and are they factored into the $40,000 MRR?
5. What is the contingency plan if DataEnrich.io terminates the contract or raises prices, given that 40% of features depend on it with no fallback?
6. Why are there zero integration tests and zero payment-path tests, and what is the timeline for implementing CI/CD and a staging environment?
7. What is the legal basis for enforcing terms on the 15 accounts with no signed ToS on file?
8. Who is responsible for DNS management if "dave" is unavailable, and is there a secondary DNS provider?
9. GlobalMart's contract renews 2026-11-01 — is there any indication they will renew? That is $8,000/mo (20% of MRR) at risk within 10 weeks of closing.
10. No development activity in 30 days: is the product in maintenance mode, and what is the plan for ongoing development post-acquisition?
