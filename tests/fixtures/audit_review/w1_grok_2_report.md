=== CLAIM SURFACE ===
45 claims

Per-document: doc1 28; doc2 10; doc9 7. Claim sources: doc1 (listing), doc2 (tech stack), doc9 (technical claims). doc3–doc8 are evidence only.

**Recommendation: Material** (of 22 claims examined individually; 45 identified). Material [delta] and [partial] findings on recoverability, customer/MRR composition, monitoring, test coverage, DNS, and vendor lock-in. Remaining 23 are mostly marketing or restatements of the same subjects already tested.

---

**Finding 1: Daily backups with 30-day retention — [partial]**

Claim (doc1:19; also doc9:6, doc9:10, doc2:5): “99.9% uptime monitoring and daily automated database backups with 30-day retention” / “backed up daily to Heroku's managed storage with 30-day retention.”

Evidence (doc4:16–19): Schedule is daily at 2:00 AM via `heroku pg:backups schedule`. “Failures recorded for the last 21 days.” “Last Successful Backup: 2026-07-30.” “Alerting: None configured for backup failures.”

Gap: Schedule exists; the last 21 days have failed and there is no alerting. Retention of 30 days is stated in the claim, not independently confirmed in config.

**Finding 2: Last recoverable backup ages out — [derived]**

Basis: doc4:18 — “Last Successful Backup: 2026-07-30”
       doc1:19 / doc9:10 — “30-day retention”

Derivation: 30-day retention from 2026-07-30 exhausts on 2026-08-29.

Consequence: As of materials date 2026-08-25, a restore window of four days remains if the seller’s 30-day figure is true. After 2026-08-29 the last successful backup is outside stated retention.

Escalates: Finding 1.

**Finding 3: 120 active accounts — [delta]**

Claim (doc1:9, doc1:13): “loyal customer base of 120 active accounts” / “Active Customers: 120.”

Evidence (doc6:3–18): CRM “Total Accounts Marked 'Active': 120.” Breakdown: 94 Stripe-linked, 3 enterprise wire, 23 “Phantom/Inactive Accounts” created Q1 2026 with no payment events in the last 90 days and last logins 60–120 days ago. All 120 still marked active.

Gap: 23 of 120 are inactive by the CRM’s own notes. Paying population in the CRM is 97 (94+3), not 120.

**Finding 4: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1:5, doc1:9, doc1:12): Blended MRR $40,000; asking price $480,000 (12× blended MRR).

Evidence (doc5:5–19): Stripe MRR $16,000. Note: three enterprise contracts at $8,000/mo each via wire, not in Stripe. 16,000 + 24,000 = 40,000. doc7:6,14,22 confirm $8,000/mo each. 12 × 40,000 = 480,000 (arithmetic).

Gap: None on the arithmetic. Caveat: $24,000/mo (60% of claimed MRR) is three wire contracts, not Stripe; GlobalMart renews 2026-11-01 (doc7:15).

**Finding 5: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): “We monitor uptime at 99.9%.”

Evidence (doc4:22–23): “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).” “Status Page: Heroku built-in status page only.” Enterprise contracts assert 99.9% / 99.5% SLAs (doc7:8,16,24).

Gap: No application uptime monitor. Heroku’s page is platform status, not the app’s 99.9%.

**Finding 6: Platform-level redundancy and automatic failover — [partial]**

Claim (doc1:18; doc2:3; doc9:5): Rails 7 with platform-level redundancy and automatic failover / Heroku process management.

Evidence (doc4:4–8): Heroku `standard-1x` (1GB RAM, 0.5 CPU). Database `heroku-postgresql:standard-0` “running on the same dyno as the application.” “Read Replicas: None.” “Separate DB Instance: No.” doc8:16: Heroku SLA 99.95% “not contractual for the app.”

Gap: Single dyno; Postgres on the same dyno; no replicas. Process restart is not database failover.

**Finding 7: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1:21; doc9:7): “well-documented with comprehensive test coverage across critical paths” / “all critical paths.”

Evidence (doc3:24–31): CI/CD none. Unit tests: 12, all in `test/utils/`. Integration tests: 0. Payment-path tests: 0. Staging none. Branch protection none.

Gap: No tests on payment or integration paths; 12 utility unit tests only.

**Finding 8: No proprietary lock-in / standard SaaS agreements — [delta]**

Claim (doc1:22; doc9:9): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in.”

Evidence (doc8:7–12): DataEnrich.io, $400/mo, 90-day termination either party, “40% of features depend on this API,” “Fallback: None implemented.” doc8:25–28: Google OAuth, “No contractual relationship”; if revoked, users cannot log in. doc4:28: DNS managed personally by dave, no secondary.

Gap: Enrichment API is a single point with no fallback; OAuth and DNS are not substitutable on a standard agreement in the materials.

**Finding 9: Automatic SSL and managed DNS — [partial]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.” SSL also doc1:20; doc2:3 Rails 7 on Heroku.

Evidence (doc4:26–28): “SSL: Heroku-managed, auto-renewed.” “DNS Provider: GoDaddy.” “DNS Management: Managed personally by 'dave'. No secondary DNS provider.”

Gap: SSL holds. DNS is GoDaddy/dave, not Heroku-managed DNS.

**Finding 10: Horizontal scale / one-click dynos — [real, operational caveat]**

Claim (doc9:8; doc2:3): scale horizontally; adding dynos one-click; automatic scaling.

Evidence (doc4:5): one `standard-1x` dyno. Heroku can add dynos; current topology is one web dyno with DB on the same dyno (doc4:6).

Gap: Mechanism exists; present capacity is a single dyno and a co-located database.

**Finding 11: Stripe payment processing — [real]**

Claim (doc1:20; doc2:5): secure payment processing via Stripe; cards and recurring billing.

Evidence (doc5; doc8:3–6): Stripe is in use; 94 subscriptions; standard merchant fees.

Gap: None for the Stripe path. Enterprise is wire (Finding 4).

**Finding 12: Redis session caching — [real]**

Claim (doc2:5): Redis for session caching.

Evidence (doc4:11–13; doc8:17–20): Heroku Redis `bb-1`, $50/mo.

Gap: None.

**Finding 13: PostgreSQL daily scheduled backups (existence of schedule) — [real, operational caveat]**

Claim (doc2:5): PostgreSQL configured with daily scheduled backups.

Evidence (doc4:16): Daily at 2:00 AM via `heroku pg:backups schedule`.

Gap: Schedule is real; outcomes are Finding 1.

**Finding 14: Low churn — [unverifiable]**

Claim (doc1:14): “Churn: Low (stable base of recurring subscribers).”

Evidence: No churn rate, cohort, or cancellation series in doc3–doc8. 23 CRM phantoms (doc6:12) are not a churn series.

Not a finding — Remaining Claims.

**Finding 15: Revenue-positive / immediate cash flow — [unverifiable]**

Claim (doc1:9): revenue-positive; immediate cash flow.

Evidence: MRR figures exist; no P&L, costs beyond listed vendor fees (doc8). Cannot confirm profit.

Not a finding — Remaining Claims.

---

**Coverage.** 45 claims identified. 22 examined (priority 1 recoverability/payment/customers; priority 2 architecture; priority 3 backups/monitoring/ops; plus the lock-in and test claims). Stopped on remaining marketing adjectives (turnkey, defensible, proven PMF, engaged customers, pain-point wording) and duplicate restatements of MRR/customers/Rails already scored. Those leftovers do not independently change price if the scored claims stand.

Consistency among checked, non-[unverifiable] claims: several [delta]/[partial] in the first two tiers — not a clean remainder.

**Ask the seller before closing.** (1) Restore drill from the 2026-07-30 backup and backup-failure logs. (2) List of 120 CRM IDs vs paying 97. (3) Wire-transfer history for the three enterprise accounts and GlobalMart renewal status. (4) DataEnrich.io contract and any assignment clause. (5) Who holds the GoDaddy login and Heroku config vars. (6) Any uptime data backing 99.9% SLAs.

=== LIMITATIONS ===
Materials: nine documents in the bound data room (doc1–doc9), as-of 2026-08-25 (git last commit and last successful backup both 2026-07-30). Seller was not consulted and has not confirmed this reading of their own claims. Limited assurance (§1a): 22 of 45 claims examined; the recommendation applies only to those 22.
