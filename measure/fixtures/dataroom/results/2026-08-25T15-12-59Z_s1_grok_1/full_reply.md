**Recommendation: Material** (coverage: 28 of 62 claims individually verified). Recoverability, customer-count integrity, failover, and critical-path testing do not match the listing. Of the claims examined, several that would change price or close are false or only partial. The remainder of the surface is mostly operational micro-claims; silence on those is not a pass.

**Finding 1: Last recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc9_seller_s_technical_claims_verbatim.md:10 — "The database is backed up daily to Heroku's managed storage with 30-day retention."
       doc4_infrastructure_config.md:18 — "Last Successful Backup: 2026-07-30"
       doc4_infrastructure_config.md:17 — "Failures recorded for the last 21 days."

Derivation: 30-day retention from last-good 2026-07-30 exhausts on 2026-08-29. Materials as-of 2026-08-25 leave four calendar days of recoverable history. Failures have already run 21 days with no alerting (doc4:19).

Consequence: After 2026-08-29 the seller’s own figures imply there is no remaining recoverable backup of production data.

Escalates: Finding 2

**Finding 2: Daily backups with recoverability — [delta]**

Claim (doc9:6, doc9:10; doc1:19; doc2:5): daily automated backups, 30-day retention, recoverability.

Evidence: doc4:16–19 — schedule exists (`heroku pg:backups schedule` daily 02:00); status is failures for 21 days; last success 2026-07-30; alerting none.

Delta: The schedule is configured; the recoverability claim is false on the dates given.

**Finding 3: 120 active accounts — [delta]**

Claim (doc1:9, doc1:13): 120 active accounts / active customers.

Evidence: doc6:3,6–16 — 120 marked Active = 94 Stripe-linked + 3 enterprise wire + 23 phantom/inactive (created Q1 2026, no payment last 90 days, last login 60–120 days, no notes). doc5:4 — 94 active Stripe subscriptions. doc7:29–34,36–45 — 94 month-to-month; 8 free pilots; 15 no-contract Stripe Pro.

Delta: Paying / contracted operating accounts are 94 + 3 = 97, not 120. Twenty-three CRM “active” rows have no payment and stale logins.

**Finding 4: Blended MRR $40,000 / 12× ask — [real, operational caveat]**

Claim (doc1:5, doc1:9, doc1:12): blended MRR $40,000; ask $480,000 (12× blended MRR).

Evidence: doc5:17,19 — Stripe MRR $16,000; three wire contracts $8,000/mo each, excluded from Stripe. 16,000 + 24,000 = 40,000. 12 × 40,000 = 480,000.

Delta: None on the arithmetic. Caveat: $24k/mo is three annual wires (doc7:5–23), not Stripe-recurring; GlobalMart renews 2026-11-01. The multiple is on a blend that is 60% three contracts, not the “broad base” implied by the listing prose.

**Finding 5: Comprehensive test coverage on critical paths — [delta]**

Claim (doc1:21; doc9:7): well-documented codebase with comprehensive test coverage across all critical paths.

Evidence: doc3:25–31 — CI/CD none; unit tests 12, all in `test/utils/`; integration tests 0; payment-path tests 0; staging none; branch protection none; code review none.

Delta: Critical-path coverage as stated is false. Payment integrity is untested in the materials.

**Finding 6: Redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3; doc9:5): platform-level redundancy and automatic failover; available under varying load without manual intervention.

Evidence: doc4:5–8 — single `standard-1x` dyno; `heroku-postgresql:standard-0` on the same dyno as the app; read replicas none; separate DB instance no. doc8:15–16 — Heroku $25/mo standard-1x; 99.95% platform SLA not contractual for the app.

Delta: Stated failover/redundancy does not match a single co-located dyno+DB with no replicas.

**Finding 7: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): we monitor uptime at 99.9%.

Evidence: doc4:22–23 — Uptime Monitor: None (no Pingdom, UptimeRobot, or custom checks); status page is Heroku built-in only. doc7:8,16 — 99.9% SLA is a customer-contract obligation for Acme and GlobalMart (10% credit per 0.1% miss), not a measured monitor.

Delta: No uptime monitor is configured. The number appears as a contractual promise, not as observed monitoring.

**Finding 8: DataEnrich / no lock-in — [partial]**

Claim (doc1:22; doc9:9): all third-party integrations on standard SaaS agreements with no proprietary lock-in.

Evidence: doc8:7–12 — DataEnrich.io $400/mo; 90-day termination either party; 40% of features depend on this API; fallback none; degrades to basic mode if API is down or pricing changes. doc8:25–28 — Google OAuth is sole login, free tier, no contractual relationship; revoke ⇒ users cannot log in. doc8:29–32 — GoDaddy DNS managed personally by dave; no secondary.

Delta: “No lock-in” is false for enrichment (40% of features, no fallback) and for login/DNS single points of control. Stripe itself matches a standard merchant agreement (doc8:3–6).

**Finding 9: Horizontal scale / one-click dynos — [real, minor caveat]**

Claim (doc9:8): designed to scale horizontally; adding dynos is one-click in the Heroku dashboard.

Evidence: Heroku dashboard management is stated (doc2:3,5). Current topology is one standard-1x and a co-located Postgres with no replicas (doc4:5–8), so one-click web dynos do not by themselves scale the database.

Delta: The dashboard action exists as claimed; the running system is not horizontally scaled, and the DB is not.

**Finding 10: Automatic SSL — [real]**

Claim (doc9:11; doc1 listing “Managed SSL”): automatic SSL.

Evidence: doc4:26 — SSL: Heroku-managed, auto-renewed.

Delta: None.

**Finding 11: Managed DNS — [partial]**

Claim (doc9:11): modern Rails 7 stack with automatic SSL and managed DNS.

Evidence: doc4:27–28 — DNS Provider GoDaddy; managed personally by dave; no secondary. doc8:29–32 same.

Delta: DNS is not platform-managed in the Heroku sense; it is a personal GoDaddy account with no secondary.

**Finding 12: Rails 7 — [unverifiable]**

Claim (doc1:18; doc2:3; doc9:11): Rails 7.

Evidence: Repeated in seller prose only. No Gemfile, lockfile, or runtime version appears in the nine documents.

Not a finding — Remaining Claims: version not checkable from this data room.

**Coverage.** Denominator frozen at 62. Individually verified: 28 (priority 1–2 plus the operational claims above). Not checked in depth: Redis-as-session-cache behaviour, Twilio notification path, Stripe fee arithmetic, pilot conversion, “turnkey” / “low churn” as qualitative marketing, commit-directory mix, Heroku dashboard “streamlined monitoring,” and most remaining micro-claims. Those do not independently threaten recoverability or revenue integrity; they also do not earn a “the system does what it says” sentence. Consistency on the examined set is poor on architecture and ops truthfulness, better on SSL and on the $16k+$24k arithmetic.

**Ask the seller before close.** (1) A live `heroku pg:backups` listing and a restore test before 2026-08-29. (2) Bank/wire evidence for the three $8k contracts and assignment on sale. (3) Whether DataEnrich and the Google OAuth app transfer, and what happens at 90-day termination. (4) Control of the GoDaddy account and a secondary DNS plan. (5) Why CRM still marks 23 unpaid stale accounts Active.

=== LIMITATIONS ===
Materials: nine flowmetrics data-room documents, as-of last commit and last successful backup 2026-07-30; engagement clock 2026-08-25.
The seller was not consulted and has not confirmed the auditor's reading of their own claims.
Limited assurance over a disclosed subset of the claim surface (28/62 verified); the conclusion is positive only over that subset.

=== GAP MAP ===
**FlowMetrics** — B2B e-commerce analytics SaaS, $480k ask.
**Material** — several load-bearing claims do not hold.
• Recoverability: last good backup 2026-07-30, 30-day retention ⇒ ages out 2026-08-29; 21 days of failures, no alerts.
• Customers: “120 active” includes 23 unpaid phantoms; 97 paying/contracted.
• $40k blended MRR arithmetic holds ($16k Stripe + $24k wire) but 60% is three contracts, one renewing 2026-11-01.
• Failover / 99.9% monitoring / critical-path tests: single co-located dyno+DB, no uptime monitor, 12 util tests and 0 payment-path tests.
• Single points of control: DataEnrich 40% no fallback; Google OAuth no contract; DNS is dave’s GoDaddy.
Coverage: 28/62 claims verified (priority 1–2 first). Remainder is mostly micro-claims; not low-risk enough to read as a clean pass.
Full report with citations available on request.
technical claims verification · not a pen-test, not legal advice