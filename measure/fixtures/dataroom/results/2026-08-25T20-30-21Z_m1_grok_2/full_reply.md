**Recommendation: Material** (of 22 claims examined individually out of 31 frozen).

Of the 22 claims examined, several material gaps change valuation or risk: recoverability, customer/MRR composition, architectural redundancy, lock-in, and test coverage. The remaining 9 are lower-priority wording (turnkey, pain-point narrative, “revenue-positive,” “minimal overhead,” “strategic partner,” “streamlined dashboard,” “standard SaaS stack,” “focus on product,” “ease of maintenance”) and were not individually verified; those checked so far did not hold consistently, so the unchecked remainder is not treated as low-risk.

**Finding 1: Daily backups with 30-day retention — [delta]**

Claim (doc1_seller_listing_description.md:19): “99.9% uptime monitoring and daily automated database backups with 30-day retention.”
Claim (doc9_seller_s_technical_claims_verbatim.md:6): “We monitor uptime at 99.9% and maintain daily automated database backups.”
Claim (doc9_seller_s_technical_claims_verbatim.md:10): “The database is backed up daily to Heroku's managed storage with 30-day retention.”
Claim (doc2_tech_stack_description_as_provided_by_se.md:5): “PostgreSQL, which is configured with daily scheduled backups”

Evidence: doc4_infrastructure_config.md:16–19 — “Daily at 2:00 AM via `heroku pg:backups schedule`”; “Failures recorded for the last 21 days.”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.” Lexical search of the data room for `retention` / `30-day` / `30 day` finds the 30-day figure only in claim sources (doc1:19, doc9:10) and in contract termination language (doc7:10,18,26), not in infrastructure config. Structural listing of doc4’s Backups section (lines 15–19) has schedule, status, last success, and alerting — no retention field.

Gap: Schedule exists; the last 21 days failed; last success is 2026-07-30; no backup-failure alerts; 30-day retention is not stated in the config document.

**Finding 2: Recoverable-backup window — [derived]**

Basis: doc4_infrastructure_config.md:18 — “Last Successful Backup: 2026-07-30”
       doc9_seller_s_technical_claims_verbatim.md:10 — “30-day retention.”
       doc1_seller_listing_description.md:19 — “daily automated database backups with 30-day retention.”

Derivation: 2026-07-30 plus 30 days is 2026-08-29.

Consequence: On the seller’s own figures, the last successful backup ages out of a 30-day retention window on 2026-08-29. After that date the materials do not show a later successful backup. As-of the materials and this report (2026-08-25), that date is four days away.

Escalates: Finding 1

**Finding 3: 120 active accounts — [delta]**

Claim (doc1_seller_listing_description.md:9): “loyal customer base of 120 active accounts”
Claim (doc1_seller_listing_description.md:13): “Active Customers: 120”

Evidence: doc6_crm_export_summary.md:3,6–18 — “Total Accounts Marked 'Active': 120”; 94 Stripe-linked; 3 enterprise wire; 23 “Phantom/Inactive Accounts” created Q1 2026, “No payment events in the last 90 days,” last login 60–120 days ago; “All 120 accounts are currently marked as 'active' in the CRM.” doc5_stripe_export_summary.md:4 — “Total Active Subscriptions: 94.” doc7_customer_contracts_summary.md:3,29,36,42 — 3 enterprise contracts, 94 month-to-month, 8 unpaid pilots, 15 no-contract Stripe Pro users.

Gap: The CRM status field is 120; paying/active composition is 94 Stripe + 3 enterprise. Twenty-three marked-active accounts have no recent payment or login.

**Finding 4: Blended MRR $40,000 / 12× asking price — [partial]**

Claim (doc1_seller_listing_description.md:5): “Asking Price: $480,000 (12x blended MRR)”
Claim (doc1_seller_listing_description.md:9,12): “blended Monthly Recurring Revenue (MRR) stands at $40,000” / “Blended MRR: $40,000”

Evidence: doc5_stripe_export_summary.md:5,17,19 — “Total MRR (Stripe): $16,000”; enterprise wires “$8,000/mo each” for Acme Retail, GlobalMart, ShopStream, “not included in the $16,000 MRR figure.” 16000 + 3×8000 = 40000. doc7:6,14,22 confirm $8,000/mo each. GlobalMart renews 2026-11-01 (doc7:15).

Gap: Arithmetic of Stripe + three wires matches $40,000. $24,000/mo (60%) is three annual wire contracts, one renewing 2026-11-01; Stripe is $16,000 on 94 subscriptions, not a “broad base” carrying most of the multiple. Asking price is 12× that blend.

**Finding 5: Platform redundancy and automatic failover — [delta]**

Claim (doc1:18): “platform-level redundancy and automatic failover.”
Claim (doc2:3): “hosted on Heroku, which provides automatic scaling and failover capabilities at the platform level.”
Claim (doc9:5): “redundancy and automatic failover through Heroku's platform-level process management.”

Evidence: doc4:5–8 — Dyno `standard-1x` (1GB RAM, 0.5 CPU); Database `heroku-postgresql:standard-0` “running on the same dyno as the application”; “Read Replicas: None.”; “Separate DB Instance: No.” Grep for `replica` / `failover` hits only claim sources, not config.

Gap: Single standard-1x dyno; Postgres on the same dyno; no replicas. Materials do not show redundancy or automatic failover of the application data plane.

**Finding 6: Horizontal scale / automatic scaling — [partial]**

Claim (doc9:8): “The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard.”
Claim (doc2:3): “automatic scaling and failover”

Evidence: doc4:5 — one `standard-1x` dyno. doc8:15 — Heroku cost “$25/mo (standard-1x dyno).” Heroku can add dynos in principle; nothing in config shows autoscaling or more than one dyno. Database on the same dyno (doc4:6) is not a horizontally scaled data tier.

Gap: One-click add-dyno is a platform property; current deployment is a single dyno with collocated Postgres. “Automatic scaling” is not evidenced.

**Finding 7: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): 99.9% uptime monitoring.

Evidence: doc4:22–23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).”; “Status Page: Heroku built-in status page only.” doc8:16 — Heroku “99.95% platform uptime (not contractual for the app).” Enterprise contracts require 99.9% (doc7:8,16) or 99.5% (doc7:24) with credits.

Gap: No application uptime monitor. Platform status is not 99.9% app monitoring. SLA credits are contracted without a measured series in the data room.

**Finding 8: Comprehensive test coverage / well-documented — [delta]**

Claim (doc1:21; doc9:7): well-documented codebase with comprehensive test coverage across critical / all critical paths.

Evidence: doc3:25–32 — “CI/CD Pipeline: None configured.”; Unit Tests: 12, all in `test/utils/`; Integration Tests: 0; Payment-Path Tests: 0; Staging: None; Branch Protection: None; Code Review: “None documented.” Last commit 2026-07-30; 0 commits in last 30 days (doc3:21–22).

Gap: Twelve utility unit tests, no payment or integration tests, no CI. Documentation of the codebase is not evidenced; “none documented” applies to review process.

**Finding 9: No proprietary lock-in / standard SaaS agreements — [delta]**

Claim (doc1:22; doc9:9): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in” / “no lock-in.”

Evidence: doc8:7–12 — DataEnrich.io: product matching, price comparison, category classification; “$400/mo flat”; “Termination Notice: 90 days (either party).”; “40% of features depend on this API.”; “Fallback: None implemented.” Those features are required on enterprise contracts (doc7:11,19,27). doc8:25–28 — Google OAuth: “No contractual relationship”; revoke means users cannot log in. doc8:29–32 — GoDaddy DNS “Managed personally by 'dave'. No secondary DNS.”

Gap: Enrichment API is a single-vendor path for ~40% of features with no fallback and 90-day termination; OAuth and personal DNS are additional single points of control.

**Finding 10: Managed DNS — [delta]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4:26–28 — “SSL: Heroku-managed, auto-renewed.”; “DNS Provider: GoDaddy”; “DNS Management: Managed personally by 'dave'. No secondary DNS provider.” doc8:29–32 same.

Gap: SSL is Heroku-managed. DNS is personal GoDaddy, not managed DNS as part of the stack.

**Finding 11: Rails 7 — [unverifiable] (not a finding; Remaining Claims)**

Claim (doc1:18; doc2:3; doc9:11): Rails 7.

Evidence: Grep for Rails / version hits only claim sources. doc3 and doc4 do not state a framework version. Absence of a version pin is not proof it is not Rails 7.

**Finding 12: Stripe payments / OAuth / SSL — [real, operational caveat]**

Claim (doc1:20; doc2:5): Managed SSL, OAuth, Stripe for cards and recurring billing.

Evidence: doc4:26 SSL Heroku-managed; doc4:29 secrets include Stripe keys and Google OAuth; doc5 Stripe subscriptions exist; doc8:1–6 Stripe; doc8:25–27 Google OAuth. Caveat: 14 secrets in Heroku config vars, no vault (doc4:29); OAuth has no contract (doc8:28); $24k/mo enterprise is wire, not Stripe (doc5:19).

**Finding 13: Redis caching — [real, minor caveat]**

Claim (doc2:5): Redis for session caching.

Evidence: doc4:11–13 Heroku Redis `heroku-redis:bb-1` $50/mo; doc8:17–20 caching; if down, “app is slow but functional.”

**Finding 14: Revenue model recurring monthly & annual enterprise — [real, operational caveat]**

Claim (doc1:15).

Evidence: doc5/doc7: 94 monthly Stripe + 3 annual $8k wires. Caveat: 8 free pilots expire Q3 2026; 15 Pro users have no signed ToS (doc7:36–45).

**Finding 15: Low churn — [unverifiable]**

Claim (doc1:9,14). No churn series, cohort, or cancel counts in the data room. 23 phantom accounts and 8 unpaid pilots cut against a “stable base” story but do not measure churn.

**Finding 16: Heroku hosting / Postgres / Stripe as stack components — [real]**

Claim (doc2:3–5). Evidence doc4:4–6, doc5, doc8:1–16.

Coverage: 31 claims frozen (doc1: 15, doc2: 9, doc9: 7). 22 examined. Stopped remaining low-narrative claims because higher tiers already produce Material findings; consistency of checked claims is not high enough to treat the rest as low-risk.

Ask the seller before closing: produce a successful restore from 2026-07-30 (or a newer backup); explain 23 CRM-active non-payers; confirm whether $40k MRR is used for the 12× multiple and GlobalMart’s 2026-11-01 renewal; DataEnrich contract and fallback; who controls the GoDaddy account post-close; Rails version lockfile; any uptime measurements backing 99.9% and enterprise SLAs.

=== LIMITATIONS ===
Materials examined: nine data-room documents (doc1–doc9) as bound to inspect_external, as-of 2026-08-25 (git range through 2026-08; last commit and last successful backup both 2026-07-30).
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 22 of 31 frozen claims examined; the recommendation applies only to those examined.

=== GAP MAP ===

FlowMetrics — small B2B e-commerce analytics SaaS offered by Dave at $480,000.

**Material** — 22 of 31 claims examined.

- Backups: daily job has failed 21 days; last success 2026-07-30; claimed 30-day retention ages that copy out on 2026-08-29; no backup alerts.
- Customers/MRR: 120 “active” includes 23 unpaid phantoms; Stripe MRR is $16k; $24k is three wires (one renews 2026-11-01).
- Architecture: one standard-1x dyno, Postgres on the same dyno, no replicas — not the claimed redundancy/failover/horizontal scale.
- Lock-in: DataEnrich.io is 40% of features, 90-day termination, no fallback; DNS is Dave’s personal GoDaddy; Google OAuth has no contract.
- Quality/ops: 12 utility unit tests, 0 payment tests, no CI, no uptime monitor against 99.9% / enterprise SLAs.

Coverage: 22/31 claims checked individually; remainder is low-priority narrative. Checked claims did not hold consistently, so the gap is not treated as low-risk.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice