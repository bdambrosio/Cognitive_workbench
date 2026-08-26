**Recommendation: Material** (of 22 claims examined individually out of 33 frozen; remainder low-priority or overlapping).

Of the 22 claims examined, several material gaps change valuation or risk: recoverability has failed for 21 days, last-good backup ages out on 2026-08-29, “120 active / $40k blended MRR” mixes 23 unpaid CRM phantoms and 8 unpaid pilots into the customer count, there is no uptime monitor against 99.9% SLAs, tests and CI are not comprehensive on critical paths, and DataEnrich.io is a 90-day-notice single point with no fallback covering required enterprise features. Stripe $16,000 plus three $8,000 wire contracts does equal the stated $40,000 blended MRR; asking $480,000 is 12× that figure. Rails 7, Heroku, Stripe, Redis, daily *schedule*, managed SSL, and OAuth are present as stated.

Materials as-of: 2026-08-25 snapshot of the nine-document data room. Last successful backup: 2026-07-30. Last commit: 2026-07-30.

**Finding 1: Daily backups with 30-day retention are not succeeding — [delta]**

Claim (doc1:19; also doc2:5, doc9:6, doc9:10): “99.9% uptime monitoring and daily automated database backups with 30-day retention”; “PostgreSQL … daily scheduled backups”; “daily automated database backups”; “backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4:16–19 — Schedule daily 2:00 AM via `heroku pg:backups schedule`; “Failures recorded for the last 21 days.”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.”

Gap: A schedule exists; successful daily backups do not. Twenty-one consecutive recorded failures, no alerting.

**Finding 2: Last recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc9:10 / doc1:19 — “30-day retention”
       doc4:18 — “Last Successful Backup: 2026-07-30”

Derivation: 2026-07-30 plus 30 calendar days is 2026-08-29. After that date the seller’s stated retention no longer covers any successful backup in these materials.

Consequence: From 2026-08-29 the data room snapshot implies the business cannot be restored from a successful backup the seller documented.

Escalates: Finding 1.

**Finding 3: “120 active customers” includes 23 unpaid phantom accounts — [partial]**

Claim (doc1:9, doc1:13): “loyal customer base of 120 active accounts”; “Active Customers: 120”

Evidence: doc6:3 — “Total Accounts Marked 'Active': 120”; doc6:6–16 — 94 Stripe-linked, 3 enterprise wire, 23 “Phantom/Inactive” created Q1 2026, no payment in 90 days, last login 60–120 days, no notes; doc6:18 all 120 marked active; doc7:36–40 eight free pilots expiring Q3 2026 with no payment obligation.

Gap: CRM status is 120; paying population in these materials is 94 Stripe + 3 enterprise = 97. Twenty-three marked-active accounts have no recent payment.

**Finding 4: Blended MRR $40,000 holds as arithmetic; 12× asking price holds — [real]**

Claim (doc1:5, doc1:9, doc1:12): Asking $480,000 (12× blended MRR); blended MRR $40,000.

Evidence: doc5:5,17 Stripe MRR $16,000; doc5:19 and doc7:6,14,22 three contracts $8,000/mo each. 3×8000+16000=40000. 12×40000=480000.

Gap: None on the dollar figure. Composition is Stripe plus off-Stripe wire, which the listing’s “mix” language allows (doc1:9).

**Finding 5: 99.9% uptime monitoring is not implemented — [delta]**

Claim (doc1:19; doc9:6): “99.9% uptime monitoring”; “We monitor uptime at 99.9%”

Evidence: doc4:22–23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).”; “Status Page: Heroku built-in status page only.” Enterprise SLAs at 99.9% / 99.5% (doc7:8,16,24) have a 10% monthly-fee credit per 0.1% miss (doc7:9,17,25).

Gap: No application uptime monitor. Platform status page is not the claimed 99.9% monitoring.

**Finding 6: Comprehensive tests on critical paths, including payments — [delta]**

Claim (doc1:21; doc9:7): “comprehensive test coverage across critical paths”; “across all critical paths.”

Evidence: doc3:25–31 — CI/CD none; unit tests 12 all in `test/utils/`; integration 0; payment-path tests 0; no staging; no branch protection; no documented code review.

Gap: Critical-path and payment tests are absent, not merely thin.

**Finding 7: No proprietary lock-in / standard SaaS agreements — [partial]**

Claim (doc1:22; doc9:9): “standard SaaS agreements with no proprietary lock-in”; “no lock-in.”

Evidence: doc8:7–12 DataEnrich.io $400/mo, 90-day termination either party, “40% of features depend on this API”, “Fallback: None”; required enterprise features include product matching / price comparison / category classification / real-time inventory (doc7:11,19,27) which are this API’s function (doc8:8). Google OAuth: no contractual relationship; revoke blocks login (doc8:25–28). DNS personal to dave, no secondary (doc4:27–28; doc8:29–32).

Gap: Legal form may be standard SaaS; operational lock-in is material (no fallback, 90-day notice, auth and DNS single-threaded to dave/Google/GoDaddy).

**Finding 8: Platform-level redundancy, automatic failover, and horizontal scale as deployed — [partial]**

Claim (doc1:18; doc2:3; doc9:5; doc9:8): Rails 7 with platform-level redundancy and automatic failover; available under varying load without manual routine ops; adding dynos one-click.

Evidence: doc4:4–8 single `standard-1x` dyno; `heroku-postgresql:standard-0` “running on the same dyno as the application”; “Read Replicas: None.”; “Separate DB Instance: No.” Heroku is the platform (doc4:4; doc8:13–16, SLA 99.95% “not contractual for the app”). One-click dyno add is a Heroku dashboard capability the materials do not contradict; current topology is one dyno, co-located DB, no replicas.

Gap: Platform *can* scale/fail over processes; this deployment is a single dyno with DB on the same dyno and no replicas. “Automatic failover” of the application data plane is not shown.

**Finding 9: Managed SSL holds; “managed DNS” does not match personal GoDaddy — [partial]**

Claim (doc1:20 managed SSL and OAuth; doc9:11 “automatic SSL and managed DNS”).

Evidence: doc4:26 “SSL: Heroku-managed, auto-renewed.”; doc4:27–28 DNS GoDaddy, “Managed personally by 'dave'. No secondary DNS provider.”; doc8:25–28 Google OAuth present; doc8:29–32 GoDaddy risk as above.

Gap: SSL and OAuth as claimed. DNS is not Heroku-managed; it is dave’s personal GoDaddy account.

**Finding 10: Daily scheduled backups (existence of schedule) — [real, operational caveat]**

Claim (doc2:5): PostgreSQL configured with daily scheduled backups.

Evidence: doc4:16 schedule exists. Caveat is Finding 1 (the schedule is failing).

**Finding 11: Redis session caching — [real]**

Claim (doc2:5): Redis for session caching.

Evidence: doc4:11–13 Heroku Redis add-on `heroku-redis:bb-1`; doc8:17–20 caching, app slow but functional if down.

**Finding 12: Stripe credit-card and recurring billing — [real]**

Claim (doc1:20; doc2:5): Stripe payments and recurring billing.

Evidence: doc5 entire file; doc8:3–6.

**Finding 13: Third-party enrichment API for real-time product data — [real, operational caveat]**

Claim (doc2:5): third-party enrichment API, real-time product data.

Evidence: doc8:7–12 DataEnrich.io. Caveat: Finding 7 (no fallback; 40% of features).

**Finding 14: Stack managed via Heroku dashboard — [real, minor caveat]**

Claim (doc2:5): entire stack managed through Heroku’s dashboard.

Evidence: Heroku app, Postgres add-on, Redis add-on, config vars (doc4). Caveat: DNS is GoDaddy, not Heroku (Finding 9).

**Finding 15: Rails 7 — [unverifiable] as to version pin**

Claim (doc1:18; doc2:3; doc9:11): modern Rails 7.

Evidence: asserted only in claim sources. Git summary (doc3) has no Gemfile/version pin. Not treated as [delta]; remains unverifiable from these materials.

**Finding 16: Low churn / engaged customers / turnkey / well-documented / revenue-positive / real-time analytics — mixed**

- Recurring Monthly & Annual Enterprise (doc1:15): [real] — doc5 + doc7 annual enterprise + monthly Stripe.
- Low churn (doc1:9,14): [unverifiable] — no cohort/churn rate in evidence; 23 unpaid “active” and 8 free pilots cut against “engaged.”
- Well-documented codebase (doc1:21; doc9:7): [unverifiable] — no README/docs inventory; test evidence is Finding 6.
- Revenue-positive (doc1:9): [unverifiable] — MRR stated; costs in doc8 (Stripe fees, $400+$25+$50+$100+$12/yr) are incomplete opex; no P&L.
- Real-time analytics / product intelligence (doc1:9): [partial] if taken as product capability — enrichment is real-time only while DataEnrich.io is up (doc8:12 degrades to basic mode).
- Minimal operational overhead / no manual routine ops (doc1:9; doc2:3): [delta] relative to backup failures with no alerting (doc4:17–19) and dave-personal DNS (doc4:28).
- Turnkey / proven PMF / immediate cash flow (doc1:9,25): marketing wrap; cash flow is the $40k composition (Finding 4) minus single-threaded ops. Not separately priced beyond Findings 1–8.

**Coverage.** Frozen surface: 33 claims (doc1 17, doc2 9, doc9 7). Examined individually: 22 (all tier-1 recoverability and payment/customer/dependency claims; architectural invariants; tests/uptime/SSL/DNS/Redis/Stripe/enrichment/Heroku management). Not separately scored: overlapping restatements of the same backup/uptime/Rails/lock-in sentences across doc1/doc2/doc9 (counted in the 33, tested once), plus qualitative “defensible product / pain point / strategic partner” (doc1:25) which is not a checkable implementation claim. Stopped there because remaining unchecked wording is low-priority marketing and the material tier has not held consistently — so the unchecked remainder is **not** assumed low-risk.

**Ask the seller before closing.** (1) Produce a successful restore from a backup dated after 2026-07-30, or a current backup artifact and retention proof. (2) Reconcile the 23 phantom CRM actives and 8 unpaid pilots against the 120 figure. (3) Confirm wire-transfer history for the three $8k contracts and assignment on sale. (4) DataEnrich.io contract, assignment, and any fallback plan; 90-day notice clock. (5) Who holds the GoDaddy login and Heroku/Google OAuth apps post-close. (6) Any uptime measurements behind the 99.9% claim. (7) Gemfile.lock Rails version. (8) Whether backup alerting was ever configured outside this snapshot.

=== LIMITATIONS ===
Nine markdown documents in the bound data room (`doc1`–`doc9`), examined as of 2026-08-25; last documented successful backup and last commit both 2026-07-30.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 22 of 33 frozen claims examined individually; this report states only those results and does not speak to unexamined wording except to say it was not used as a consistency warrant.
