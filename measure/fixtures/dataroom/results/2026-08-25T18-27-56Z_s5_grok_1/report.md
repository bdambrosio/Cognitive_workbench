**Recommendation: Material** (of 33 claims identified, 22 checked individually). Material findings change valuation and recoverability risk. Coverage: remaining 11 are low-impact product-wording claims (pain-point copy, “turnkey,” “engaged customers”) that do not independently move price once MRR, recoverability, tests, and lock-in are priced.

As-of date of materials: snapshot in the data room as examined 2026-08-25.

**Finding 1: Daily backups with 30-day retention — [partial]**

Claim (doc9_seller_s_technical_claims_verbatim.md:10; also doc1:19, doc2:5): “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:16 — “Schedule: Daily at 2:00 AM via `heroku pg:backups schedule`”; doc4:17 — “Failures recorded for the last 21 days.”; doc4:18 — “Last Successful Backup: 2026-07-30”; doc4:19 — “Alerting: None configured for backup failures.”

Gap: Schedule exists; it has failed for 21 days with no alerts. Retention is asserted, not independently evidenced as a remaining window of good copies.

**Finding 2: Recoverability window from last-good backup — [derived]**

Basis: doc4_infrastructure_config.md:18 — “Last Successful Backup: 2026-07-30”
       doc9_seller_s_technical_claims_verbatim.md:10 — “30-day retention.”

Derivation: 2026-07-30 plus 30 days is 2026-08-29.

Consequence: If retention is as claimed, the last recoverable copy ages out on 2026-08-29. As of 2026-08-25 that is four days away. After that date the materials show no later successful backup.

Escalates: Finding 1

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): “99.9% uptime monitoring” / “We monitor uptime at 99.9%”

Evidence: doc4:22 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks).” Structural: infrastructure config is the place a monitor would be listed. Lexical: no Pingdom/UptimeRobot/custom checks in the corpus except this negation.

Gap: Monitoring as claimed is not present. Enterprise contracts (doc7:8,16) still state 99.9% SLA (ShopStream 99.5% at doc7:24).

**Finding 4: Platform redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3; doc9:5): platform-level redundancy and automatic failover / Heroku process-level redundancy.

Evidence: doc4:5 — Dyno `standard-1x` (1GB RAM, 0.5 CPU); doc4:6 — Database `heroku-postgresql:standard-0` “(running on the same dyno as the application)”; doc4:7 — “Read Replicas: None.”; doc4:8 — “Separate DB Instance: No.”

Gap: Single dyno, database co-located, no replicas. That is not the redundancy/failover asserted.

**Finding 5: Horizontal scale, one-click dynos / automatic scaling — [partial]**

Claim (doc9:8; doc2:3): “adding dynos is a one-click operation”; Heroku “automatic scaling and failover”.

Evidence: doc4:5 single `standard-1x`; doc8:15 Heroku cost “$25/mo (standard-1x dyno)”. Dashboard one-click is a Heroku property; current config is one unscaled dyno. Automatic scaling is not shown as enabled.

Gap: Capability of the platform vs. configured operation.

**Finding 6: 120 active accounts — [partial]**

Claim (doc1:9, doc1:13): “120 active accounts” / “Active Customers: 120”

Evidence: doc6:3 — “Total Accounts Marked 'Active': 120”; doc6:6 — 94 Stripe-linked; doc6:7–11 — 3 enterprise wire; doc6:12–16 — 23 “Phantom/Inactive Accounts” created Q1 2026, no payment events in last 90 days, last login 60–120 days ago; doc6:18 all 120 marked active.

Gap: 120 is a CRM flag. Paying set in evidence is 94 + 3 = 97. 23 have no recent payment or login.

**Finding 7: Blended MRR $40,000 and 12× asking price — [real, operational caveat]**

Claim (doc1:5, doc1:9, doc1:12): Asking $480,000 (12× blended MRR); blended MRR $40,000.

Evidence: doc5:5 Stripe MRR $16,000; doc5:19 three enterprises $8,000/mo each via wire, not in Stripe; 3 × $8,000 = $24,000; $16,000 + $24,000 = $40,000. 12 × $40,000 = $480,000.

Gap: None on the arithmetic. Caveat: $24,000/mo (60% of blended MRR) is three annual wire contracts, not Stripe (doc5:14,19; doc7:5–27). GlobalMart renews 2026-11-01 (doc7:15).

**Finding 8: Comprehensive tests on critical paths — [delta]**

Claim (doc1:21; doc9:7): comprehensive test coverage across critical / all critical paths.

Evidence: doc3:25 CI/CD none; doc3:27 Unit Tests: 12, all in `test/utils/`; doc3:28 Integration Tests: 0; doc3:29 Payment-Path Tests: 0; doc3:30 Staging none; doc3:31 Branch protection none.

Gap: No payment-path or integration tests. Twelve utility unit tests are not comprehensive coverage of critical paths.

**Finding 9: Low vendor lock-in / standard SaaS agreements, no proprietary lock-in — [delta]**

Claim (doc1:22; doc9:9).

Evidence: doc8:7–12 DataEnrich.io: $400/mo, 90-day termination either party, “40% of features depend on this API”, “Fallback: None implemented.” Enterprise required features (doc7:11,19,27) include product matching, price comparison, category classification, real-time inventory sync — the functions listed for that API (doc8:8).

Gap: A 90-day-notice, no-fallback dependency for ~40% of features and for contracted enterprise features is proprietary operational lock-in relative to the claim.

**Finding 10: Managed DNS — [delta]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4:26 SSL Heroku-managed auto-renewed (SSL half holds). doc4:27–28 DNS GoDaddy, “Managed personally by 'dave'. No secondary DNS provider.” doc8:29–32 same.

Gap: DNS is personal GoDaddy, not managed DNS as claimed. Single-person control of reachability.

**Finding 11: Low churn — [unverifiable] as a finding; noted in Remaining Claims**

Claim (doc1:9, doc1:14). Evidence shows 23 CRM-active accounts with no payment in 90 days (doc6:12–15) but no churn rate series. Not reported as [delta].

**Finding 12: Stripe payments / Redis / Rails 7 hosting / SSL — [real] or [real, minor caveat]**

Stripe recurring billing: doc2:5 vs doc5 (holds). Redis session cache: doc2:5 vs doc4:11–13 heroku-redis:bb-1 (holds). Heroku-managed SSL: doc9:11 vs doc4:26 (holds). Rails 7: asserted doc1:18, doc2:3, doc9:11; git summary does not state framework version — version itself [unverifiable] from these materials; hosting on Heroku [real] (doc4:4).

**Finding 13: Single points of control — [derived]**

Basis: doc3:8 dave 618/847 commits (73%); doc4:28 DNS managed personally by dave; doc3:21 last commit 2026-07-30; doc3:22 0 commits last 30 days.

Derivation: Application change, DNS, and secrets (doc4:29, 14 config vars, no vault) concentrate on one person; repo idle since the same date as last-good backup.

Consequence: Key-person and recoverability risk coincide on 2026-07-30.

Escalates: Findings 2 and 10.

Checked that held without material gap: Stripe as processor; Redis present; Heroku as host; SSL auto-renew; blended $40k arithmetic if wires are counted; asking multiple arithmetic; 94 Stripe subscriptions matching CRM Stripe-linked count; three named enterprise contracts at $8,000/mo.

**Coverage.** 22 of 33 claims checked. Unchecked 11 are marketing/PMF wording (D1-2 real-time analytics as product description, D1-6 turnkey/PMF, D1-16 defensible pain-point copy, D1-17 “technology proven / customers engaged,” D2-1 ease of maintenance as design intent, D2-8 dashboard management as a true Heroku property, plus overlapping restatements already scored). Stopped because remaining items are §4 tier 4 and checked claims did not hold consistently — the stop is not “low-risk remainder,” it is budget on the material set. That matters: consistency rate on checked claims is not a clean pass (multiple [delta] on recoverability, tests, failover, lock-in, DNS, customer count).

**Ask the seller before closing.** (1) Produce a successful restore from a backup newer than 2026-07-30, or concede the 2026-08-29 age-out. (2) Reconcile 120 CRM-active vs 97 paying. (3) DataEnrich contract, assignment on sale, and written fallback. (4) DNS/registrar credentials and secondary DNS. (5) Whether GlobalMart 2026-11-01 renewal is committed. (6) Any uptime evidence behind the 99.9% SLA credits. (7) Payment-path tests if they exist outside `test/utils/`.

=== LIMITATIONS ===
Materials: nine markdown documents in the bound data room (doc1–doc9), as-of 2026-08-25. No running system, no restore drill, no seller interview.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 22 of 33 enumerated claims checked; recommendation rests on those 22.
