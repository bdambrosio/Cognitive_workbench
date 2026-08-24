**Recommendation: Material**

The seller’s reliability claims (daily backups with 30-day retention, 99.9% uptime monitoring, comprehensive tests, platform redundancy/failover, no lock-in, managed DNS) do not match the data-room evidence. The commercial headlines ($40k blended MRR, 120 active accounts, 12× ask) hold only if CRM “active” includes 23 non-paying accounts and if three $8k/mo wires land outside Stripe. A buyer must price those gaps or walk.

---

**Finding 1: Daily automated backups / 30-day retention — [delta]**

Claim (doc9:6, doc9:10; doc1:19; doc2:5): daily automated database backups to Heroku managed storage with 30-day retention.

Evidence (doc4:16–19): schedule exists (`heroku pg:backups schedule`, daily 02:00); “Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30”; “Alerting: None configured for backup failures.”

Delta: scheduled ≠ succeeding. Retention as a live restore capability is not what is on disk today.

**Finding 1a: Restore window from seller figures — [derived]**

Basis: doc4:18 — “Last Successful Backup: 2026-07-30”
       doc9:10 / doc1:19 — “30-day retention”

Derivation: 2026-07-30 + 30 days = 2026-08-29. Session date is 2026-08-24.

Consequence: if the seller’s own retention term is applied to the last-good backup they recorded, the restore window has about five days left, after 21 consecutive failed backups and no failure alerting.

Escalates: Finding 1.

**Finding 2: 99.9% uptime monitoring — [delta]**

Claim (doc9:6; doc1:19): “We monitor uptime at 99.9%” / “99.9% uptime monitoring.”

Evidence (doc4:21–23): “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)”; “Status Page: Heroku built-in status page only.”

Delta: the claimed monitoring is not present. Separate commercial exposure: enterprise contracts (doc7:8, :16, :24) still promise 99.9% / 99.5% SLAs with credits.

**Finding 3: Comprehensive test coverage across critical paths — [delta]**

Claim (doc9:7; doc1:21): well-documented codebase with comprehensive test coverage across all / critical paths.

Evidence (doc3:25–32): no CI; 12 unit tests, all in `test/utils/`; 0 integration tests; 0 payment-path tests; no staging; no branch protection; no documented code review. Last commit 2026-07-30; 0 commits in last 30 days (doc3:21–22).

Delta: “comprehensive … critical paths” is false on the seller’s own git summary. Documentation quality is [unverifiable] from these materials (no source tree).

**Finding 4: 120 active accounts — [partial]**

Claim (doc1:9, doc1:13): 120 active accounts / active customers.

Evidence (doc6:3–18): CRM marks 120 active = 94 Stripe-linked + 3 enterprise wire + 23 “Phantom/Inactive” (created Q1 2026, no payment in 90 days, last login 60–120 days, no notes).

Delta: the count is the CRM flag, not paying or engaged customers. Paying set is 97 if the three wires are real (94 + 3).

**Finding 5: $40,000 blended MRR / $480k = 12× — [partial]**

Claim (doc1:5, :9, :12): blended MRR $40,000; ask $480,000 = 12× blended MRR.

Evidence (doc5:5, :17, :19; doc7:6, :14, :22): Stripe MRR stated $16,000; three wires $8,000/mo each = $24,000 if collected. 16,000 + 24,000 = 40,000 on paper. No bank/wire export in the room. Stripe line items: 91 × $149 + 3 × $817 = **$16,010**; seller states Pro total $13,549 and Stripe MRR $16,000 (doc5:11, :17) — a $10 / $61 internal mismatch on their own figures.

Delta: the multiple is tautological if the blend is accepted; the $24k wire leg is not independently evidenced; Stripe arithmetic is not internally consistent at the dollar.

**Finding 6: Redundancy and automatic failover — [delta]**

Claim (doc9:5; doc1:18; doc2:3, :6): redundancy and automatic failover through Heroku platform-level process management; remains available under varying load with no manual intervention.

Evidence (doc4:5–8): one `standard-1x` dyno (1GB / 0.5 CPU); Postgres `heroku-postgresql:standard-0` “running on the same dyno as the application”; “Read Replicas: None”; “Separate DB Instance: No.”

Delta: a single dyno with co-located Postgres and no replicas is not redundancy or automatic failover of the application or its data. Heroku process restart is not what the listing describes.

**Finding 7: Horizontal scale / one-click dynos — [partial]**

Claim (doc9:8; doc2:3): designed to scale horizontally; adding dynos is one-click; automatic scaling.

Evidence (doc4:5–8): one standard-1x; DB on the same dyno, no replicas. Adding web dynos is a Heroku dashboard action; it does not scale the co-located Postgres or remove the single-instance data plane.

Delta: the one-click web-dyno claim is narrowly true as a platform feature; “designed to scale horizontally” / “automatic scaling” overstates the deployed topology.

**Finding 8: No lock-in / standard SaaS agreements — [partial]**

Claim (doc9:9; doc1:22): all third-party integrations on standard SaaS agreements with no lock-in / no proprietary lock-in.

Evidence (doc8:7–12): DataEnrich.io — $400/mo; 90-day termination either party; “40% of features depend on this API”; “Fallback: None implemented.” Google OAuth (doc8:25–28): “No contractual relationship”; revoke ⇒ users cannot log in. Enterprise required features (doc7:11, :19, :27) include product matching / price comparison / category classification — the DataEnrich functions.

Delta: Stripe/Twilio/Heroku look standard. DataEnrich is operational lock-in (40% of features, no fallback, 90-day notice) even if the paper is a “standard” SaaS form.

**Finding 9: Managed DNS — [delta]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence (doc4:26–28; doc8:29–32): SSL is Heroku-managed, auto-renewed — [real] for the SSL half. DNS: GoDaddy, “Managed personally by 'dave'. No secondary DNS provider.” If domain expires or GoDaddy has an issue, the app is unreachable.

Delta: “managed DNS” as a platform/ops claim is false; it is a personal GoDaddy account.

**Finding 10: Rails 7 + automatic SSL — [real, minor caveat]**

Claim (doc9:11; doc1:18, :20; doc2:3): Rails 7; automatic / managed SSL.

Evidence: stack statements in doc2:3 and doc9:11; SSL at doc4:26. Rails version is asserted, not independently version-pinned in a Gemfile in this room — treated as [real] on seller consistency across three docs, with that caveat.

**Finding 11: Low churn / engaged customers / turnkey — [partial]**

Claim (doc1:9, :14, :25): loyal / engaged base; “low churn”; turnkey; immediate cash flow.

Evidence: 23 phantom accounts (doc6:12–16); 8 free pilots expiring Q3 2026 with no payment obligation (doc7:36–40); 15 Pro accounts with no signed ToS on file (doc7:42–45); MTM 94 can cancel at end of cycle with no SLA (doc7:29–34); last commit 2026-07-30 and 0 in 30 days (doc3:21–22); single-operator concentration (dave 73% of 847 commits, doc3:8).

Delta: “low churn” is not quantified. Engagement and turnkey are overstated relative to phantoms, pilots, key-person git, and personal DNS/secrets.

**Finding 12: Secrets posture — [non-delta]**

No listing claim about a vault. Observed (doc4:29): 14 secrets in Heroku config vars; no vault. Noted for completeness, not a claim test.

---

**Coverage.** Identified claim surface ~35–40 micro-claims across nine documents. Individually verified: the load-bearing technical set (doc9 all seven bullets + listing financials + lock-in + churn language) and the commercial headlines. Not checked: live Heroku/Stripe/bank access; Gemfile/lock for Rails 7; actual DataEnrich contract PDF; whether wires cleared; production logs vs SLA; source-level “well-documented.” Remainder is low-risk only if the buyer treats unverified commercial cash as $16k Stripe until wires are shown — not if they underwrite $40k.

**Ask the seller before close**

1. Why have `pg:backups` failed for 21 days, and produce a successful restore from 2026-07-30 (or a newer backup) before 2026-08-29.
2. Bank evidence that Acme / GlobalMart / ShopStream $8,000 wires cleared for the last 12 months.
3. Why 23 CRM “active” accounts have no payment and stale logins; written policy for the active flag.
4. Reconcile Stripe $16,000 / $13,549 vs 91×149+3×817 = $16,010.
5. DataEnrich contract, assignment on sale, and a fallback plan for the 40% of features and the enterprise required-feature lists.
6. Transfer of the GoDaddy domain and Heroku app; secondary DNS; inventory of the 14 config-var secrets.
7. Whether enterprise SLAs (99.9% / 99.5% with 10% credits per 0.1%) have ever been measured or credited, given no uptime monitor.

---

=== GAP MAP ===

**FlowMetrics** — B2B e-commerce analytics SaaS (seller: Dave; ask $480k).

**MATERIAL**

- **Backups failing 21 days; last-good 2026-07-30.** Stated 30-day retention exhausts 2026-08-29. No backup-failure alerts.
- **No 99.9% uptime monitoring.** Enterprise contracts still promise 99.9%/99.5% with credits.
- **“Comprehensive tests” = 12 util unit tests, 0 payment/integration, no CI.**
- **120 “active” includes 23 unpaid phantoms; $40k MRR needs $24k unproven wires.** Stripe-only is $16k; ask is 12× the blend.
- **Single dyno, DB on the same box, no replicas; DNS is Dave’s GoDaddy; 40% of features on DataEnrich with no fallback.**

**Coverage:** load-bearing technical claims (doc9 + listing) and headline financials verified against the nine-document room. No live systems or bank files. Remainder is not low-risk if the buyer underwrites $40k MRR unconfirmed.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice