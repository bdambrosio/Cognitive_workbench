**Recommendation: Material** — of 22 claims examined of 34 identified, multiple `[delta]` and `[partial]` findings change valuation and recoverability. Asking price $480,000 is 12× stated $40,000 blended MRR (doc1:5,12).

**Finding 1: Active customer count — [delta]**

Claim (doc1_seller_listing_description.md:9,13): “loyal customer base of 120 active accounts”; “Active Customers: 120”.

Evidence: doc6_crm_export_summary.md:3,6–16 — 120 marked active; 94 Stripe-linked; 3 enterprise wire; 23 “Phantom/Inactive” created Q1 2026, no payment in 90 days, last login 60–120 days ago.

Gap: Paying/operating accounts are 97 (94+3), not 120. The 23 phantoms are still marked active.

**Finding 2: Daily backups and recoverability — [delta]**

Claim (doc1:19; doc9:6,10): “daily automated database backups with 30-day retention”; “The database is backed up daily to Heroku's managed storage with 30-day retention.”

Evidence: doc4_infrastructure_config.md:16–19 — daily schedule 2:00 AM; “Failures recorded for the last 21 days”; “Last Successful Backup: 2026-07-30”; “Alerting: None”. Doc4 does not state 30-day retention.

Gap: Schedule exists; successful daily backups and 30-day retention as claimed do not. No failure alerting.

**Finding 3: Last recoverable backup ages out — [derived]**

Basis: doc9:10 — “30-day retention.”
       doc4:18 — “Last Successful Backup: 2026-07-30”

Derivation: 30 days from 2026-07-30 is 2026-08-29.

Consequence: If the seller’s own retention figure applies to the last successful backup, that copy ages out on 2026-08-29. Materials as-of 2026-08-25; four days later the last stated-good backup may be gone. Failures continue through the 21 days preceding the as-of date (doc4:17).

Escalates: Finding 2.

**Finding 4: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): “99.9% uptime monitoring”; “We monitor uptime at 99.9%”.

Evidence: doc4:21–23 — “Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)”; status page is Heroku built-in only. Doc7:8,16 — Acme and GlobalMart SLA 99.9%; ShopStream 99.5% (doc7:24).

Gap: No uptime monitor. SLA measurement path is not in the materials. Heroku platform SLA is 99.95% and “not contractual for the app” (doc8:16).

**Finding 5: Redundancy, failover, horizontal scale — [delta]**

Claim (doc1:18; doc2:3, last sentence; doc9:5,8): “platform-level redundancy and automatic failover”; “automatic scaling and failover”; “redundancy and automatic failover through Heroku's platform-level process management”; “scale horizontally — adding dynos is a one-click operation”.

Evidence: doc4:5–8 — single `standard-1x` (1GB, 0.5 CPU); Postgres `heroku-postgresql:standard-0` “running on the same dyno as the application”; “Read Replicas: None”; “Separate DB Instance: No.” Doc8:15 — Heroku cost $25/mo standard-1x.

Gap: Single dyno, database on that dyno, no replicas. Horizontal add-a-dyno is not evidenced as configured; co-located Postgres is not a redundant data plane.

**Finding 6: Comprehensive critical-path tests — [delta]**

Claim (doc1:21; doc9:7): “comprehensive test coverage across critical paths”; “across all critical paths.”

Evidence: doc3_git_history_summary.md CI/CD & Testing — “CI/CD Pipeline: None”; Unit Tests: 12, all in `test/utils/`; Integration Tests: 0; Payment-Path Tests: 0; no staging, branch protection, or code review.

Gap: Twelve utility unit tests; no integration or payment-path coverage; no CI.

**Finding 7: No proprietary lock-in — [partial]**

Claim (doc1:22; doc9:9): “All third-party integrations are on standard SaaS agreements with no proprietary lock-in” / “no lock-in.”

Evidence: doc8:7–12 — DataEnrich.io $400/mo; 90-day termination either party; “40% of features depend on this API”; “Fallback: None”. Doc7 required features for enterprise (product matching, price comparison, classification, inventory sync) map to that API (doc8:8). Google OAuth: “No contractual relationship”; revoke stops login (doc8:25–28). DNS personal to dave, no secondary (doc4:27–28; doc8:29–32).

Gap: Agreements may be standard; 40% of features and login/DNS have no fallback. Withdrawal of DataEnrich on 90 days degrades product to “basic mode”.

**Finding 8: Managed DNS — [delta]**

Claim (doc9:11): “modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4:26–28 — SSL Heroku-managed auto-renewed; “DNS Provider: GoDaddy”; “Managed personally by 'dave'. No secondary DNS provider.”

Gap: SSL holds. DNS is personal GoDaddy, not managed platform DNS.

**Finding 9: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1:9,12): blended MRR $40,000 from enterprise contracts and month-to-month subscribers.

Evidence: doc5:5,17,19 — Stripe MRR $16,000; three enterprises $8,000/mo each by wire, not in Stripe. 3 × $8,000 + $16,000 = $40,000.

Gap: None on the arithmetic. Operational: $24,000/mo is three annual wires, not Stripe-collected SaaS; GlobalMart renews 2026-11-01 (doc7:15).

**Finding 10: Asking multiple 12× blended MRR — [real]**

Claim (doc1:5): “$480,000 (12x blended MRR)”.

Evidence: 12 × $40,000 = $480,000. Holds as stated multiple of stated MRR, not as a quality of earnings opinion.

**Finding 11: Rails 7 on Heroku, Stripe, Redis, SSL — [real]**

Claim (doc2:3–5; doc1:20; doc9:11 in part).

Evidence: doc4:4–5,11–12,26; doc5 Stripe export; doc8:3,13,17,25. Holds for stack identity. Does not rehabilitate Findings 5 and 8.

**Finding 12: Revenue-positive recurring SaaS / mixed monthly and annual enterprise — [real, minor caveat]**

Claim (doc1:9,15).

Evidence: Stripe subscriptions + three annual enterprise contracts (doc5, doc7). Caveat: 8 free pilots expire Q3 2026; 15 Pro accounts have no signed ToS on file (doc7:36–45).

Coverage: 22 of 34 claims checked individually (all §4 tiers 1–3 that were stated, plus several tier-4 stack facts). Unchecked 12 are low-impact marketing (defensible product, PMF, “engaged” customers, “turnkey”, “minimal operational overhead”, qualitative “low churn” without a rate, “well-documented” beyond tests). Those remaining cannot settle valuation the way recoverability, headcount, SLA, and DataEnrich can. Consistency among checked claims is poor on architecture and ops; financial arithmetic on $40k holds.

What to ask the seller before closing: (1) restore proof after 2026-07-30 and whether any backup still exists after 2026-08-29; (2) written assignment of the three enterprise contracts and GlobalMart renewal status; (3) DataEnrich contract, assignment, and any fallback plan; (4) transfer of the GoDaddy account and Heroku app (14 config-var secrets); (5) explanation of 23 phantom CRM actives; (6) how 99.9% SLA is measured given no monitor.

=== LIMITATIONS ===
Materials: nine data-room documents in the bound corpus, as-of 2026-08-25. Seller was not consulted and has not confirmed this reading of their claims. Limited assurance (§1a): 22 of 34 enumerated claims examined; no opinion on the rest.
