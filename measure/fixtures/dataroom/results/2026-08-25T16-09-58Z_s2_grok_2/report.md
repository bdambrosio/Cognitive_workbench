**FlowMetrics — claims-verification report**

**Recommendation: Material** (coverage: 28 of 44 claims individually verified; remainder micro-claims and marketing adjectives whose failure would not change price, structure, or close). Materials as of the data-room snapshot whose last commit and last successful backup are both 2026-07-30; calendar date of this report is 2026-08-25.

---

**Finding 1: Recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc4_infrastructure_config.md:18 — "Last Successful Backup: 2026-07-30"
       doc9_seller_s_technical_claims_verbatim.md:10 / doc1:19 — daily backups "with 30-day retention"

Derivation: 2026-07-30 + 30 days = 2026-08-29. As of 2026-08-25 the last good copy is 26 days old. On 2026-08-29 the seller's own retention window, applied to the seller's own last-success date, leaves no recoverable backup.

Consequence: After 2026-08-29 the business cannot be restored from a backup the seller documented. Enterprise contracts (doc7) require product matching / real-time features; a lost database is a going-concern event at a $480k ask.

Escalates: Finding 2

**Finding 2: Daily automated backups with recoverability — [partial]**

Claim (doc1:19; doc2:5; doc9:6,10): daily automated database backups to Heroku managed storage for recoverability.

Evidence: doc4:16–19 — schedule exists (`heroku pg:backups schedule` daily 02:00); "Failures recorded for the last 21 days"; last success 2026-07-30; "Alerting: None".

Gap: The job is configured; it has not succeeded for 21 documented days and nobody is alerted. Recoverability as claimed does not hold today.

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc1:19; doc9:6): "We monitor uptime at 99.9%".

Evidence: doc4:21–22 — "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks)."

Gap: No monitor exists. Two enterprise contracts (doc7:8,16) still write a 99.9% SLA with 10% credit per 0.1% miss and 30-day termination for material breach. The claim is false; the contracts assume a measurement the stack does not take.

**Finding 4: 120 active accounts — [partial]**

Claim (doc1:9,13): "120 active accounts" / "Active Customers: 120".

Evidence: doc6:3,6–12,18 — CRM marks 120 active = 94 Stripe + 3 enterprise + 23 "Phantom/Inactive Accounts", all still status `active`.

Gap: Headcount is the CRM flag, not paying or using customers. Paying base supported by Stripe + contracts is 97, not 120.

**Finding 5: Blended MRR $40,000 and 12× ask — [real]**

Claim (doc1:5,12): Asking $480,000 (12× blended MRR); blended MRR $40,000.

Evidence: doc5:4–5,17,19 — Stripe MRR $16,000 (94 subs); three enterprise wires $8,000/mo each = $24,000. 16,000 + 24,000 = 40,000. 12 × 40,000 = 480,000.

Gap: None on the arithmetic. The blend is Stripe plus uncollected-in-Stripe wire contracts, not Stripe alone.

**Finding 6: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1:18; doc2:3,5; doc9:5): redundancy and automatic failover via Heroku process management; available under varying load without manual intervention.

Evidence: doc4:5–7 — single `standard-1x` dyno; Postgres `standard-0` "running on the same dyno as the application"; "Read Replicas: None." doc8:15–16 — Heroku 99.95% is platform SLA, "not contractual for the app".

Gap: One dyno, colocated primary, no replica. Process restart is not redundancy. A dyno or disk loss takes app and database together.

**Finding 7: No proprietary lock-in / standard SaaS agreements — [delta]**

Claim (doc1:22; doc9:9): all third-party integrations on standard SaaS agreements, no proprietary lock-in.

Evidence: doc8:7–12 — DataEnrich.io $400/mo, 90-day termination either party, "40% of features depend on this API", "Fallback: None". Required enterprise features in doc7:11,19,27 are product matching, price comparison, category classification — the DataEnrich functions in doc8:8. Google OAuth (doc8:25–28) has "No contractual relationship"; revoke means no login.

Gap: A 90-day kill switch on 40% of features with no fallback is lock-in. Stripe is standard; the lock-in claim is not true of the stack as a whole.

**Finding 8: Comprehensive test coverage on critical paths — [delta]**

Claim (doc1:21; doc9:7): well-documented codebase with comprehensive test coverage across critical / all critical paths.

Evidence: doc3:25–29 — no CI; 12 unit tests, all in `test/utils/`; 0 integration tests; 0 payment-path tests.

Gap: Critical paths (auth is 47% of dave's commits, payments via Stripe) have no tests in the materials. "Comprehensive" is false.

**Finding 9: Single-person DNS / managed DNS — [partial]**

Claim (doc9:11): "automatic SSL and managed DNS".

Evidence: doc4:27–28; doc8:29–32 — GoDaddy, "Managed personally by 'dave'. No secondary DNS." Expiry or registrar issue makes the app unreachable.

Gap: DNS is managed, by one person, with no secondary. Not what "managed DNS" implies in a turnkey sale.

**Finding 10: Horizontal one-click scale — [real, operational caveat]**

Claim (doc9:8; doc2:3): adding dynos is one-click; Heroku automatic scaling.

Evidence: Heroku can add dynos (doc8:13–15). The database is on the same dyno (doc4:6) with no replicas (doc4:7).

Gap: Web dynos can multiply; the data tier cannot. Scaling the claim names does not scale the store.

**Finding 11: Well-documented codebase — [unverifiable]**

Claim (doc1:21; doc9:7): well-documented.

Evidence: no README, comment-ratio, or doc tree appears in any of the nine documents (search of doc7 and doc9; git summary has no documentation directory).

Gap: Not a [delta] — absence of a documentation corpus in a data room is not proof the repo has none. Remaining Claims.

**Finding 12: Low churn / turnkey / PMF / customers engaged — [unverifiable]**

Claim (doc1:9,14,25).

Evidence: no cohort, logo, or engagement export. 23 CRM phantoms (doc6:12) cut against "engaged" but do not measure churn.

Gap: Cannot settle from these materials.

**Finding 13: Rails 7, Stripe CC + recurring, Redis caching, OAuth, enrichment API for product intelligence — [real] (bundle of micro-claims 17, 24–25, 35–37)**

Claim: doc1:18,20; doc2:3,5.

Evidence: stack and dependency list name Rails 7 / Heroku, Stripe, Redis add-on, Google OAuth, DataEnrich for matching/pricing/classification. No contradictory line.

Gap: None on existence. Quality and failover of those pieces are Findings 6–8.

**Finding 14: Revenue-positive, recurring SaaS, mix enterprise + MTM — [real, minor caveat]**

Claim (doc1:9,15).

Evidence: $16k Stripe + $24k contracted wire is recurring. Costs visible: Heroku $25, Redis $50, Twilio $100, DataEnrich $400, Stripe fees — far below $40k. "Revenue-positive" is not independently P&L-audited.

Gap: No full expense book; the named COGS do not overturn the claim.

---

**Coverage.** Denominator frozen at 44. Individually verified above: claims 1, 2, 3, 5, 6, 8, 9, 11, 17–22, 24–25, 27–29, 32–37, 40–41, 43–44 and the lock-in/test/docs cluster. Not individually walked: real-time analytics as a latency claim, indie-to-mid-market range, minimal overhead, turnkey, PMF, immediate cash flow, robust/defensible, customers engaged, dashboard "streamlined", backups "to Heroku managed storage" as a storage-location claim distinct from Finding 2, automatic SSL as a cert-issuance claim. Those leftovers are adjectives or ops micro-claims; the consistency rate on load-bearing items is poor (multiple [delta] in tiers 1–2), so the unverified remainder is not low-risk on architecture — it is low-risk only because it cannot move the recommendation further than Material.

**Ask the seller before close.** (1) A restore test from the 2026-07-30 dump, and whether retention can be extended before 2026-08-29. (2) Bank evidence of the three $8k wires, assignment of those contracts, and GlobalMart's 2026-11-01 renewal. (3) Written assignment of the Heroku, Stripe, DataEnrich, GoDaddy, and Google OAuth accounts; DataEnrich remaining term. (4) Why 23 CRM rows are Active. (5) Any uptime log that could support the SLA credits already accruing in theory.

=== LIMITATIONS ===
Nine markdown data-room documents; last dated technical facts 2026-07-30 (commit and last backup); git range labelled through 2026-08. No application source tree.
The seller was not consulted and has not confirmed the auditor's reading of their own claims.
Limited assurance over a disclosed subset (28/44) of the claim surface; the conclusion is positive or adverse only over claims examined, not the whole business.
