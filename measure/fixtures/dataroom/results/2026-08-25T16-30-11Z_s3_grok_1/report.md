**Recommendation: Material**

Of the claims examined, recoverability, uptime monitoring, architectural redundancy, customer-count, test coverage, and third-party lock-in do not match the listing. Stripe-plus-wire MRR of $40,000 holds as arithmetic. Coverage: 22 of 66 claims individually verified (priority tiers 1–3 plus selected micro-claims). Remainder is mostly listing micro-claims (turnkey, low overhead, engaged customers) whose failure is less expensive than the gaps already priced.

A PE partner can explain this back as: the $40k blended figure is Stripe $16k plus three $8k wire contracts; 23 of 120 CRM “active” accounts have no recent payment; backups have failed for 21 days and the last good copy ages out of the stated 30-day retention on 2026-08-29; there is no uptime monitor; the database shares the single dyno with no replicas; DataEnrich.io is 40% of features with no fallback and 90-day termination; DNS sits with dave personally.

**Finding 1: Daily backups with 30-day retention — [partial]**

Claim (doc1 L19; doc9 L10; also doc2 L5, doc9 L6): daily automated database backups with 30-day retention / backed up daily to Heroku managed storage with 30-day retention.

Evidence: doc4 L16–19 — schedule is daily 02:00 via `heroku pg:backups schedule`; failures recorded for the last 21 days; last successful backup 2026-07-30; no alerting for backup failures. Retention period is not stated in doc4.

Gap: Schedule exists; the last 21 days did not produce a successful backup; retention is claimed, not observed in infra notes.

**Finding 2: Last recoverable backup ages out on 2026-08-29 — [derived]**

Basis: doc9 L10 — “The database is backed up daily to Heroku's managed storage with 30-day retention.”
       doc4 L18 — “Last Successful Backup: 2026-07-30”

Derivation: 2026-07-30 plus 30 days is 2026-08-29.

Consequence: Under the seller’s own retention term, the only cited successful backup is not guaranteed to exist after 2026-08-29. Combined with 21 days of failures and no backup-failure alerts (doc4 L17, L19), restore may already be stale and then become empty.

Escalates: Finding 1.

**Finding 3: 99.9% uptime monitoring — [delta]**

Claim (doc1 L19; doc9 L6): “We monitor uptime at 99.9%” / “99.9% uptime monitoring”.

Evidence: doc4 L22–23 — Uptime Monitor: None (no Pingdom, UptimeRobot, or custom checks). Status page: Heroku built-in only. doc8 L16 — Heroku SLA 99.95% is platform uptime, “not contractual for the app.”

Gap: No application uptime monitor is configured. Enterprise contracts (doc7) still cite 99.9% / 99.5% SLAs with service-credit remedies.

**Finding 4: Platform-level redundancy and automatic failover — [delta]**

Claim (doc1 L18; doc2 L3, L9; doc9 L5): redundancy and automatic failover through Heroku platform-level process management; remains available under varying load with no manual intervention.

Evidence: doc4 L5–8 — single `standard-1x` dyno; `heroku-postgresql:standard-0` running on the same dyno as the application; Read Replicas: None; Separate DB Instance: No.

Gap: Stated redundancy/failover does not hold at the application or database tier as configured.

**Finding 5: 120 active customers — [partial]**

Claim (doc1 L9, L13): loyal customer base of 120 active accounts / Active Customers: 120.

Evidence: doc6 L3, L6–18 — 120 marked ‘active’; 94 Stripe-linked; 3 enterprise wire; 23 phantom/inactive (created Q1 2026, no payment events in last 90 days, last login 60–120 days ago). doc5 L4 — 94 active Stripe subscriptions.

Gap: Paying population is 97 (94+3), not 120. CRM status field still marks all 120 active.

**Finding 6: Blended MRR $40,000 — [real, operational caveat]**

Claim (doc1 L5, L9, L12): blended MRR $40,000; asking price $480,000 at 12× blended MRR.

Evidence: doc5 L5, L17, L19 — Stripe MRR $16,000; three enterprise contracts $8,000/mo each via wire, not in Stripe. 16000 + 3×8000 = 40000.

Gap: None on the arithmetic if wire revenue is included. Caveat: $24,000/mo is three annual contracts (doc7), one renewing 2026-11-01 (GlobalMart); 94 month-to-month can cancel at cycle end (doc7 L29–32). Multiple is on a mix, not on Stripe alone.

**Finding 7: Comprehensive test coverage across critical paths — [delta]**

Claim (doc1 L21; doc9 L7): well-documented codebase with comprehensive test coverage across all critical paths.

Evidence: doc3 L24–32 — CI/CD none; unit tests 12, all in `test/utils/`; integration tests 0; payment-path tests 0; no staging, branch protection, or documented code review.

Gap: Critical-path coverage as stated is not present.

**Finding 8: No lock-in / standard SaaS agreements — [delta]**

Claim (doc1 L22; doc9 L9): all third-party integrations on standard SaaS agreements with no lock-in / no proprietary lock-in.

Evidence: doc8 L7–12 — DataEnrich.io: 40% of features depend on this API; 90-day termination either party; fallback none; product degrades to basic mode. doc8 L25–28 — Google OAuth: no contractual relationship; if Google revokes the app, users cannot log in. doc7 L11, L19, L27 — enterprise required features include product matching / price comparison / classification (DataEnrich functions).

Gap: Material feature and auth concentration is not “no lock-in.”

**Finding 9: Managed DNS — [partial]**

Claim (doc9 L11): “We use a modern Rails 7 stack with automatic SSL and managed DNS.”

Evidence: doc4 L26–28 — SSL Heroku-managed auto-renewed; DNS GoDaddy, managed personally by ‘dave’; no secondary DNS. doc8 L29–32 — if domain expires or GoDaddy has an issue, the app is unreachable.

Gap: SSL matches. DNS is personal, not platform-managed, and is a single control point.

**Finding 10: Horizontal scale / one-click dynos — [partial]**

Claim (doc9 L8; doc2 L3): stack designed to scale horizontally — adding dynos is a one-click Heroku dashboard operation; automatic scaling.

Evidence: doc4 L5–6 — one `standard-1x` dyno; database on the same dyno. Heroku can add dynos; co-located Postgres is not a horizontal data tier.

Gap: Dashboard scaling of web dynos is plausible; “automatic scaling” and a scalable data tier are not what is deployed.

**Finding 11: Stripe payment processing — [real]**

Claim (doc1 L20; doc2 L5): secure payment processing via Stripe; credit cards and recurring billing.

Evidence: doc5 entire; doc8 L3–6. Stripe is live with 94 subscriptions.

Gap: None for the Stripe path. Enterprise cash is wire, outside Stripe (already in Finding 6).

**Finding 12: Rails 7 + automatic SSL — [real, minor caveat]**

Claim (doc1 L18; doc9 L11; doc2 L3): modern Rails 7; automatic SSL.

Evidence: seller statements in docs 1, 2, 9; SSL in doc4 L26. No application source in the data room to confirm Rails major version independently.

Gap: SSL observed. Rails 7 is unverified in code (data room has no repo tree); treated as uncontradicted seller statement, not independently confirmed.

Coverage statement: 22/66 verified. Not checked in depth: “revenue-positive,” “immediate cash flow,” “minimal operational overhead,” “turnkey,” “customers are engaged,” “technology is proven,” Redis session-caching performance, Twilio notification path, exact Heroku config-var contents, whether 12 unit tests pass. Those do not change recoverability, control-point, or customer-count findings. Addendum rule: surface frozen at 66; later claims do not enter the denominator.

Ask the seller before closing: (1) a restore test from the 2026-07-30 backup and whether any copy exists off Heroku; (2) who will hold GoDaddy/DNS and Heroku after close, and whether dave’s personal control transfers; (3) DataEnrich contract and a written fallback plan; (4) Google OAuth app ownership transfer; (5) evidence for the 23 CRM actives and for GlobalMart renewal; (6) source tree sufficient to confirm Rails 7 and the test suite.

=== LIMITATIONS ===
Materials: nine markdown data-room documents (doc1–doc9), as-of this snapshot; last commit and last successful backup both dated 2026-07-30 in those files.
The seller was not consulted and has not confirmed the auditor’s reading of their own claims.
Limited assurance over a disclosed subset of the claim surface (22 of 66 claims verified); the conclusion is expressed positively over that subset only.
