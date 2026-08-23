**Recommendation: Material**

Four stated claims are contradicted by the data room. A derived finding puts a date on the most serious. The buyer must price these in explicitly or walk.

---

**Finding 1: Backup pipeline non-functional — [delta]**

Claim (doc1, "Technical Highlights"; doc9, line 6): "daily automated database backups with 30-day retention" / "backed up daily to Heroku's managed storage with 30-day retention."

Evidence (doc4, "Backups"): "Status: Failures recorded for the last 21 days. Last Successful Backup: 2026-07-30. Alerting: None configured for backup failures."

Delta: The last successful backup was 24 days before this audit. No alerting exists for the failure. The claim is that backups run daily; the evidence is that they have not.

---

**Finding 2: No uptime monitoring — [delta]**

Claim (doc1, "Technical Highlights"; doc9, line 2): "99.9% uptime monitoring" / "We monitor uptime at 99.9%."

Evidence (doc4, "Monitoring"): "Uptime Monitor: None (No Pingdom, UptimeRobot, or custom checks). Status Page: Heroku built-in status page only."

Delta: No uptime monitor exists. The 99.9% figure is a stated number, not a measurement. Nothing in the nine documents supports the claim.

---

**Finding 3: No critical-path test coverage — [delta]**

Claim (doc1, "Technical Highlights"; doc9, line 3): "comprehensive test coverage across all critical paths."

Evidence (doc3, "CI/CD & Testing"): "CI/CD Pipeline: None configured. Unit Tests: 12 (all located in test/utils/). Integration Tests: 0. Payment-Path Tests: 0. Staging Environment: None."

Delta: Twelve utility tests. No CI. No integration tests. Zero payment-path tests. "Comprehensive across critical paths" is not supported by anything in the data room.

---

**Finding 4: Vendor lock-in contradicted — [delta]**

Claim (doc1, "Technical Highlights"; doc9, line 5): "All third-party integrations are on standard SaaS agreements with no proprietary lock-in."

Evidence (doc8, item 2, DataEnrich.io): "Dependency: 40% of features depend on this API. Fallback: None implemented. If API is down or pricing changes, product degrades to basic mode (no enrichment)." Termination notice: 90 days, either party.

Delta: Forty percent of features depend on a single vendor with no fallback and a 90-day termination notice. That is lock-in.

---

**Finding 5: Retention window closes 2026-08-29 — [derived]**

Basis: doc1/doc9 — "30-day retention" (stated claim).
doc4, "Backups" — "Last Successful Backup: 2026-07-30."

Derivation: 2026-07-30 + 30 days = 2026-08-29. Audit date: 2026-08-23. Six days.

Consequence: If no backup succeeds before the 29th, the last recoverable database snapshot is gone. Combined with 21+ days of failures and no alerting (Finding 1), the buyer may acquire a business with no data-recovery path within a week.

Escalates: Finding 1.

---

**Finding 6: 120 "active" accounts, 23 non-paying — [partial]**

Claim (doc1): "120 active accounts" / "low churn, stable base of recurring subscribers."

Evidence (doc6): 120 marked 'active'. 94 Stripe-linked, 3 enterprise wire, 23 phantom: "Created in Q1 2026. No payment events in the last 90 days. Last login dates: 60–120 days ago. No notes or memos on file regarding creation or non-payment." Doc7 breaks the 23 into 8 pilot agreements (free 90-day trial, no payment obligation) and 15 no-contract accounts (no signed ToS on file).

Delta: 97 accounts are paying. The 23 are marked active with no payment, no recent login, no documented origin; 15 of the 23 have no signed ToS. "Low churn, stable base" is not supported; the account base carries 19% dead weight.

---

**Finding 7: Single-dyno "redundancy and failover" — [partial]**

Claim (doc1; doc2; doc9, line 1): "platform-level redundancy and automatic failover" / "automatic scaling and failover capabilities at the platform level."

Evidence (doc4): "Dyno: standard-1x (1GB RAM, 0.5 CPU). Database: heroku-postgresql:standard-0 (running on the same dyno as the application). Read Replicas: None. Separate DB Instance: No."

Delta: One dyno, database co-located. Heroku will restart a crashed dyno, but there is no second instance to fail over to and no database redundancy. Auto-restart is not failover.

---

**Finding 8: "Managed DNS" — [partial]**

Claim (doc9, line 7): "automatic SSL and managed DNS."

Evidence (doc4): "DNS Provider: GoDaddy. DNS Management: Managed personally by 'dave'. No secondary DNS provider." SSL: Heroku-managed, auto-renewed.

Delta: SSL holds. DNS is one person at a registrar with no secondary. "Managed" implies a service or a team; it is a single individual. If Dave leaves or the GoDaddy account lapses, the domain goes with it.

---

**Finding 9: SLA without a measurement — [real, operational caveat]**

Claim (doc7, enterprise contracts 1–3): 99.9% SLA (Acme, GlobalMart); 99.5% SLA (ShopStream). Remedy: service credit of 10% of monthly fee per 0.1% below SLA.

Evidence (doc4, "Monitoring"): "Uptime Monitor: None."

Delta: None — the SLAs exist as written. The operational caveat: compliance cannot be demonstrated. No telemetry, no monitor, no data source exists to measure or dispute uptime. If a customer invokes the SLA remedy, the seller has no measurement to produce.

---

**Finding 10: MRR arithmetic — [real, minor caveat]**

Claim (doc1): "Blended MRR: $40,000."

Evidence (doc5): 91 Pro × $149 = $13,559 (doc5 prints $13,549 — $10 multiplication typo). 3 Enterprise Stripe × $817 = $2,451. Stripe total: $16,010 (doc5 prints $16,000). Wire: 3 × $8,000 = $24,000 (doc7). Total: $40,010.

Delta: None. The $40,000 figure holds to the nearest $10. The $10 discrepancy is a typo in doc5, not a structural error.

---

**Finding 11: Horizontal scaling — [real, minor caveat]**

Claim (doc9, line 4): "The stack is designed to scale horizontally — adding dynos is a one-click operation in the Heroku dashboard."

Evidence (doc4): Single standard-1x dyno. No multi-dyno configuration, no session-stickiness handling, no connection-pooling evidence in the data room.

Delta: None — adding dynos is a Heroku platform capability. The caveat is that it is available, not demonstrated. No load test, no multi-dyno config, no evidence the application handles horizontal scaling correctly.

---

**Remaining Claims (unverifiable from available materials)**

- "Well-documented codebase" (doc1, doc9): no source code in the data room.
- "Rails 7" (doc2, doc9): no version file, no Gemfile, no code.
- Git contributor percentages, directory structure (doc3): internal data-room descriptions, no testable operational state.

Not findings. "I could not find the code" is not the same as "the code isn't there."

---

**Coverage**

75 claims identified across nine documents. 11 findings: 4 delta, 1 derived, 3 partial, 1 operational caveat, 2 minor caveat. Remaining ~64 claims are descriptive (Rails version, Redis caching, Stripe payments, Twilio notifications, git statistics, directory structure) and carry no testable operational state. No source code is in the data room. No live-system access (Heroku dashboard, Stripe, Google admin, DNS provider) was available. I did not verify claims that require either. Where I could not verify, I said so.

---

**What the buyer should ask the seller before closing**

1. Produce a current backup artifact or 30 days of backup logs before 2026-08-29. If no backup exists after 2026-07-30, state that and confirm a backup will be taken before closing.
2. Identify the uptime-monitoring tool (if any) and provide 90 days of uptime data. If none exists, confirm in writing that the three enterprise SLAs are unmeasurable.
3. Provide the CI pipeline URL, last 30 days of test-run logs, and the current coverage report. If the pipeline is not running, state that.
4. Provide the source-code repository (read-only) and the Rails version file.
5. Confirm the Heroku dyno count and whether a second dyno or standby is configured.
6. Provide the signed ToS for the 15 unsigned accounts, or confirm termination before closing.
7. Provide 90 days of Stripe payout history to independently confirm the 94 paying accounts.
8. Confirm the three wire-transfer contracts are current; provide the most recent payment confirmation for each.
9. Provide the DataEnrich.io contract and confirm the $400/mo rate; state whether a fallback is planned.
10. Provide the Google Cloud project ID; confirm admin access will transfer at closing.

---

=== GAP MAP ===

**flowmetrics** — B2B e-commerce analytics SaaS, $40k MRR, 97 confirmed paying accounts, single-dyno Heroku, Google OAuth sole auth

**Recommendation: Material**

1. No active backup; last recoverable database state expires 2026-08-29 — six days from audit date. No alerting on failure.
2. No uptime monitor despite three contracted SLAs (99.9% ×2, 99.5% ×1). Compliance cannot be demonstrated.
3. MRR $40k confirmed to the nearest $10 across Stripe and wire. The revenue figure is real.
4. 23 of 120 accounts (19%) are phantom: no payment, no contract, no recent login. Paying base is 97.
5. 40% of features depend on one vendor with no fallback and 90-day termination. "No lock-in" is contradicted.

Coverage: 75 claims identified · 11 findings · ~64 descriptive claims not testable from documents alone. No code. No live-system access.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*