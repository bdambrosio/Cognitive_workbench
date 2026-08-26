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
