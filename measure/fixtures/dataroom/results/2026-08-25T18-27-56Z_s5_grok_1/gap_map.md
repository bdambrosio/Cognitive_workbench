**FlowMetrics** — B2B e-commerce analytics SaaS, asking $480k.
**Material** — 22 of 33 claims checked.
- Backups scheduled but failed 21 days; last success 2026-07-30; claimed 30-day retention ages out **2026-08-29**.
- No uptime monitor; single `standard-1x` dyno with Postgres on the same dyno; no replicas — redundancy/failover/99.9% monitoring do not hold.
- “120 active” is a CRM flag; 23 have no payment in 90 days (97 paying: 94 Stripe + 3 wire).
- $40k blended MRR arithmetic holds only if three $8k wire contracts are counted (60% of MRR); not in Stripe.
- DataEnrich.io: 90-day termination, 40% of features, no fallback — contradicts no-lock-in.
- Tests: 12 utility unit tests, 0 payment-path, 0 CI.
Coverage: 22/33 checked; remainder is low-impact copy. Checked set is not consistently true, so the unchecked remainder is not assumed low-risk.
Full report with citations available on request.
technical claims verification · not a pen-test, not legal advice
