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
