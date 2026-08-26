Target: FlowMetrics — subscription software business with Stripe, wire-transfer enterprise accounts, Heroku infrastructure, and third-party dependencies.

Recommendation: Material

Key items:
- Backup recovery is not supported; the last successful backup was 2026-07-30, with the stated 30-day retention boundary expiring on 2026-08-29 if no later successful backup exists.
- The supplied infrastructure evidence does not demonstrate claimed redundancy or automatic failover: one dyno hosts the application and PostgreSQL.
- No uptime monitor was found despite the 99.9% monitoring claim.
- DataEnrich.io supports 40% of features without fallback, with Google OAuth and GoDaddy as additional control points.
- Testing evidence is limited to 12 utility unit tests, with no integration or payment-path tests; the stated $40,000 MRR reconciles, while 23 of 120 CRM accounts show no payment events in 90 days.

Coverage: 15 of 273 frozen claims individually verified as of 2026-08-25; the remaining 258 were not individually verified and are not treated as deltas.

Technical claims verification · not a pen-test, not legal advice.
