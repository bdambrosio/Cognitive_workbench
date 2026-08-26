**FlowMetrics** — B2B e-commerce analytics platform (product intelligence, price comparison, inventory sync for e-commerce sellers).

**Recommendation: Material**

Key items:

- **Backups failed 21 days; last recoverable backup expires 2026-08-29.** No alerting configured. After that date, no recoverable backup exists.
- **No platform redundancy.** Single dyno, co-located app and database. No failover. The claimed redundancy does not exist.
- **No uptime monitoring.** Claimed 99.9% monitoring is not implemented. No monitoring tool configured.
- **Zero test coverage on critical paths.** 12 utility tests only. Payment path, authentication, and data processing have no tests.
- **Active-customer count inflated 19%.** 120 claimed; 97 genuinely active. 23 phantom accounts with no payment or login in 90 days.

Coverage: 67/67 claims verified. 5 deltas, 4 partials, 1 derived, 2 unverifiable, 54 real.

Full report with citations available on request.

*Technical claims verification · not a pen-test, not legal advice.*
