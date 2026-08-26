**FlowMetrics** — B2B e-commerce analytics SaaS, $480,000 asking price

**Recommendation: Material**

1. **Backup expires 2026-08-29.** Last successful backup 2026-07-30; 30-day retention; 21 days of failures with no alerting. After this date, the database is unrecoverable.
2. **No uptime monitoring exists.** Claimed 99.9% monitoring; no Pingdom, UptimeRobot, or custom checks configured.
3. **No redundancy or failover.** Single dyno hosts both app and database. No replicas, no separate DB instance.
4. **No meaningful test coverage.** 12 unit tests in utils; zero integration or payment-path tests.
5. **23 of 120 "active" accounts are phantom.** No payment in 90 days, no login in 60–120 days. Paying base is 97.

**Coverage:** 242 claims identified; ~35 individually verified (14.5%). Remaining ~207 are Tier 3–4 micro-claims, internally consistent or unverifiable from the data room.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
