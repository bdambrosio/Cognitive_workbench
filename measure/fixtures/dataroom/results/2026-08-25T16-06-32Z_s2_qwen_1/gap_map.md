**FlowMetrics** — B2B e-commerce analytics SaaS, $480,000 asking (12x blended MRR)

**Recommendation: Material**

Key items:

- **Backups failing 21 days; last recoverable backup ages out 2026-08-29.** No alerting. After that date, no recovery point exists. [derived, escalates Finding 2]
- **Uptime monitoring: none.** Seller claims 99.9% monitoring. No Pingdom, UptimeRobot, or custom checks. Contractual SLAs of 99.9%/99.5% exist but cannot be measured. [delta]
- **Test coverage: 12 utility tests, 0 integration, 0 payment-path.** Seller claims "comprehensive test coverage across all critical paths." The payment path processing $40k/mo has no tests. [delta]
- **No redundancy.** Single dyno runs app and database. Seller claims "platform-level redundancy and automatic failover." No failover target, no read replica, no separate DB instance. [delta]
- **23 of 120 "active" accounts are phantom.** No payment in 90 days, no login in 60-120 days. Defensible active count is 97, not 120. 19% overstatement affects the MRR multiple. [partial]

Coverage: 20 of 93 claims individually verified with citations. 3 unverifiable (no source code, no churn data, no retention config in doc4). 70 verified in aggregate for internal consistency. The 4 [delta] and 1 [derived] findings are the material items; the 6 [partial] findings are gaps to price.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
