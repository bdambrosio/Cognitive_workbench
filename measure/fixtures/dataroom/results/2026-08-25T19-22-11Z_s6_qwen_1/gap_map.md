**FlowMetrics** — SaaS platform for retail product analytics and pricing intelligence.

**Recommendation: Material.** 7 `[delta]` and 6 `[partial]` findings out of 28 claims examined. The buyer must price these gaps explicitly.

Key items:

- **Backups failing 21 days; last good backup expires 2026-08-29.** No alerting. After that date, no recoverable backup exists.
- **No redundancy, no failover, no uptime monitoring.** Single dyno, single DB, no replicas. The claims of redundancy, failover, and 99.9% uptime monitoring are all false.
- **No test coverage on critical paths.** 12 unit tests in a utility directory, 0 integration tests, 0 payment-path tests.
- **40% of features depend on DataEnrich.io with no fallback.** 90-day termination notice. The "no proprietary lock-in" claim is false.
- **23 of 120 "active" accounts are phantom.** No payment events in 90 days. True active accounts: 97.

Coverage: 28 of 32 claims examined. 4 unexamined (low-priority, no evidence available). Consistency rate: 50%.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
