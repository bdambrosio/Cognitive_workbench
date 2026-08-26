**Target:** FlowMetrics — B2B e-commerce analytics SaaS, Rails 7 on Heroku, $40k MRR, 120 accounts.

**Recommendation: Material**

1. **Backups failing 21 days; last good backup ages out 2026-08-29.** No alerting. Four days to unrecoverable data loss.
2. **No redundancy, no failover, no uptime monitoring.** Single dyno, co-located DB, no replicas. The "99.9% uptime" and "automatic failover" claims are false.
3. **Zero payment-path tests, zero integration tests.** "Comprehensive test coverage across critical paths" is false. 12 utility tests only.
4. **23 of 120 accounts are phantom.** No payment in 90 days. "120 active accounts" is 19% inflated.
5. **40% of features depend on DataEnrich.io with no fallback.** "No lock-in" understates the dependency.

**Coverage:** 23 of 23 claims examined. 7 hold (30% consistency rate). 11 are `[delta]`, 4 are `[partial]`, 1 is `[unverifiable]`.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
