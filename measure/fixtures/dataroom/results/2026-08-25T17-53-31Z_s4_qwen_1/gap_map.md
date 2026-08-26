**Target:** FlowMetrics — B2B e-commerce analytics SaaS, Heroku-hosted, $480k asking price.

**Recommendation: Material**

**Key items:**

1. **Backups failing for 21 days.** Last successful backup 2026-07-30. No alerting. Last recoverable backup ages out on 2026-08-29 — four days from the as-of date.
2. **120 active accounts claimed; 97 are active.** 23 accounts show no payment or login activity in 90 days.
3. **No uptime monitoring.** The 99.9% SLA in enterprise contracts is unmeasured and unenforceable.
4. **No test coverage on the payment path.** 12 unit tests in utils, 0 integration tests, 0 payment-path tests.
5. **Practical lock-in despite "no lock-in" claim.** DataEnrich.io (40% of features, no fallback), single-person DNS management.

**Coverage:** 28 of 28 claims examined. 9 [delta], 8 [partial], 6 [real], 1 [real, minor caveat], 2 [real, operational caveat], 2 [unverifiable].

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
