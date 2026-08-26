**Target:** FlowMetrics — B2B e-commerce analytics SaaS, $480,000 asking price
**Recommendation:** Material

**Key Items:**
1. No working automated backups; last recoverable backup expires 2026-08-29. No alerting on failures.
2. 99.9% uptime SLA is unmeasured — no uptime monitor exists. Enterprise contracts carry SLA remedies the seller cannot trigger.
3. No platform redundancy — single dyno hosts both app and database. No failover.
4. Test coverage: 12 unit tests in utils, zero integration tests, zero payment-path tests. "Comprehensive" is false.
5. 120 "active" accounts: 23 have no payment in 90 days, 8 are free pilots, 15 have no signed agreement. Paying contracted customers: 97.

**Coverage:** 29 of 29 claims examined. 4 delta, 5 partial, 13 real/caveat, 7 unverifiable.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice
