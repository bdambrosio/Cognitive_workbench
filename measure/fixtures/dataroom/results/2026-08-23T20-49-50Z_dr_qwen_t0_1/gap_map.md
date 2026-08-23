**FlowMetrics** — B2B E-Commerce Analytics Platform, $480K asking (12× MRR)

**Recommendation: Material**

| # | Item | Note |
|---|---|---|
| 1 | Backup coverage expires 2026-08-29 | Last successful backup 2026-07-30; 30-day retention; 6 days of recoverability remain. After that date, no backup exists. |
| 2 | No uptime monitoring; contractual SLAs unenforceable | Two enterprise contracts ($16K/mo, 40% of MRR) promise 99.9% SLA with service credits. No monitoring tool exists to detect or quantify a breach. |
| 3 | Single point of failure | One dyno, database co-located. No redundancy, no failover, no read replicas. The "platform-level redundancy" claim is false. |
| 4 | No payment-path tests | 12 unit tests in `test/utils/`. Zero integration tests. Zero payment-path tests. The primary revenue mechanism is untested. |
| 5 | 19% of "active" accounts are phantom | 23 of 120 accounts show no payment in 90 days, no login in 60–120 days. Marked active in CRM. MRR is unaffected; account count is inflated. |

**Coverage:** 29 of 32 distinct claims verified. 4 unverifiable (no code, no churn data, no dependency manifest in data room). 1 non-delta (valuation multiple).

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
