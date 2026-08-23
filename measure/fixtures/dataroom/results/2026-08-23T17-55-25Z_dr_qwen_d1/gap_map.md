**FlowMetrics** — B2B e-commerce analytics SaaS, 120 accounts, $40k MRR, asking $480k

**Recommendation: Material**

| # | Item | Note |
|---|------|------|
| 1 | Backups failing 21 days; last good backup expires 2026-08-29 | No alerting. After Aug 29, no recoverable state exists. |
| 2 | 99.9% uptime claimed; no monitor exists | SLA contracts (3 × $8k/mo) obligate 99.9%/99.5%. Unmeasured. |
| 3 | "Comprehensive test coverage" claimed; 12 unit tests, 0 integration | Payment path, API integration, and billing flow are untested. |
| 4 | 120 active accounts claimed; 23 are phantom | No payment 90 days, no login, no documentation. Functional base: 97. |
| 5 | "Platform-level redundancy" claimed; single dyno, DB co-located | No replicas, no separate DB, no failover path. |
| 6 | MRR $40k verified ($16k Stripe + $24k wire) | Arithmetically consistent. 60% in 3 annual contracts with SLA. |

**Coverage:** 191 claims enumerated; 13 findings reported across all 4 priority tiers. Source code and running system not available in data room — claims verified against seller-provided documents only.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
