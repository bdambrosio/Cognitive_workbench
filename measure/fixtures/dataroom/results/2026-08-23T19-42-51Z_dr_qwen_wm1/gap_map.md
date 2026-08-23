# FlowMetrics — Gap Map

**B2B e-commerce analytics SaaS · $480k asking · $40k MRR**

**Recommendation: Material**

---

**Key Items**

⚠️ **Backups failing 21 days, retention window closes 2026-08-29.** Last good backup 2026-07-30. No alerting. After 2026-08-29, 24+ days of data exist only in the live DB with no copy. [delta + derived]

⚠️ **No uptime monitoring exists.** Claimed at 99.9%; no monitor of any kind is configured. The figure is unmeasured. [delta]

⚠️ **Payment path has zero test coverage.** 12 unit tests, all in `test/utils/`. No integration tests, no CI/CD, no staging. [delta]

⚠️ **Single point of failure.** One dyno, DB on the same instance, no read replicas, no failover. [delta]

⚠️ **40% of product depends on DataEnrich.io.** No fallback. 90-day termination either party. [delta]

**Coverage:** 57/59 claims verified. 2 unverifiable (churn, documentation quality). Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
