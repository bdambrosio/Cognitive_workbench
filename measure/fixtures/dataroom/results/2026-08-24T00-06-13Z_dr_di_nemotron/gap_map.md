**Target**: FlowMetrics — B2B e-commerce analytics SaaS

**Verdict**: **Material** (red band)

**Key Items**

🔴 **No redundancy, failover, or auto-scaling** — Listing claims all three; infrastructure is a single standard-1x dyno with no replicas. A buyer pricing for HA is buying a single point of failure.

🔴 **No uptime monitoring** — Claimed 99.9% monitoring; actual: none configured, only Heroku status page. Failures go undetected until a customer reports them.

🔴 **No backup failure alerting** — Backups failed for 21 days (last good 2026-07-30) with zero notification. Derived: retention window exhausts 2026-08-29, after which no recoverable backup exists.

🟡 **Critical vendor dependency, no fallback** — DataEnrich.io API ($400/mo) drives 40% of features; 90-day termination clause; seller explicitly admits no fallback. Migration risk is unpriced.

🟢 **Revenue/customer metrics verified** — All 26 micro-claims (MRR $4,200, 14 customers, 0% churn, contract terms) cross-verified across financial docs.

**Coverage**: 60/62 claims individually verified (97%). 2 unverifiable (Rails version labels — does not affect operational posture).

Full report with citations available on request.

---

*Technical claims verification · not a pen-test, not legal advice*
