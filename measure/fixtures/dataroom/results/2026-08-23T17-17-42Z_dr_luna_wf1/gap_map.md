**FlowMetrics — buyer-side claims verification**

**Recommendation: CONDITIONAL**

- **Reliability:** Daily backups are scheduled, but they have failed for 21 days with no alerting; no independent uptime monitor exists.
- **Architecture:** The application and database share one dyno, with no replica or separate database instance.
- **Testing:** The claimed comprehensive critical-path coverage contrasts with 12 utility unit tests, zero integration tests, and zero payment-path tests.
- **Commercial claims:** $40,000 MRR reconciles, but 120 “active” customers includes 23 records explicitly marked phantom/inactive.
- **Dependencies:** 40% of features depend on DataEnrich.io with no fallback; the blanket no-lock-in claim is not supported, and DNS has no secondary provider.

**Coverage:** Roughly 30–40 claims were identified across nine documents; the load-bearing reliability, architecture, testing, customer, dependency, DNS, security-presence, MRR, and contract claims were individually checked. Production execution, backup restoration, penetration testing, and legal enforceability were not tested.

Full report with citations available on request.

*Technical claims verification · not a pen-test, not legal advice*
