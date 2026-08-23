**FlowMetrics — B2B e-commerce analytics SaaS**

**Recommendation: Material**

- **Backups:** Daily automated backups are claimed, but the supplied infrastructure record reports 21 consecutive days of failures, a last successful backup of 2026-07-30, and no failure alerting.
- **Reliability:** The seller claims 99.9% uptime monitoring; the record shows no application uptime monitor, only Heroku’s built-in status page.
- **Architecture:** Redundancy and automatic failover are claimed, while the documented setup has one dyno, a same-dyno database, no read replicas, and no separate database instance.
- **Testing and dependencies:** Comprehensive critical-path testing is contradicted by 12 utility-only unit tests, zero integration tests, and zero payment-path tests. DataEnrich supports 40% of features with no fallback, and DNS is personally controlled by Dave.
- **Revenue and customers:** MRR arithmetic is inconsistent, and 23 of 120 CRM accounts are labelled Phantom/Inactive with no payment events in 90 days.

**Coverage:** Approximately 100 claims were identified across nine documents; a material priority subset was verified. Code, deployment configuration, actual monitoring, backup artifacts, authentication details, secrets handling, and many lower-priority claims remain unverified.

Full report with citations available on request.

Technical claims verification · not a penetration test · not legal advice
