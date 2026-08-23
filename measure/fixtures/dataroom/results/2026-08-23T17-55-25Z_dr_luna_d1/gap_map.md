**FlowMetrics — B2B e-commerce analytics SaaS**

**Recommendation: Material**

- **Reliability:** Claimed monitoring and daily backups do not match the documented 21-day backup failure and absence of uptime monitoring.
- **Architecture:** Database redundancy, automatic failover, and horizontal scaling are not present in the documented deployment.
- **Operations:** Testing, CI/CD, staging, alerting, and review controls are materially thinner than represented.
- **Commercial evidence:** $40,000 MRR is plausibly reconciled, but active-account and low-churn claims require substantiation.
- **Dependencies and security:** DataEnrich creates a material dependency without fallback; authorization, encryption at rest, secret rotation, rollback, and recovery testing remain unresolved.

**Coverage:** Approximately 195 micro-claims were identified across nine documents; load-bearing reliability, architecture, operational, commercial, and dependency claims were prioritized. Several security, recovery, and documentation claims remain unverified, and three requested documents were absent with their names not recorded.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
