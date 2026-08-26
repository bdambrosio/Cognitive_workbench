**Target:** FlowMetrics — B2B e-commerce analytics SaaS platform.

**Recommendation:** Material — 86 claims identified; 18 checked individually.

- Backup claim: daily automated backups are scheduled, but failures persisted for 21 days, with no alerting and no successful backup since 2026-07-30.
- Failover claim: the database shares the application dyno, with no read replica or separate database instance.
- Monitoring and testing claims: no uptime monitor, no CI/CD, no integration tests, and no payment-path tests were found in the evidence.
- Revenue/customer claims: $16,000 Stripe MRR and 94 subscriptions reconcile only with separately stated wire contracts; 23 CRM accounts are marked phantom/inactive.
- Dependency/control claims: DataEnrich.io supports 40% of features with no fallback; DNS is personally controlled by Dave with no secondary DNS.

**Coverage:** 18 of 86 claims checked, focused on business-ending risks, architectural invariants, and operational parameters; 68 lower-priority claims remain unchecked.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
