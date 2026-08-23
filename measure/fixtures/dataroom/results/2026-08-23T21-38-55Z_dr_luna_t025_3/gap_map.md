# FlowMetrics — claims-verification gap map

**Recommendation: Material**

- **Backups:** Daily scheduling is documented, but records show 21 days of failures, no failure alerting, and no successful backup after 2026-07-30.
- **Resilience:** The seller claims Heroku redundancy and automatic failover; the supplied configuration shows one dyno, a co-located database, no replicas, and no separate database instance.
- **Commercial quality:** The listing says 120 active accounts, but the CRM identifies 23 phantom/inactive accounts; three enterprise customers represent 60% of stated MRR.
- **Operational controls:** The claimed 99.9% uptime monitoring is not supported by any application monitor; testing records show no integration or payment-path tests.
- **Ownership and dependencies:** DNS is managed personally by the seller, and the platform depends on multiple external vendors; security-control detail remains incomplete.

**Coverage:** Roughly 70–80 micro-claims were identified across nine documents. The material reliability, architecture, operational, dependency, and commercial claims were prioritised. Churn, engagement, detailed security controls, documentation quality, and continuous subscription retention remain unresolved where the data room lacks the necessary evidence.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
