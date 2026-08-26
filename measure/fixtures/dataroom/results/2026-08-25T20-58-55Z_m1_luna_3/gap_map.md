**FlowMetrics** — B2B e-commerce analytics SaaS offered at $480,000.

**Recommendation: Material** — based on 14 of 90 claims checked.

- **Daily backups:** The listing claims daily automated backups with 30-day retention; the infrastructure record shows failures for the last 21 days and no alerting.
- **Redundancy/failover:** The seller claims platform-level redundancy; the documented system has one dyno, a database on the same dyno, no read replicas, and no separate database instance.
- **Uptime monitoring:** The listing claims 99.9% monitoring; the infrastructure record says no uptime monitor is configured.
- **Revenue/accounts:** The listing says 120 active accounts and $40,000 MRR; 23 CRM accounts are labelled phantom/inactive, while 94 Stripe subscriptions and three wire-transfer accounts are separately documented.
- **Critical dependency:** DataEnrich.io supplies functionality for 40% of features, with no fallback if it is unavailable or changes pricing.

**Coverage:** 14 of 90 claims were checked, prioritising recoverability, architectural invariants, operational controls, and revenue/account integrity. The remaining 76 claims were not individually checked and are not covered by the recommendation.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
