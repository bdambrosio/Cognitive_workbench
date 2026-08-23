# FlowMetrics — Claims Verification

**Recommendation: Material**

- **Backup protection:** The claimed daily backups with 30-day retention are contradicted by 21 days of recorded failures, a last successful backup on 2026-07-30, and no failure alerting.
- **Reliability evidence:** The claimed 99.9% uptime monitoring has no corresponding uptime monitor; only Heroku’s status page is recorded.
- **Architecture and testing:** Heroku is present, but the deployment records one dyno, a same-instance database, no replicas, and no separate database instance; the test inventory has zero integration and payment-path tests.
- **Customer quality and dependencies:** 23 of 120 CRM accounts have no payment events for 90 days and stale logins; DataEnrich.io supports 40% of features with no fallback, while DNS has no secondary provider.
- **What holds:** The $40,000 MRR reconciles from $16,000 Stripe MRR plus three $8,000 wire-transfer contracts; Rails 7, managed SSL, and the recurring contract structure are supported by the reviewed records.

**Coverage:** Approximately 68 claims identified across nine documents; load-bearing categories and selected commercial, security, dependency, and operational claims were checked. Lower-impact micro-claims and live-system validation remain outside this review.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
