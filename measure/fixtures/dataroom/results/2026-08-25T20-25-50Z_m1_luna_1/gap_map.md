**FlowMetrics** — B2B e-commerce analytics SaaS platform

**Recommendation: Material**

- **MRR and customer count:** Listing states $40,000 MRR and 120 active customers; evidence reconciles the revenue only by adding three wire contracts and identifies 23 phantom/inactive CRM accounts.
- **Data recoverability:** Daily backup claim is undermined by 21 days of failures, no alerting, and no successful backup after 2026-07-30; the stated 30-day window reaches its boundary on 2026-08-29.
- **Availability architecture:** Claimed redundancy and failover are not supported by a single dyno/database placement with no replicas or separate DB instance.
- **Uptime and testing:** No uptime monitor exists, and the repository has no integration or payment-path tests despite the comprehensive-coverage claim.
- **Dependency and control risk:** DataEnrich supports 40% of features with no fallback; DNS is personally managed by Dave with no secondary provider.

**Coverage:** 18 of 62 claims checked individually, prioritising business-ending, architectural, and operational claims; 44 lower-priority claims remain unchecked.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
