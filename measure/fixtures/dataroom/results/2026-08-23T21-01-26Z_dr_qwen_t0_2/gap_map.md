FlowMetrics — B2B E-Commerce Analytics Platform
Asking price: $480,000 (12x blended MRR)

RECOMMENDATION: MATERIAL

The seller's technical claims do not match the observed implementation. Three claims are false (backups, uptime monitoring, test coverage), and the backup situation is urgent: the last good backup expires in 6 days. Enterprise revenue (40% of MRR) is exposed to SLA breach with no monitoring and no fallback for the API that provides the required features.

Key items:

1. Backups failing for 21 days. Last good backup 2026-07-30, expires 2026-08-29. No alerting. After that date, zero recoverable backup.
2. No uptime monitor exists. Enterprise contracts require 99.9% SLA with no mechanism to detect a breach.
3. No integration or payment-path tests. 12 unit tests in test/utils/ only. Critical paths untested.
4. $16,000/mo enterprise revenue (40% of MRR) depends on DataEnrich.io, which has no fallback and no monitoring. 30-day termination notice for SLA breach.
5. MRR $40,000 verified. But 60% is 3 wire contracts (2.5% of customers). 23 of 120 "active" accounts show no payment in 90 days.

Coverage: 15 of ~68 claims individually verified (22%). Remaining claims are micro-claims (tech stack versions, dependency costs, commit counts) that are internally consistent and low-risk.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice
