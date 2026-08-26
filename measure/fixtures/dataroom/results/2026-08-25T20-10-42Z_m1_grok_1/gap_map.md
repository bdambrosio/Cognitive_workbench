**FlowMetrics** — B2B e-commerce analytics SaaS, asking $480k.

**Material** (18 of 32 claims checked).

- Backups: daily schedule, failed 21 days, last success 2026-07-30; 30-day retention exhausts **2026-08-29**.
- Architecture: one standard-1x dyno, DB on the same dyno, no replicas, no uptime monitor — not the claimed redundancy, failover, or 99.9% monitoring.
- Customers: 120 CRM-active vs 97 paying (94 Stripe + 3 wire); 23 phantoms.
- Lock-in: DataEnrich.io, 40% of features, 90-day either-party terminate, no fallback; enterprise SLAs need those features.
- Tests/CI: 12 unit tests in test/utils, 0 payment-path, no CI. DNS is Dave’s GoDaddy, not managed DNS.
- MRR $40k arithmetic holds if the three $8k wires collect; Stripe alone is $16k.

Coverage: 18/32 checked; remainder low-priority marketing language. Professional judgement: the recoverability date and the DataEnrich concentration are the items that move price or structure; they are not sprint-fixable after 2026-08-29 if the backup ages out.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice
