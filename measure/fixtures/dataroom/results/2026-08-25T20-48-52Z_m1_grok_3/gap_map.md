FlowMetrics — small B2B e-commerce analytics SaaS, asking $480k.

**Material** (22 of 34 claims examined).

- 120 “active” accounts: CRM shows 23 phantoms; 97 paying (94 Stripe + 3 wire).
- Backups failed 21 days; last success 2026-07-30; claimed 30-day retention exhausts 2026-08-29; no alerting.
- No uptime monitor vs 99.9% claim and enterprise SLAs.
- Single standard-1x dyno, Postgres on the same dyno, no replicas vs redundancy/failover/horizontal scale.
- 12 util unit tests, 0 integration, 0 payment-path, no CI vs comprehensive critical-path coverage.
- DataEnrich: 40% of features, 90-day out, no fallback; DNS personal to Dave.

Holds: $40k blended MRR is $16k Stripe + $24k three wires; 12× asking math; Rails/Heroku/Stripe/Redis/SSL identity.

Coverage: 22/34 checked; remainder is low-impact marketing. Recoverability and concentration are not low-risk.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice
