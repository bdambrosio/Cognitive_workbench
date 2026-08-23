**FlowMetrics — B2B E-Commerce Analytics Platform**

**Recommendation: MATERIAL**

Key items:

1. Backups failing 21 days — last success 2026-07-30; no restorable snapshot within claimed retention after 2026-08-29.
2. No uptime monitoring — 99.9% SLA in three enterprise contracts is unmonitored and undemonstrable.
3. Single-node infrastructure — "automatic failover" and "platform-level redundancy" not configured; one dyno, one DB, no replicas.
4. 23 of 120 "active" accounts have no payment in 90 days — effective customer base is 97, not 120.
5. No test coverage on payment paths — 12 utility tests, 0 integration, 0 payment-path, no CI/CD.

Coverage: 29 findings covering all Tier 1–3 claims. ~140 claims identified; Tier 4 micro-claims not individually verified — low-risk, do not change the recommendation.

Full report with citations available on request.

Technical claims verification · Not a pen-test · Not legal advice.
