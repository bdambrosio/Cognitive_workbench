**FlowMetrics** — B2B e-commerce analytics SaaS, $480k ask.

**MATERIAL**

- Recoverability: daily backups claimed; 21 days of failures; last success 2026-07-30; stated 30-day retention exhausts 2026-08-29; no backup alerts.
- Customers / MRR mix: $40k is $16k Stripe + $24k wire; 120 “active” includes 23 non-paying phantoms; 94 MTM can cancel at cycle end.
- Architecture: single standard-1x dyno, Postgres on the same dyno, no replicas — not the claimed redundancy/failover; no app uptime monitor against 99.9% claims and enterprise SLAs.
- Control points: DataEnrich.io is 40% of features, 90-day out, no fallback; DNS is dave’s GoDaddy; Google OAuth has no contract.
- Tests: 12 utility unit tests, zero payment-path or integration tests, no CI — not comprehensive critical-path coverage.

Coverage: 22/66 claims verified (tiers 1–3 plus selected micro-claims). Remaining listing adjectives are lower-cost unknowns than the gaps above.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice
