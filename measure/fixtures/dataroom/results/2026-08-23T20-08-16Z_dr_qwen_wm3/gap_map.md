**FlowMetrics** — B2B e-commerce analytics SaaS, $40K MRR, 120 claimed accounts, Rails 7 / Heroku / PostgreSQL / Stripe.

**Verdict: Conditional**

| # | Item | Note |
|---|------|------|
| 1 | Backups not running for 24 days, no alerting | Last good backup 2026-07-30; expires from retention 2026-08-29 (6 days). Most time-critical item. |
| 2 | No uptime monitoring exists | No Pingdom, UptimeRobot, or custom checks. The 99.9% figure is unmeasured. |
| 3 | Zero test coverage on critical paths | 12 utils tests, 0 integration, 0 payment-path. No CI/CD, no staging, no branch protection. |
| 4 | MRR of $40,000 verified | $16K Stripe + 3×$8K wire, confirmed against Stripe export and contracts. The revenue is real. |
| 5 | Stack and security basics are as stated | Rails 7, managed SSL, Google OAuth, Stripe payments, PostgreSQL, Redis — all confirmed. |

**Coverage:** 22 of 24 claims individually verified. 2 remain unverifiable (documentation content, product-market fit) — neither load-bearing for valuation.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
