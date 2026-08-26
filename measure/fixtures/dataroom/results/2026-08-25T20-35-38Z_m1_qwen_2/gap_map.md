**FlowMetrics** — B2B e-commerce analytics SaaS, 120 accounts, $40k MRR, asking $480k (12x MRR).

**Recommendation: Material**

The target's most consequential claims do not hold. A buyer must price these gaps explicitly.

**What matters most:**

1. **Backups have been failing for 21 days.** Last good backup: 2026-07-30. With 30-day retention, the last recoverable backup expires **2026-08-29** — four days from the as-of date. After that, the database is unrecoverable.
2. **No uptime monitoring exists.** The seller claims 99.9% uptime monitoring. There is no monitor. Enterprise contracts carry 99.9% SLAs with service-credit remedies that cannot be triggered because no one is watching.
3. **No redundancy.** Single dyno, database on the same dyno, no replicas. "Platform-level redundancy and automatic failover" is not present. If the dyno dies, the business is down.
4. **No test coverage on the payment path.** 12 unit tests in a utilities directory. Zero integration tests. Zero payment-path tests. No CI/CD. The mechanism collecting $40k/month is untested.
5. **"No lock-in" is contradicted by the dependency list.** 40% of features depend on DataEnrich.io (no fallback, 90-day termination notice). Auth depends on Google (no contract). DNS is held by one person (no secondary).

**What holds:**

The $40,000 MRR figure reconciles from the Stripe and contract documents. The product exists and does what it says. SSL, Stripe payments, PostgreSQL, Redis, and the data enrichment API are all present and functioning as described. The revenue is real.

**Coverage:** 48 of 48 claims examined. 31 definitive verdicts, 6 unverifiable (no evidence in data room), 1 non-delta, 1 derived. No claims left unchecked.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
