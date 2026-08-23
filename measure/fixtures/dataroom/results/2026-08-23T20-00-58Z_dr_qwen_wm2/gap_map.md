**FlowMetrics** — B2B e-commerce analytics SaaS, 97 paying accounts, $40K MRR, Rails 7 on Heroku. Asking $480K.

**Recommendation: Material**

**Key items**

⚠ Last successful backup was 2026-07-30. On the seller's own 30-day retention, it expires 2026-08-29 — six days from now. After that, there is no backup. No alerting was configured for the 21 days of backup failures.

⚠ "99.9% uptime monitoring" and "comprehensive test coverage across all critical paths" are false. No uptime monitor exists. 12 unit tests in a utility directory; zero integration or payment-path tests; no CI/CD.

⚠ "Redundancy and automatic failover" and "designed to scale horizontally" describe infrastructure that does not exist. Single dyno, single database, no replicas, no failover path.

⚠ "120 active accounts" includes 23 that have not paid in 90+ days and 8 free pilots. True paying base: 97. The CRM status field is not maintained.

✓ The $40,000 MRR figure is arithmetically correct ($16K Stripe + $24K wire). The Rails 7 / SSL stack is as described.

**Coverage:** 14 of ~28 claims verified (50%). All safety-critical and architectural claims verified. Remaining claims are unverifiable from the data room (no codebase, no financials, no product demo) or non-delta marketing language.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
