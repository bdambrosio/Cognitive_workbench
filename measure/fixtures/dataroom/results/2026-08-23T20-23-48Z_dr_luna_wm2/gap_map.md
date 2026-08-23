**FlowMetrics — SaaS claims verification**

**Recommendation: CONDITIONAL**

**Key items**

- **Backups:** Daily scheduling exists, but the infrastructure record shows 21 days of failures, a last successful backup of 2026-07-30, and no failure alerting.
- **Monitoring:** The claimed 99.9% uptime monitoring is not configured in the supplied record.
- **Account quality:** 120 accounts are marked active, while 23 are separately identified as phantom/inactive.
- **Testing:** The supplied repository record shows 12 utility unit tests, zero integration tests, and zero payment-path tests.
- **Continuity and lock-in:** The single-dyno/no-separate-DB topology does not establish database failover, and Google OAuth is recorded as having no contractual relationship.

**Coverage:** Nine data-room documents reviewed; approximately 150 micro-claims enumerated. Six material findings were identified, while low churn, payment security, operating overhead, scaling evidence, and several implementation claims remain unverified because the supplied materials do not contain the necessary supporting records.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
