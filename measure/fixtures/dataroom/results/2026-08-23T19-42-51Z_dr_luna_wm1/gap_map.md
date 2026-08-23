**FlowMetrics — B2B e-commerce analytics SaaS**

**Recommendation: Material**

- **Backups and recovery:** Daily backups are claimed, but the infrastructure record shows 21 days of failures, no failure alerting, and a last successful backup dated 2026-07-30.
- **Testing controls:** Comprehensive critical-path coverage is claimed; the recorded repository has 12 utility-only unit tests, no integration or payment-path tests, and no CI/CD or staging environment.
- **Dependency resilience:** No vendor lock-in is claimed; DataEnrich.io supports 40% of features and has no implemented fallback.
- **Customer base:** 120 active accounts are claimed; 23 CRM accounts are labelled phantom/inactive, with no payment events in 90 days and old last-login dates.
- **Continuity posture:** Heroku-managed SSL and Stripe processing are supported, but DNS is personally managed by Dave, there is no secondary DNS provider, and 14 secrets are held in config vars.

**Coverage:** Roughly 30–40 micro-claims were identified across nine documents. Load-bearing reliability, architecture, testing, dependency, customer, revenue, and selected security/operations claims were checked. Undocumented security controls, access ownership, independent uptime, incident response, and source-code implementation remain unresolved.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
