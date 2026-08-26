FlowMetrics — SaaS data and payment platform

Recommendation: **Material**

Key items:
- **Backup recovery:** 21 consecutive failure days, with no alerting; the last successful backup reaches the stated retention boundary on 2026-08-29.
- **Redundancy:** The database shares the application dyno, with no replicas or separate database; the documented failover claim does not hold.
- **Testing:** Utility-only tests leave integration and payment paths untested.
- **Payment and vendor risk:** 14 secrets are stored without a vault, and DataEnrich supplies 40% of relevant data without fallback.
- **Customer records:** 23 accounts are identified as phantom or inactive; eight are free pilots and fifteen lack recorded signed Terms of Service acceptance.

Coverage: 15 of 286 claims assessed; 271 unexamined. The assessed findings include material contradictions, so the remainder is not low-risk by default.

Full report with citations available on request.

Technical claims verification · not a pen-test, not legal advice
