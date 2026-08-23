**Target:** FlowMetrics — B2B e-commerce analytics SaaS, 120 claimed active accounts, $40k MRR, asking $480k (12x MRR). Seller: Dave.

**Recommendation:** DO NOT PROCEED as presented.

**Top items:**
1. Backups failing 21 days, no alerting — no recoverable data state from the last 3 weeks.
2. No uptime monitoring against contractual 99.9% SLAs — cannot demonstrate compliance; SLA breach exposure on $16k/mo of enterprise revenue.
3. 23 phantom accounts inflate customer count 24% — verifiable base is 97, not 120.
4. Single dyno, co-located DB, no replicas — "redundancy and failover" claim is unsupported; single point of failure for compute and data.
5. 60% of MRR ($24k/mo wire transfers) has no payment evidence in the data room — contract documents only.

**Coverage:** No source code, no bank statements, no actual Stripe transaction history, no churn data, no DataEnrich.io contract in the room. MRR arithmetic checks out but rests on unverified wire transfers. Technical claims are contradicted by the seller's own infrastructure and git documents.
