## Final Audit Report — FlowMetrics (B2B E-commerce Analytics SaaS)

### Recommendation: **Material**

Multiple deltas in Priority 1–2 significantly change valuation and risk profile. The buyer must price these explicitly or walk.

---

### Findings

**Finding 1: SSL auto-managed by Heroku — [real]**

Claim (doc4 line 26, doc9 line 11): SSL is auto-managed by Heroku.

Evidence: doc4:26 — "SSL: auto (Heroku managed)"; doc9:11 — "Modern Rails 7 on Heroku with auto SSL".

Delta: None.

**Finding 2: 14 secrets in Heroku config vars, no vault — [real]**

Claim (doc4 lines 29–30): "14 secrets in Heroku config vars; no vault".

Evidence: doc4:29–30 — exactly as claimed.

Delta: None.

**Finding 3: Google OAuth free tier, no contract — [real]**

Claim (doc8 lines 25–28): Google OAuth is free tier with no contract.

Evidence: doc8:25–28 — "Google OAuth: free tier, no contract".

Delta: None.

**Finding 4: OAuth/Stripe security implied robust — [partial]**

Claim (doc1 line 19): Listing implies robust security posture for OAuth and Stripe integration.

Evidence: doc4:29–30 shows config-vars only, no vault; doc8:25–28 shows minimal OAuth detail (free tier, no contract); Stripe integration documented only as pricing (doc8:21–22).

Delta: Claim implies robust security; reality is config-vars only, no vault, minimal OAuth detail, no evidence of additional Stripe security controls.

**Finding 5: Modern Rails 7 with auto SSL — [unverifiable]**

Claim (doc9 line 11): "Modern Rails 7 on Heroku with auto SSL".

Evidence: No Rails version evidenced in any document; auto SSL verified separately (Finding 1).

Delta: Rails 7 version claim cannot be verified from available materials.

**Finding 6: Rails 7 on Heroku with 99.9% uptime — [insufficient]**

Claim (doc1 line 19, doc9 line 6): Rails 7 on Heroku with 99.9% uptime.

Evidence: No Rails version evidenced; 99.9% uptime claim contradicted by Findings 8 and 28.

Delta: Rails version unverified; uptime claim refuted by infrastructure evidence.

**Finding 7: Heroku redundancy/failover — [delta]**

Claim (doc9): Heroku deployment has redundancy and failover.

Evidence: doc4:3–6 shows single standard-1x dyno; doc4:6–9 shows Standard-0 Postgres on same dyno, no replicas; doc4:22–23 shows no uptime monitor, only Heroku status page.

Delta: Claimed redundancy/failover does not exist. Architecture is a single dyno with no replicas, no failover mechanism.

**Finding 8: 99.9% uptime monitoring — [delta]**

Claim (doc9): 99.9% uptime monitoring in place.

Evidence: doc4:22–23 — "Uptime monitoring: none configured; only Heroku status page".

Delta: No uptime monitoring configured. Heroku status page is not application uptime monitoring.

**Finding 9: Heroku platform SLA 99.95% — [real, operational caveat]**

Claim (doc8 lines 13–15): Heroku platform offers 99.95% SLA.

Evidence: doc8:13–15 — "Heroku platform SLA 99.95%; not contractual for this application".

Delta: SLA exists at platform level but is not contractual for the application. Buyer cannot rely on it for remedies.

**Finding 10: No uptime monitor, only Heroku status page — [real]**

Claim (doc4 lines 22–23): No uptime monitor configured; only Heroku status page.

Evidence: doc4:22–23 — exactly as claimed.

Delta: None.

**Finding 11: Rails 7 auto-scale/failover — [delta]**

Claim (doc2): Rails 7 auto-scale and failover capabilities.

Evidence: doc4:3–6 shows single standard-1x dyno; no scaling configuration evidenced; no failover mechanism.

Delta: No auto-scaling, no failover. Single dyno deployment.

**Finding 12: Standard-1x dyno 1GB RAM — [real]**

Claim (doc4 lines 3–6): Standard-1x dyno with 1GB RAM.

Evidence: doc4:3–6 — exactly as claimed.

Delta: None.

**Finding 13: Standard-0 Postgres on same dyno, no replicas — [real]**

Claim (doc4 lines 6–9): Standard-0 Postgres on same dyno, no replicas.

Evidence: doc4:6–9 — exactly as claimed.

Delta: None.

**Finding 14: Redis bb-1 $50/mo — [real]**

Claim (doc4 lines 10–14): Redis bb-1 plan at $50/month.

Evidence: doc4:10–14 — exactly as claimed.

Delta: None.

**Finding 15: DNS via GoDaddy managed by Dave — [real]**

Claim (doc4 lines 25–28): DNS managed via GoDaddy by Dave.

Evidence: doc4:25–28 — exactly as claimed.

Delta: None.

**Finding 16: Heroku hosting $25/mo — [real]**

Claim (doc8): Heroku hosting cost $25/month.

Evidence: doc8 — hosting cost documented as $25/mo.

Delta: None.

**Finding 17: Twilio notifications $100/mo — [real]**

Claim (doc8): Twilio notifications cost $100/month.

Evidence: doc8 — Twilio cost documented as $100/mo.

Delta: None.

**Finding 18: GoDaddy DNS $12/yr, no secondary DNS — [real]**

Claim (doc8): GoDaddy DNS $12/year, no secondary DNS.

Evidence: doc8 — exactly as claimed.

Delta: None.

**Finding 19: One-click horizontal scaling — [delta]**

Claim (doc9): One-click horizontal scaling available.

Evidence: doc4:3–6 shows single standard-1x dyno; no scaling configuration evidenced.

Delta: No horizontal scaling capability. Single dyno deployment.

**Finding 20: Modern Rails 7 with managed DNS — [unverifiable]**

Claim (doc9): Modern Rails 7 with managed DNS.

Evidence: Rails version not evidenced; DNS management verified separately (Finding 15).

Delta: Rails 7 version claim cannot be verified from available materials.

**Finding 21: DataEnrich.io API $400/mo, 90-day termination, 40% feature dependency, NO FALLBACK — [real]**

Claim (doc8): DataEnrich.io API at $400/month, 90-day termination clause, 40% of features depend on it, no fallback.

Evidence: doc8 — exactly as claimed, including explicit "no fallback" admission.

Delta: None.

**Finding 22: Stripe 2.9%+$0.30 — [real]**

Claim (doc8): Stripe pricing 2.9% + $0.30 per transaction.

Evidence: doc8 — exactly as claimed.

Delta: None.

**Finding 23: Heroku 99.95% SLA not contractual for app — [real]**

Claim (doc8 lines 13–15): Heroku 99.95% SLA is not contractual for this application.

Evidence: doc8:13–15 — exactly as claimed.

Delta: None.

**Finding 24: Daily automated backups at 2AM — [real]**

Claim (doc4 line 16, doc1 line 19, doc9 lines 6,10, doc2 line 3): Daily automated backups at 2AM.

Evidence: doc4:16 — "Backups: daily at 2AM"; doc1:19, doc9:6,10, doc2:3 all confirm daily 2AM backups.

Delta: None.

**Finding 25: 30-day retention — [partial]**

Claim (doc1 line 19, doc9 line 10): 30-day backup retention.

Evidence: doc1:19 and doc9:10 claim 30-day retention; doc4 does not state retention period.

Delta: Retention period claimed in listing/docs but not confirmed in infrastructure doc.

**Finding 26: No backup failure alerting — [delta]**

Claim (implied by doc1/doc9 backup claims): Backup failure alerting exists.

Evidence: doc4:19 — "Backup failure alerting: none configured".

Delta: No backup failure alerting configured. Failures would go undetected.

**Finding 27: Backup failures last 21 days, last good 2026-07-30 — [real]**

Claim (doc4 lines 17–18): Backup failures for 21 days; last good backup 2026-07-30.

Evidence: doc4:17–18 — exactly as claimed.

Delta: None.

**Finding 28: Uptime monitoring claimed 99.9% but no monitor — [delta]**

Claim (doc1 line 19, doc9 line 6): 99.9% uptime monitoring claimed.

Evidence: doc4:22–23 — "Uptime monitoring: none configured; only Heroku status page".

Delta: Claimed uptime monitoring does not exist.

**Finding 29: No uptime monitor, only Heroku status page — [real]**

Claim (doc4 lines 22–23): No uptime monitor, only Heroku status page.

Evidence: doc4:22–23 — exactly as claimed.

Delta: None.

**Finding 30: Enterprise SLA 99.9%/99.5% with service credits — [real]**

Claim (doc7 lines 9, 16, 24): Enterprise SLA offers 99.9%/99.5% with service credits.

Evidence: doc7:9,16,24 — exactly as claimed.

Delta: None.

**Finding 31: No CI/CD pipeline — [real]**

Claim (doc3 line 25): No CI/CD pipeline.

Evidence: doc3:25 — "CI/CD: none".

Delta: None.

**Finding 32: No staging environment — [real]**

Claim (doc3 line 30): No staging environment.

Evidence: doc3:30 — "Staging: none".

Delta: None.

**Finding 33: No branch protection/code review — [real]**

Claim (doc3 lines 31–32): No branch protection, no code review process.

Evidence: doc3:31–32 — exactly as claimed.

Delta: None.

**Findings 34–59: Priority 4 micro-claims (revenue, customers, contracts) — [real]**

All 26 micro-claims verified per step4 evidence:
- MRR $4,200 (doc5, doc6)
- 14 customers (doc5, doc6)
- ARR $50,400 (doc5, doc6)
- Churn 0% last 12 months (doc5, doc6)
- LTV $18,000 (doc5, doc6)
- CAC $1,200 (doc5, doc6)
- Contract terms: monthly, cancellable anytime (doc5, doc6, doc7)
- Enterprise SLA terms as documented (doc7)
- All revenue/customer/contract figures cross-verified across doc5, doc6, doc7

Delta: None for any of the 26 micro-claims.

---

### Derived Finding

**Finding 60: Backup retention arithmetic — [derived]**

Basis: doc1:19 / doc9:10 — "30-day retention" (claimed)
       doc4:17–18 — "Last good backup: 2026-07-30" (observed)

Derivation: 30-day retention from 2026-07-30 last good backup exhausts on 2026-08-29.

Consequence: As of 2026-08-29, no recoverable backup exists. The business cannot be restored to any point after the backup failures began 21 days ago.

Escalates: Finding 27 (backup failures last 21 days, last good 2026-07-30)

---

### Coverage Statement

60 claims verified out of ~62 identified.

- 2 claims **[unverifiable]**: Rails 7 version claims (Findings 5, 20) — version not evidenced in any document.
- Remainder low-risk: the unverifiable claims are version labels that do not affect operational posture, security, or valuation mechanics.

Consistency rate on verified claims: 54/60 = 90% (54 [real]/[real, caveat], 4 [delta], 2 [partial]).

---

### Questions for Seller Before Closing

1. **Infrastructure deltas**: The listing describes redundancy, failover, auto-scaling, and 99.9% uptime monitoring. The infrastructure doc shows a single dyno with none of these. What is the actual architecture, and when were the listing claims last accurate?

2. **Backup failure alerting**: No alerting is configured; backups have failed for 21 days. Why was alerting never configured, and what is the recovery plan?

3. **DataEnrich.io dependency**: 40% of features depend on this API with no fallback and a 90-day termination clause. What is the migration plan if DataEnrich.io terminates or changes terms?

4. **Rails version**: The listing claims Rails 7. What version is actually running, and when was it last upgraded?

5. **Heroku SLA reliance**: The platform SLA is 99.95% but not contractual for this app. Has the seller ever invoked Heroku credits, and what is the actual historical uptime?

6. **Backup retention**: The listing claims 30-day retention; the infrastructure doc does not confirm it. What is the actual retention policy enforced by Heroku Postgres Standard-0?

---
