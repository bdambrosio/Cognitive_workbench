**FlowMetrics** — B2B e-commerce analytics SaaS, $480K asking (12× MRR)

**Recommendation: MATERIAL**

Six deltas and one partial in the first eleven findings. The reliability and code-quality claims do not hold. The revenue number is real; the operational story behind it is not.

**Key items:**

⚠️ **Backups failing 21 days; last good backup expires in 6 days.** Daily backup schedule exists but has been failing since 2026-07-30. No alerting configured. The 30-day retention window closes 2026-08-29. After that date, the business cannot be restored.

⚠️ **No uptime monitoring despite 99.9% claim.** No Pingdom, no UptimeRobot, no custom checks. The Heroku status page monitors Heroku, not the application. The seller is contractually bound to 99.9% SLAs (three enterprise contracts) with no means of monitoring them.

⚠️ **Single dyno, co-located database, no redundancy.** "Platform-level redundancy and automatic failover" is a description of Heroku's crash-restart behaviour, not an architectural property. There is no second instance, no replica, no failover target.

⚠️ **23 of 120 "active" accounts are phantom.** No payment in 90 days, no login in 60–120 days, no notes on file. True active base: 97. Stated base overstates by ~24%.

⚠️ **40% feature dependency on DataEnrich.io, no fallback, 90-day termination notice.** "No lock-in" is false. If the API degrades or pricing changes, the product drops to basic mode with no alternative.

**Coverage:** 11 of ~30 claims individually verified, covering all 7 verbatim technical claims and the core financial/account claims. Remaining ~19 are contract terms, dependency facts, and design claims requiring code access or seller confirmation. The unverified items are less expensive to a buyer than the deltas already found, but the enterprise SLA terms are directly relevant to the uptime gap.

Full report with citations available on request.

*technical claims verification · not a pen-test, not legal advice*
