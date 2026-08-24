**FlowMetrics** — B2B e-commerce analytics SaaS (120 claimed active accounts, $40k claimed MRR, asking $480k)

**Material** — deltas in the reliability axis; valuation depends on unverified revenue. Borders on Walk.

Key items:
- ⚠️ **Backups failing 21 days** — last good 2026-07-30; 30-day retention expires **2026-08-29** (six days). No alerting.
- ⚠️ **"Redundancy and automatic failover"** — single dyno, DB co-located, zero replicas. No failover exists.
- ⚠️ **"99.9% uptime monitoring"** — no monitor at all; enterprise SLAs unmonitored.
- ⚠️ **$40k MRR** — $16k verified via Stripe; $24k wire unverified. If wire fails, 12x → 30x on $16k.
- ✓ **Real:** Stripe payment processing, Redis session caching, no-lock-in SaaS agreements — confirmed in config and contracts.

Coverage: 20 of ~22 claims assessed from the data room; 2 codebase-quality claims unverifiable (no code in the room).

Full report with citations available on request.

*Technical claims verification · not a pen-test, not legal advice*
