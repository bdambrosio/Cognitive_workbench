# Answer key — synthetic data room

**Do not place this file where an agent under test can read it.**

Provenance: recovered from the Jill/Jack exchange of 2026-08-20 (trace rows
2411–2634 of the archived world at `scenarios/jill_chat.bak`). The corpus was
built by Jill as a *format* test — "does the report template handle three
finding types" — not as a discovery test. Two consequences follow, and both
matter for using it as a benchmark:

1. **A leak was removed.** Document 9 originally ended with an "Omissions
   Note" enumerating the planted answers outright. It is reproduced at the
   bottom of this file and has been stripped from `corpus/`. If you regenerate
   the corpus from the trace, strip it again.
2. **The corpus supports more findings than were planted.** Entries 4–5 in the
   original exchange were discovered, not seeded. So an agent surfacing an
   unplanted-but-supported finding is being *correct*, and a grader that counts
   it as a false positive is measuring the wrong thing.

Total corpus: 9 documents, ~11.3k characters. Company: `flowmetrics`, seller
`dave`.

## Tier 1 — the three planted findings (must-find)

| # | Type | Claim | Contradicting evidence | Severity |
|---|---|---|---|---|
| P1 | `vocabulary_gap` | Doc 9: "redundancy and automatic failover through Heroku's platform-level process management" | Doc 4: single `standard-1x` dyno, Postgres **on the same dyno**, no read replicas, no separate DB instance. Dyno restart is not failover; there is no redundancy to fail over to. | High |
| P2 | `operational_degradation` | Doc 9: "daily automated database backups"; "backed up daily … with 30-day retention" | Doc 4: "Failures recorded for the last 21 days", last successful backup **2026-07-30**, alerting **none configured**. | Critical |
| P3 | `structural_omission` | Doc 9: "All third-party integrations are on standard SaaS agreements with no lock-in"; DataEnrich.io named nowhere in Docs 1, 2 or 9 | Doc 8: DataEnrich.io, **$400/mo**, **40% of features depend on it**, 90-day termination either party, **no fallback implemented**. | Critical |

## Tier 2 — derived findings (arithmetic on stated figures)

- **F1 — revenue verifiability.** Doc 5: Stripe MRR **$16,000**. Doc 7: three
  enterprise contracts at **$8,000/mo each = $24,000/mo**, paid by wire and
  explicitly *not* in Stripe. Against a claimed ~$40k, **60% of revenue is
  unverifiable from the payment processor**, and wire revenue is **150% of**
  Stripe MRR. (Beware: the original exchange contains a wrong "28%" figure and
  a corrected 10x error. Neither is in the corpus. 28% is not a correct answer.)
- **F2 — backup expiry.** 30-day retention against 21 days of failures puts the
  last recoverable backup (2026-07-30) at **total loss around 2026-08-29**. This
  is P2 escalated by arithmetic, and is the strongest single finding available.

## Tier 3 — supported but unplanted (credit, never penalise)

- **B1** Doc 9 "comprehensive test coverage across all critical paths" vs Doc 3:
  12 unit tests all in `test/utils/`, **0 integration, 0 payment-path**, no
  CI/CD, no staging, no branch protection, no code review.
- **B2** Doc 9 "We monitor uptime at 99.9%" vs Doc 4 "Uptime Monitor: **None**"
  — while Acme and GlobalMart hold **99.9% SLAs** with 10%-per-0.1% credits.
  The SLA is contractually owed and not measured.
- **B3** Key person. Doc 3: dave = 73% of commits, **0 commits in 30 days**,
  last commit **2026-07-30** — the same date as the last successful backup.
  Doc 4: DNS "managed personally by 'dave'", no secondary DNS.
- **B4** SLA heterogeneity: ShopStream **99.5%** where the other two are 99.9%.
- **B5** Legal: **15 accounts with no recorded ToS acceptance**; 8 pilots
  expiring Q3 2026 with no payment obligation.
- **B6** Doc 9 "scale horizontally — adding dynos is one-click" vs Doc 4's
  co-located database, which adding dynos does not scale.
- **B7** Concentration: 3 customers ≈ 60% of revenue; GlobalMart renews first
  (2026-11-01), inside the 90-day non-renewal notice window.

## Severity ordering (for a length-bounded deliverable)

If the deliverable is a one-pager, the forced choice is the measurement. Ranked:
**P2/F2 → P3 → P1 → F1 → B3 → B2 → B1 → B7 → B5 → B4 → B6.**
Score whether the top-ranked findings made the page, not merely whether they
were found somewhere.

## The removed leak (was the tail of Document 9)

    *   The seller does NOT mention DataEnrich.io.
    *   The seller does NOT mention that PostgreSQL is on the same dyno.
    *   The seller does NOT mention the lack of CI/CD or test coverage.
    *   The seller does NOT mention the single-contributor Git history.
    *   The seller does NOT mention that the backup has been failing for 3 weeks.
