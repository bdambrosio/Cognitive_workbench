=== SUMMARY ===
**Report reviewed:** the claims-audit REPORT / LIMITATIONS / GAP MAP blocks delivered as report.md (201 lines) for the ChatterMate engagement — a buyer-side claims audit of the ChatterMate open-source AI customer-support platform, run 2026-08-29, claim surface of 58 claims from README.md, llms.txt, HELP_CENTER_INFRA.md and backend/app/knowledge/README.md; the report states 56 of 58 resolved, 48 of 56 supported.

**Review result:** ADMISSIBLE. Supported by their citations: 51 of 52 findings. Exceptions by disposition: 1 [broken citation]. Result: **FAIL** — a single [broken citation] fails the report under §9, and [broken citation] is a file-operation-settled fact that is not retested, so the fail stands with no retest tally to report.

**Exception that would change a reader's understanding:**

- **F12 (report:55-56) — [broken citation].** "Masked columns are unreadable, including via WHERE and smuggling — [real]" cites tools/guardrailed_db_toolkit.py:355-369 for the mask_rows re-application on return. The file is 280 lines; nothing exists at 355-369. The re-application does exist at line 273 of that file, and the masking enforcement at sql_guardrails.py:287-334 resolves, so the finding's substance looks sound — but as written it is not defensible from what it cites. This review does not repair citations.

Everything else held. 51 findings checked against their cited lines and claim-source text, each supported as stated. Notable, not exceptions: the citation index's 20 `resolved: false` marks were wrong, not the report — the index's file line counts are stale (README.md 150 vs 625 actual; api/widget.py 95 vs 599; services/jira.py 32 vs 339; services/ticket.py 390 vs 1045) — and every affected reference was verified against the real tree and resolves as a line number. models/shopify/shopify_shop.py:31 is `access_token = Column(String, nullable=True)` in the real tree; the index's text for that line ('is_installed') is an index artifact. No `evidence fields pointing nowhere` (conformance.json: 58 fields, zero empty).

**Review completeness:** 52 of 52 findings checked out of the frozen surface (F1–F52, report:6-176). No addendum.

**Coverage (§7):** the working record shows all four claim sources read in full early, followed by targeted structural reads of the SQL connector, messaging pipeline, ticketing autonomy, workers, integrations and knowledge modules. I found no document the report claims coverage of that the record does not show being read. No coverage exception.

*report review · checks findings against cited evidence · not a re-audit of the target*
=== END SUMMARY ===
