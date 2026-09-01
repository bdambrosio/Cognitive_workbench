# Editor notes — 2026-08-30T19-34-17Z_cm_plain_glm_med_1

Produced by workflows/audit_postprocess/deliver.py. Every item below needs a human decision; none of them is a defect this script is entitled to settle.

## 1. Citations that did not resolve

3 of 94 citations did not resolve. METHOD §12 step 6b says a finding with an invalid citation does not ship — but the index resolves only `docN:NN` references and quoted spans, and it can resolve a name to the wrong file, so **check each against the materials before acting**. This script does not repair citations.

- `ticket_investigator.py:628-635` — ticket_investigator.py has 398 lines (not mentioned by any review)
- `ticket.py:755-767` — ticket.py has 287 lines (not mentioned by any review)
- `package.json:17-18` — package.json has 12 lines (not mentioned by any review)

## 1b. What the independent review said

Admissible, no exceptions raised. Note that a review finding nothing is the least discriminating outcome available, not a guarantee.

## 1c. Observations parked in an appendix

The auditor wrote these into the claim surface rather than as findings, so they sit at the end of the last appendix where a client will not reach them. Decide whether each belongs in the body — this script will not promote an observation into a finding.

- One internal tension already visible from the sources themselves (to be tested, not yet a finding): knowledge README's usage example states max_depth default 3 and max_links 10, while its Configuration Options section states defaults 5 and 25 (claims 45).

## 2. Consequence ordering

The audit orders findings by verdict class, which is not the same as consequence to this buyer — and consequence to this buyer is not derivable from the materials, so neither the audit nor this script can rank it. Re-order the findings below if the client's priorities differ from the order the verdicts imply.

- Finding 1 — [delta] — Cross-session conversation memory
- Finding 2 — [delta] — Auto translation
- Finding 3 — [delta] — ChatterMate CLI
- Finding 4 — [partial] — Workflow version control
- Finding 5 — [partial] — Knowledge worker stuck-job recovery
- Finding 6 — [real, minor caveat] — Shopify integration
- Finding 7 — [real, minor caveat] — Human handoff context
- Finding 8 — [real, minor caveat] — MCP investigation sources
- Finding 9 — [real, minor caveat] — File attachments
- Finding 10 — [real, minor caveat] — Jira integration
- Finding 11 — [real, minor caveat] — Knowledge defaults
- Finding 12 — [real, minor caveat] — Knowledge-module tests

## 3. Claim numbers

The claim surface is appended to the deliverable, so every claim number in the coverage statement and the supported appendix resolves. Nothing to do.

The supported appendix holds 36 entries, keyed by claim number rather than finding number.

## 4. Structure

- report as delivered: 16,764 characters
- narrative findings: 12
- appendix entries: 36
- gap map present: True (delivered separately)
- blocks closed: {'REPORT': True, 'LIMITATIONS': True, 'GAP MAP': True}
