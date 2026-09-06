# METHOD — superseded rules and rationale

Moved out of `workflowsv2/claims_audit/method/METHOD.md` §19 on 2026-09-05. Historical: each entry records a rule the method once had and why it changed. Nothing here is an instruction.

Kept separate so obsolete vocabulary does not sit near the active instructions.

### The deliverable was five text blocks

The audit emitted `CLAIM SURFACE`, `REPORT`, `COVERAGE`, `LIMITATIONS` and `GAP MAP` as marker-delimited prose. Markers replaced conversational turn boundaries, which did not reliably indicate what had been completed; a schema replaces the markers for the same reason and more strongly, because a marker can be named in a status line without the block being written, and a schema field cannot.

### The report carried a conclusion, a Gap Map and a limitations statement

All three judged the offering as a whole. A run that sees one claim source cannot make that judgement, and a per-run version of it is not merely redundant but false once runs are combined — a limitations statement naming the documents a run did not use is wrong the moment its findings sit beside theirs. They moved to the stage that has every run in front of it.

### Claims were enumerated into a frozen surface, and coverage divided by it

The surface was the denominator for coverage figures. It was also self-declared: nothing validated that a run's own count was the number of claims the document contains, so a run that identified twenty and adjudicated five could not be told from one that identified five. Bounding each run to one claim source, and estimating the surface from agreement across runs, replaces an unvalidatable denominator with a measurable one.

### Claims were assigned one of four priority tiers

The tiers ordered the work so the highest-consequence claims were settled first, and ordered the findings in the report. Neither job remains: every claim is attempted, so there is nothing to triage, and the report is assembled elsewhere. Materiality — whether a gap would change a buyer's decision — moved to the stage that can weigh it, because it depends on the transaction and on what every other run found.

### The finding was defined ontologically

§4 once read: "A finding is the statement of the verdict of adjudicating a set of cited evidence with respect to a claim." That is the formulation the ontology was settled on, and it is why a finding contains its evidence rather than pointing at it. It was replaced in the executable text by four nested definitions, because it introduces `finding`, `verdict` and `adjudication` at once and a model has to unpick the recursion before it can act.

### There were two caveat verdicts, and a claim could hold with one

`real_minor_caveat` and `real_operational_caveat` both meant "the claim holds,
however —". Across three runs they took 18 of 64 findings, and **five of the
nine adverse findings two stronger models independently produced on the same
corpus were filed under `real_operational_caveat`**, recorded as claims that
hold. Minimal operational overhead and horizontal scaling both went that way.

The cause was structural rather than a model failing. The verdicts encode one
dimension — how the claim fared against the evidence — and a model that is
confident but not certain has nowhere to put that, so uncertainty drained into
the slot that sounded like it. A framework version inferred from directory
names was recorded as holding with a caveat when §8 makes it `unverifiable`.

One caveat verdict remains because one case needs it and nothing else fits:
a claim every part of which is true, where the evidence shows something a
reader must know. Blended MRR of $40,000 that is 60% manual wire transfers
from three customers is the case, and it is a finding three earlier runs
missed entirely. `partial` would be wrong — no part of the assertion fails —
and `real` would be misleading. What changed is that the boundary is now a
test rather than a magnitude: can you name a part of the assertion that fails?

### The v2 wordings lowered the evidential bar, twice

Rewriting §6 for v2 replaced two v1 phrasings that each demanded positive
evidence with ones satisfied by less:

    real     "without a reportable caveat"  ->  "nothing a reader needs"
    partial  "specific, citable material gap" -> "a part is not borne out"

Both were unintended. `reportable` is a judgement about what is worth
reporting; `needs` is met by almost any fact, which invites a model out of
`real` — GLM used `real` zero times in seventeen findings. And "not borne out"
is satisfied by silence, which makes an unsettled claim recordable as adverse:
the error §6 warns about one row above, written into the row itself.

v1's wordings are restored, less `material`, which moved downstream with the
materiality judgement and was not the word doing the work.

### `delta` was the word for a contradicted claim

`delta` is engineering vocabulary for a difference. It is not the register the
rest of this method borrows — assurance work says misstatement, exception or
deviation — and it names only that something differs, which is equally true of
`partial` and of a caveat. `contradicted` says the direction, which is the
thing that separates it from the others.

### The verdict vocabulary was bracketed

Verdicts were written `[real]`, `[partial]`, `[delta]` because they had to be parsed out of prose. They are now field values.

### `[unclaimed]` was once named for what it was not

The category for material the seller did not claim was previously named as the negation of a contradicted claim, which read as "this claim is not false" rather than "no claim was made".

### The conclusion vocabulary once contained buyer actions

Report conclusions once included instructions to the buyer. They were removed because the audit reports the state of the evidence; the vocabulary itself has since moved downstream with the conclusion.
