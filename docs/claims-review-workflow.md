# The claims review, top down

The vocabulary of a claims review as the site and the runners use it, one
level at a time, each term defined where it first appears. Written
2026-09-05 against the code in `workflowsv2/` and `src/client_ui/`, and
brought up to date on 2026-09-18 for the stages added since: the seller's
materials page, the sorting of the materials, the pass that marks repeated
claims, the reliance statement, the tiers, and hand-back. The architecture
of the runners is in `workflowsv2-architecture.md`.

## The engagement and its stages

**Engagement**: one client, one target, one directory under
`workflowsv2/claims_audit/engagements/` holding everything about it. It
moves through **stages**, the ten marks the site records in its `state.json`:
created, letter accepted, intake finished, materials ready, materials
sorted, claims enumerated, surface frozen, review run, report released,
closed. Each stage
is a mark with who made it and when. Inside the stages that do work:

**Intake** is one **conversation**: a chat between the client and the
**intake agent**, a model with `workflowsv2/intake/method/INTAKE.md` as its
instructions and no repository tools. Each exchange is a **turn**, one
message from the client and one reply. After every turn a separate model
call rewrites the **form**, sixteen fields in five **slots** (identify,
situation, background, assessment, recommendation), from the whole
conversation so far. **Finish** writes the transaction and the buyer's
thresholds out of the form into the engagement; the agent never finishes
it, a person does, from the page.

**Materials** are the **target**, the repository the review reads as
evidence, plus the **claim sources**, the documents in which the seller
says what the software does, named by path inside the target. **Evidence
excludes** are the paths, claim sources and docs directories, that the
review may list but not read as text or cite (METHOD §7).

The **seller** is a second role on the site, by email in the engagement's
`seller_emails`. The seller sees one page, the **materials page**
(`src/client_ui/materials.py`): the files under the target, which the seller
uploads, arranges and deletes until the practice marks the materials
**ready**. The seller sees nothing of the client's pages, and the client
never sees the materials.

**Sorting** (`workflowsv2/materials_sorting/runner.py`,
`method/SORTING.md`, procedure in `claim-source-selection.md`) proposes the
claim sources and the evidence excludes. Code lists every prose file in the
target, the files inside archives, and what the prose links to; one model
call per file says what kind it is, by reading it and never by its name; the
two lists follow from the kinds by rule. A person **confirms** or changes
them on the sorting page, and only then is `engagement.yaml` written.
Enumeration is refused until the sorting is confirmed. The record is
`sorting/selection.json` and `SELECTION.md`, outside the target.

## Enumeration and the surface

**Enumeration** is a **run** of the audit program that stops after its
first phase. A **run** is one execution of one stage's program, leaving a
**run directory** under the engagement's `runs/` with its outputs and its
working record. Enumeration reads the claim source one **section** at a
time and emits **claims**: a claim is one assertion, with its verbatim
**quote**, its **lines**, a **statement** in plain words, and **about**,
whom it concerns: the target, the seller, or a document (METHOD §5). The
list of all claims is the **surface**.

The **surface page** holds the surface as a **draft** the practice edits;
the client reads it and comments. **Decompose** asks the agent to propose
testable properties of one broad claim (`method/DECOMPOSE.md`); each
accepted proposal becomes a **subclaim**, an ordinary claim marked
`implied_by` its parent, with the parent's quote and lines and the
practice's statement. Enumeration ends with the **behaviour split**
(`method/BEHAVIOUR_SPLIT.md`): a claim that says what the software does when
it runs becomes a mechanism claim, which code can settle, and a behaviour
claim, which only records of the software running can settle. The parent's
statement becomes the mechanism reading and the behaviour claim is appended
as a subclaim `implied_by` it, marked `approved_by` the pass rather than a
person. The splits are accepted unless the practice drops them on the surface
page; `--no-behaviour-split` on the audit runner turns the pass off.

Three passes follow, over every claim source at once, as further steps of
the **enumerate job**. Each writes marks onto the claims and changes nothing
else, and the surface page shows the marks.

**Repeats** (`claims_audit/duplicates.py`, `method/DUPLICATES.md`). Each
claim source is enumerated alone, so a claim made in two documents is listed
twice. A claim that repeats an earlier one is marked `same_as` it and starts
left out on the surface page; the practice may keep it. A claim narrower than
another is marked `within` the wider one and stays in: it is tested like any
other, and the report names the wider claim beside it as *covered by*. Only
pairs whose statements are close by embedding are shown to the model; the
model decides whether they are the same assertion. The record is
`surface/duplicates.json`.

**The reliance statement** (`claims_audit/reliance.py`,
`method/RELIANCE.md`). One model call reads the whole intake form and every
claim's statement and writes what this buyer relies on the offering for: a
paragraph on the buyer's **use**, and ten to thirty **items**, each a
function, property or commitment of the offering, marked `depends`, `uses`
or `does_not_use`, with what the buyer would have to do if it failed, and
whether that rests on the buyer's words or the practice's inference. It
exists because the buyer's thresholds are in business language and the
claims in technical language. It is shown above the claims on the surface
page, to the client and the practice; the practice corrects it there. The
record is `surface/reliance.json`.

**Tiers** (`claims_audit/tiers.py`, `method/TIERS.md`). Each claim is rated
by whether any way it could be untrue would matter to this buyer, against the
transaction, the thresholds and the reliance statement, fifteen claims to a
call, with no tools and no evidence. **Tier 1**: some way the claim could be
untrue would make the buyer go back to the seller to reopen the price or the
terms over this claim alone, or not close. A claim can reach tier 1 only
through a **route**: a stated threshold that names the kind of failure, or a
reliance item the buyer `depends` on; a failure the buyer's plan survives,
by another way the documents describe, does not count. **Tier 2**: a detail
that a person setting up or operating the software would act on and would
meet in use, whose failure on its own changes nothing for the buyer. **Tier
3**: neither. Each rating carries a **basis**, whose last sentence says
whether it rests on a threshold the buyer stated, on the buyer's plan as the
buyer stated it, on the practice's reading of the buyer's plan, or on the
scale alone. The tier and the basis are marks
on the claim; the practice changes either on the surface page, and a tier a
person set is left alone by a later rating. Tier 1 is tested. Tiers 2 and 3
are **listed, not tested**: they stay on the surface and in the report. A
claim with no tier is tested. After correcting the reliance statement the
practice rates again by running the tiers command; there is no button for
it.

**Freeze** writes the surface in its final form to
`surface/<source>.surface.json`, tiers included; after that nothing is added
to it. The confirmation says how many claims will be tested and how many
only listed.

## The chain

**Run the review** on the practice page starts a **job**: a background
process the site supervises (`src/client_ui/jobs.py`), with a log under the
engagement's `jobs/` and a lock so one engagement runs one job at a time.
The chain job has four **steps**, one program each, in order:

1. **Audit** (`workflowsv2/claims_audit/runner.py`). Takes the frozen
   surface, so its own enumeration is skipped. Claims in tier 2 or 3 go
   under `not_tested` in the run's `claims.json` and are not among its
   `claims`, so nothing later in the chain is asked about them; a claim
   source with nothing to test gets no audit and no review, and a chain
   with nothing to test anywhere is refused. The step has two parts:
   **gathering**, in which the auditor reads the target and files evidence
   under the claims it serves, judging nothing; then **adjudication**, in
   which it judges each claim on the evidence filed under it, using no
   tools. Evidence is gathered; findings and their citations are produced
   by adjudication. The **auditor** is the agent for this step: a model in
   a fresh **world**, the per-run directory under `scenarios/` holding its
   memory and traces. Gathering is done in **legs**: a
   leg is one turn of the agent, driven by the program, which says
   "continue" after each. Inside a leg the agent runs an **action loop**:
   up to sixteen **iterations**, each one JSON action, a tool call or a
   final answer. The action that matters here is an **evidence request**,
   one call to the code-reading **subagent**
   (`src/chat/subagents/code_subagent.py`), itself a small loop of
   **primitives**, list, read, grep and cite over the target, ending in an
   answer with the cited lines copied verbatim; one `cite` carries up to
   ten spans. A request that hits the subagent's step cap reports what it
   found and offers a continuation id; the auditor may resume it once,
   filed under the same claims. (The subagent's `map`
   primitive, a repository map with a category per directory, is switched
   off for the audit and the review: on chhoto, audits run with it scored
   worse on review, 49 and 47 findings holding of 58 against 55 and 55,
   with the reviewer shown unchanged.) Every evidence request is
   tagged with the claims it serves; the auditor organizes requests by
   where the evidence lives and files every claim the material bears on,
   so one request commonly serves seven or more claims. A leg ends when the agent **yields**,
   handing the remainder to the next leg, or responds, meaning it is done
   gathering. Between gathering and adjudication, or after a first
   adjudication, come **chase** legs the program forces: one for claims no
   request was filed under, one for files a search named that nobody
   opened; only the claims affected are then adjudicated again.
   **Adjudication**: for each **batch** of about
   ten claims, one constrained model call reads the claims and the evidence
   filed under them and emits one **finding** per claim: a **verdict**
   from five (real, real with caveat, partial, contradicted, unverifiable)
   and the evidence it rests on (METHOD §6, §7). The **output check** then
   verifies every citation mechanically.

2. **Review** (`workflowsv2/audit_review/runner.py`). A different agent, in
   its own world, reads the run and the target, then answers in **parts**:
   **claim checks**, one per claim, on whether the statement is faithful to
   the quote; **finding reviews**, one per finding, four **observations**:
   evidence relevant, evidence supports, verdict calibration, searches
   adequate; and a **record check** on the run as a whole (REVIEW §5, §6,
   §7). A finding **holds** when all four observations are clean. A
   **retest** gives the failed findings, and a sample of held ones, to a
   second reviewer blind, and records agreement (REVIEW §9).
   **Hand-back** (`workflowsv2/claims_audit/handback.py`) is a separate
   command, not a step of the chain job: where the review names lines in
   the audit's own record that a finding rated `real` did not cite, those
   claims are adjudicated again with the location named and the reviewer's
   reading left out, into a copy of the run, which is then reviewed.

3. **Materiality** (`workflowsv2/audit_materiality/runner.py`). Each
   adverse or caveated finding is rated against the buyer's thresholds
   from the intake, and against the reliance statement where there is one:
   not material, material, or decisive for the deal. **Material** has the
   meaning tier 1 has: the buyer would go back to the seller over this
   finding alone; a threshold that reaches a gap does not make it material
   by itself. Each
   rating is made twice by independent model calls, the **samples**; a
   split goes to five samples, and a three-to-two result is **borderline**,
   decided by the practice. The output is the **merged** directory under
   the engagement's `merged/`, the record of this run of the whole
   engagement.

4. **Report** (`workflowsv2/audit_report/runner.py`). Renders the merged
   directory into the report: markdown, HTML and PDF. Its scope table
   counts, per claim source, the claims tested and the claims not tested,
   and a second appendix lists every claim not tested with its tier and
   basis, read from the frozen surfaces.

**Release** is the practice's mark after reading the report; before it the
client cannot open the report page. The **report page** is the
post-delivery **conversation**: a chat agent bound to the merged record and
the target (`workflowsv2/claims_audit/post_session.py`), answering the
client's questions from the record. **Close** marks the end; deletion on
the retention date is still a manual step.

## Where each thing lives

| Thing | Place |
|---|---|
| the engagement, its stages and jobs | `engagements/<name>/`, `state.json` |
| the intake form and its finish | `engagements/<name>/intakes/<id>/` |
| a run's outputs and working record | `engagements/<name>/runs/<stamp>_<world>/` |
| an agent's world while it runs | `scenarios/<world>/` (swept after 30 days) |
| the frozen surface, comments, drafts | `engagements/<name>/surface/` |
| the repeats record, the reliance statement, the tier ratings | `engagements/<name>/surface/duplicates.json`, `reliance.json`, `tiers.json` |
| the sorting record | `engagements/<name>/sorting/` |
| the merged record, ratings and report | `engagements/<name>/merged/<stamp>_<label>/` |
| job logs | `engagements/<name>/jobs/<id>.log` |
| the methods the agents work to | `workflowsv2/*/method/*.md` |
