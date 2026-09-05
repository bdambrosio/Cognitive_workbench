# The claims review, top down

The vocabulary of a claims review as the site and the runners use it, one
level at a time, each term defined where it first appears. Written
2026-09-05 against the code in `workflowsv2/` and `src/client_ui/`; the
architecture of the runners is in `workflowsv2-architecture.md`.

## The engagement and its stages

**Engagement**: one client, one target, one directory under
`workflowsv2/claims_audit/engagements/` holding everything about it. It
moves through **stages**, the ten marks the site records in its `state.json`:
created, letter accepted, intake finished, materials ready, claims
enumerated, surface frozen, review run, report released, closed. Each stage
is a mark with who made it and when. Inside the stages that do work:

**Intake** is one **conversation**: a chat between the client and the
**intake agent**, a model with `workflowsv2/intake/method/INTAKE.md` as its
instructions and no repository tools. Each exchange is a **turn**, one
message from the client and one reply. After every turn a separate model
call rewrites the **form**, fifteen fields in five **slots** (identify,
situation, background, assessment, recommendation), from the whole
conversation so far. **Finish** writes the transaction and the buyer's
thresholds out of the form into the engagement; the agent never finishes
it, a person does, from the page.

**Materials** are the **target**, the repository the review reads as
evidence, plus the **claim sources**, the documents in which the seller
says what the software does, named by path inside the target. **Evidence
excludes** are the paths, claim sources and docs directories, that the
review may list but not read as text or cite (METHOD §7).

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
practice's statement. **Freeze** writes the surface in its final form to
`surface/<source>.surface.json`; after that nothing is added to it.

## The chain

**Run the review** on the practice page starts a **job**: a background
process the site supervises (`src/client_ui/jobs.py`), with a log under the
engagement's `jobs/` and a lock so one engagement runs one job at a time.
The chain job has four **steps**, one program each, in order:

1. **Audit** (`workflowsv2/claims_audit/runner.py`). Takes the frozen
   surface, so its own enumeration is skipped. The **auditor** is the agent
   for this step: a model in a fresh **world**, the per-run directory under
   `scenarios/` holding its memory and traces. Its work is in **legs**: a
   leg is one turn of the agent, driven by the program, which says
   "continue" after each. Inside a leg the agent runs an **action loop**:
   up to sixteen **iterations**, each one JSON action, a tool call or a
   final answer. The action that matters here is an **evidence request**,
   one call to the code-reading **subagent**
   (`src/chat/subagents/code_subagent.py`), itself a small loop of
   **primitives**, list, read, grep and cite over the target, ending in an
   answer with the cited lines copied verbatim. Every evidence request is
   tagged with the claims it serves. A leg ends when the agent **yields**,
   handing the remainder to the next leg, or responds, meaning it is done
   gathering. After gathering come **chase** legs the program forces: one
   for claims no request was filed under, one for files a search named
   that nobody opened. Then **adjudication**: for each **batch** of about
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

3. **Materiality** (`workflowsv2/audit_materiality/runner.py`). Each
   adverse or caveated finding is rated against the buyer's thresholds
   from the intake: not material, material, or decisive for the deal. Each
   rating is made twice by independent model calls, the **samples**; a
   split goes to five samples, and a three-to-two result is **borderline**,
   decided by the practice. The output is the **merged** directory under
   the engagement's `merged/`, the record of this run of the whole
   engagement.

4. **Report** (`workflowsv2/audit_report/runner.py`). Renders the merged
   directory into the report: markdown, HTML and PDF.

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
| the merged record, ratings and report | `engagements/<name>/merged/<stamp>_<label>/` |
| job logs | `engagements/<name>/jobs/<id>.log` |
| the methods the agents work to | `workflowsv2/*/method/*.md` |
