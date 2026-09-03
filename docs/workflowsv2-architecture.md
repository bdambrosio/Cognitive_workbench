# workflowsv2 — the claims-audit workflow, as built

Re-derived from the code on 2026-09-02. Four stages, each a runner over a
method document and a schema, each writing typed JSON that the next stage
reads. Nothing downstream re-judges what a stage upstream decided; every
figure is arithmetic over typed fields.

| stage | directory | method | reads | writes |
|---|---|---|---|---|
| audit | `workflowsv2/claims_audit/` | `method/METHOD.md` | an engagement (`engagements/<name>/engagement.yaml`, `brief.md`) and its target tree | `runs/<ts>_<world>/claims.json`, `findings.json`, `run_meta.json`, `issues.jsonl`, `working_record/` |
| review | `workflowsv2/audit_review/` | `method/REVIEW.md` | one run directory and the target | `runs/<run>/review/review.json`, `outcomes.json`, `retest.json`, `statistics.json`, `issues.jsonl` |
| materiality | `workflowsv2/audit_materiality/` | `method/MATERIALITY.md` | one or more reviewed runs and the current intake's `transaction:` and `thresholds:` (else engagement.yaml's) | `merged/<ts>_<label>/merged.json`, `materiality.json`, `materiality.md`, `meta.json` (pins the intake), `issues.jsonl` |
| report | `workflowsv2/audit_report/` | `method/REPORT.md` | one merged directory and the intake it pinned | `report.md`, `report.html`, `report.pdf`, `report_skeleton.md`, `prose.json`, `worklist.md` |

Two runners sit either side of the chain. `intake/runner.py` with
`method/INTAKE.md` is the client conversation that fills the ISBAR form and,
on `--finish`, writes the blocks a run reads. `claims_audit/continuation.py`
with `method/CONTINUATION.md` is the post-delivery conversation: it answers
questions about a finished run from its record and is interactive.

## Engagement state (2026-09-03)

The engagement comes first, explicitly: `engagement_state.py <name> new
[--clone <url or path>]` creates the directory, a stub engagement.yaml and,
with `--clone`, the materials at `target/` inside it (the default `target:`;
a local clone hardlinks objects and takes seconds). Nothing else creates an
engagement, and the intake refuses a name it does not find. The practice
fills `claim_sources:` after the intake has said where the claims are.

An engagement directory holds intakes under `intakes/<id>/` (the form,
`intake_meta.json`, `intake.log`, and after `--finish` a `blocks.yaml` with
`transaction:` and `thresholds:`) and runs under `merged/<ts>_<label>/`, each
of which pins in its `meta.json` the intake its ratings were read against.
One intake is *current*, and one run per intake: an explicit choice if one
was made and not cancelled, else the most recent not cancelled. `state.json`
holds only the explicit choices and the cancelled marks; nothing is deleted.
`workflowsv2/engagement_state.py` is the module and the CLI (`status`,
`intake current|cancel <id>`, `run current|cancel <name>`); every runner
reads the current intake and run through it, and `--intake` / `--merged`
override. An engagement with no intake reads its blocks from engagement.yaml,
which is how the fixtures state them.

The browser page (`src/client_ui/app.py`, `intake` and `post` subcommands)
drives the same two conversations through `workflowsv2/intake/session.py`
and `workflowsv2/claims_audit/post_session.py`, the objects the terminal
runners loop over, from one worker thread per process: chat on the left, the
filling form or the delivered report on the right, a token on the URL, an
upload that lands in `engagements/<e>/uploads/` and is told to the agent as
a turn. The page initiates nothing: `--new`, `--finish` and the current
choices stay on the command line.

Worlds: the intake runs in `client_<engagement>`, reused across sessions. The
post-delivery conversation runs in one world per (intake, run), named from
their timestamps and reused, so a second session remembers the first and a
different run or intake starts clean. Audit, review, materiality and report
each run in a fresh world.

## The audit stage

**Enumeration by section.** The claim source is cut at markdown headings
outside code fences (`schemas.split_sections`); each section is one
schema-constrained call that sees the claims enumerated so far with their ids.
The runner assigns ids in document order, folds a `restates` reference and any
identical quote within a section into the earlier claim's `locations`, and
marks each claim `about: target | seller`. METHOD §2 says what is not a claim.
The surface is frozen when the last section is in. `--enumerate-only` stops
here.

**Gathering legs.** The agent works the target through the `inspect_external`
subagent, naming on each request the claims it serves (`claims: [ids]`, written
into the trace's query line). Legs end with `yield` and are continued by the
runner; `respond` ends gathering. A leg cut by the action cap is a boundary.

**Adjudication in batches.** `evidence_batches` walks claims in id order and
fills a batch until ten claims or until the union of its claims' full traces
would exceed `--evidence-budget` (400k chars). Each call is handed only the
traces filed under its claims: full if they fit, else trimmed to cited lines
with a 16-line band (`trim_trace`), else query-and-answer (`compact_trace`).
Every claim gets one finding under `schemas.audit_schema`; `check_output`
resolves every citation against the target.

**Two chases.** Before adjudication, claims no request was filed under get one
targeted gathering leg each pass until every claim is named. After
adjudication, `unverifiable` findings whose searches named a file the run
never opened (`not_examined`) get a leg to open it and are re-adjudicated.
Both loops stop when a pass changes nothing or the leg cap is spent, and what
is left is recorded in `run_meta.chase` and as issues.

## The review stage

Mechanical statistics first (`statistics()`), then reading legs, then
schema-constrained calls in batches of ten: claim fidelity per claim, four
observations per finding with a required exception when any is not clean,
and one record check. Outcomes are derived, never written by the reviewer. A
blind second reviewer retests adverse observations and a sample of findings
that hold; the sample is a calibration figure only.

## The materiality stage

`merge.py` concatenates per-document runs with their review outcomes and
citation problems, no deduplication (the surface is human-owned). Findings
with a verdict about the claim are rated for `materiality`; `unverifiable`
findings for `exposure`; never in one total. Rated against the engagement's
`transaction:` block.

## The report stage

`render.py` assembles the client document from the record: the transaction,
what the audit showed by materiality, unsettled claims by exposure, claims not
examined, claims that hold, questions, observations, computed coverage, an
appendix of every claim with its verdict. One schema-constrained call writes
six passages (REPORT.md §6). `--no-prose` assembles without a model.
`worklist.md` gathers every stage's `issues.jsonl`.

## Controls that hold across stages

- **Sampling settings are code.** Temperature per model in
  `src/chat/model_params.py`; a model without an entry raises.
- **Every route records its reliability.** `transient_events` in every
  stage's metadata: transient statuses retried, by code, and calls that
  spent the retry budget.
- **Method documents are linted** (`lint_workflow.py`): retired tokens, dates
  in the prompt, section references, and each document's vocabulary against
  its schema.
- **A fresh world per audit run.** The conversational runners (intake,
  post-delivery) persist their worlds by design; the four chain stages never
  reuse one.

## Routes

Local: Qwen3.8-Flash-Next on the box (`measure/models/local_qwen38flashnext.yaml`)
for debugging. Hosted: GLM-5.3-Flash on Fireworks
(`measure/models/fw_glm53flash.yaml`), the default hosted route from
2026-09-02; direct hosts before brokers (`docs/model-prescreen.md`).

## Not built

An overall conclusion or recommendation (the thresholds now exist; the stage
does not); the browser UI for the intake and post-delivery conversations;
multi-document merge on a real engagement; the security audit (held); a
per-stage model choice so enumeration can run hosted while the rest runs
local. The fixture (`measure/fixtures/dataroom/`) is scored by reading
`findings.json` against `answer_key.md`.
