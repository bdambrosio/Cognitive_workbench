# workflowsv2 — the claims-review programs, as built

Re-derived from the code on 2026-09-19, at commit `ad2a8cd5` (tag `v2.2`).
Every statement here was checked against the file it names. The vocabulary
(engagement, surface, tier, finding, holds, material) is defined in
`claims-review-workflow.md` and is not repeated here. This document says which
program does each step, what it reads and writes, and which controls the code
enforces.

Each step is a runner over a method document and a JSON schema. A method
document is loaded by `chat.workflow.load_workflow`, which drops the sections
marked for the practice, and the result is the system text of the model calls.
Every deliverable is produced by `workflowsv2/emit.py`: one schema-constrained
call with two messages and no conversation history, which records how the
response parsed and whether the route dropped `response_format`. A later step
reads the typed JSON an earlier step wrote.

## The programs

Before the freeze, per engagement:

| step | program | method | reads | writes |
|---|---|---|---|---|
| intake | `intake/runner.py`, `intake/session.py` | `INTAKE.md` | the client's turns | `intakes/<id>/intake.json`, `intake.log`; on finish `blocks.yaml`, `intake_meta.json`, and `brief.md` if there is none |
| sorting | `materials_sorting/runner.py` | `SORTING.md` | every prose file under the target | `sorting/selection.json`, `SELECTION.md`; on confirm, `claim_sources` and `evidence_excludes` in `engagement.yaml` and extracted text under `target/claim_sources/` |
| enumeration | `claims_audit/runner.py --enumerate-only` | `METHOD.md`, `BEHAVIOUR_SPLIT.md` | one claim source | `runs/<ts>_<world>/claims.json` |
| repeats | `claims_audit/duplicates.py` | `DUPLICATES.md` | the latest enumeration of every source | `same_as` and `within` marks in those `claims.json`; `surface/duplicates.json` |
| reliance | `claims_audit/reliance.py` | `RELIANCE.md` | the current intake's whole form (else the engagement's two blocks), every claim statement | `surface/reliance.json` |
| tiers | `claims_audit/tiers.py` | `TIERS.md` | the two blocks, the reliance statement, the claims | `tier` and `tier_basis` on each claim; `surface/tiers.json` |
| decompose | `claims_audit/decompose.py`, from the surface page | `DECOMPOSE.md` | one claim and its section | proposals; accepted ones become draft claims |

After the freeze, the chain:

| step | program | method | reads | writes |
|---|---|---|---|---|
| audit | `claims_audit/runner.py --surface` | `METHOD.md` | the frozen surface of one claim source, `brief.md`, the target | `runs/<ts>_<world>/claims.json`, `findings.json`, `run_meta.json`, `run.log`, `issues.jsonl`, `working_record/` |
| review | `audit_review/runner.py` | `REVIEW.md` | one audit run directory and the target | `<run>/review/statistics.json`, `review.json`, `outcomes.json`, `retest.json`, `adverse_recall.json`, `review_meta.json` |
| materiality | `audit_materiality/runner.py` | `MATERIALITY.md` | the named runs, the intake's blocks, the reliance statement | `merged/<ts>_<label>/merged.json`, `materiality.json`, `materiality.md`, `meta.json`, `issues.jsonl` |
| report | `audit_report/runner.py` | `REPORT.md` | one merged directory, the intake it pinned, the frozen surfaces, `surface/duplicates.json` | `report.md`, `report.html`, `report.pdf`, `report_skeleton.md`, `prose.json`, `report_meta.json`, `worklist.md` |

Beside the chain: `claims_audit/handback.py` (a command, not a job step), and
the post-delivery conversation (`claims_audit/post_session.py`,
`continuation.py`, `record.py`, method `CONTINUATION.md`).

All paths in the tables are under
`workflowsv2/claims_audit/engagements/<name>/`; methods are under each
program's `method/` directory.

## Engagement state

`workflowsv2/engagement_state.py` is the module every runner and the site use,
and a CLI: `<engagement> new [--clone <url or path>]`, `status`, `intake
current|cancel <id>`, `run current|cancel <merged dir>`, `stage <stage>
<value>`.

`new` creates the directory and a stub `engagement.yaml` (`target`,
`claim_sources`, `client_emails`, `seller_emails`, `retention`) and marks the
stage `created`; with `--clone` it fills `target/` and refuses to replace an
entry already there. Nothing else creates an engagement. The target is
`target/` inside the engagement unless `engagement.yaml` names another path.
`evidence_excludes` is the engagement's list when the key is present, else the
claim sources. `update_engagement` rewrites only the named key in place, so
comments in the file survive.

`state.json` holds five things: the explicit current intake, the explicit
current run per intake, the cancelled lists, `stages` and `jobs`. Intakes and
runs are read from disk, never listed in the file. In this module a *run* is a
merged directory that holds a `meta.json`; that file pins the intake the
ratings were read against. The current intake or run is the explicit choice if
it exists and is not cancelled, else the most recent not cancelled. A new
intake clears the explicit choice. An engagement with no intake reads
`transaction` and `thresholds` from `engagement.yaml`.

The ten stages, in order: `created`, `letter`, `intake`, `materials`,
`sorting`, `enumeration`, `surface`, `chain`, `release`, `closed`. Each mark
is `{value, at, by}`. A job record in state `running` is the engagement's
lock: `add_job` refuses a second one. Writes to `state.json` are serialised by
a lock within one process and written by atomic replace; two processes writing
one engagement at once is not handled.

## Intake

`IntakeSession` owns the chat loop and the form; the terminal runner and the
site both call its `turn()`. The intake agent has no repository tools. The
runner sends the opening turn, appends to each client turn a one-line ledger
of the slots still empty, and after each exchange makes one schema-constrained
call that re-emits the whole form (`intake/schemas.py`: five slots) from the
conversation. The form is written to `intake.json` and mirrored into a note in
the world. The world is `client_<engagement>` and is reused across sessions.
An upload lands under the engagement's `target/` and its text is given to the
agent as a turn.

`finish` writes `transaction:` and `thresholds:` from the form to the intake's
`blocks.yaml`. `--finish --conclusion` on the command line also writes
`conclusion: true`. The site's finish button calls `finish` without it, so a
conclusion can be asked for only from the command line.

## Sorting

Code lists the candidates: every prose file in the target (by `git ls-files`,
else a walk), members of zip archives, the files and outside hosts the prose
links to, and the repository's hosting description (the only thing fetched
from outside the target). One model call per file with at least 20 words
assigns a kind (`description`, `instrument`, `product_text`, `neither`).
`propose` derives the two lists by rule: every description is a proposed claim
source and an evidence exclude, plus `claim_sources/`. The stage is marked
`proposed`.

`--confirm --by <email>` (or the sorting page) applies the proposal or the
lists given in its place: claim sources that are HTML, PDF or archive members
are extracted to text under `target/claim_sources/`, the two keys in
`engagement.yaml` are written, the confirmation and its changes are recorded,
and the stage becomes `confirmed`. The enumerate job is refused until then.

## The audit runner

One program, `claims_audit/runner.py`, does enumeration and the audit. It
refuses a world name that already exists under `scenarios/`, replaces the
scenario's `llm_config` with the model file's (never merges), asks the server
what it serves when the model file sets `expects_served_model`, resolves the
temperature before any work, and sets `evidence_excludes` and `subagent_map:
false` in the agent's configuration. Autonomy is off; the runner drives every
leg.

**Enumeration.** `schemas.split_sections` cuts the claim source at markdown
headings outside code fences, with a minimum and a maximum section size. Each
section is one call that sees the claims so far. `schemas.assemble_surface`
assigns ids in document order and folds a claim carrying `restates`, or a
repeat of the same quote and statement within a section, into the earlier
claim's `locations`. Each claim is marked `about`: `target`, `seller` or
`document`. Then `behaviour_split.propose` runs once per section and `apply`
rewrites the parent's statement and appends the behaviour claim (`implied_by`,
`approved_by` the pass); `--no-behaviour-split` skips it, and a section whose
call fails keeps its claims and records an issue. `schemas.check_surface`
checks every quote against the source; problems are recorded as blocking
issues. `--enumerate-only` stops here.

**A frozen surface.** With `--surface` nothing is enumerated or split.
`schemas.split_by_tier` puts claims rated tier 2 or 3 under `not_tested` in
the run's `claims.json`; tier 1 and unrated claims are `claims`. Only `claims`
reach the legs, adjudication and every later step.

**Gathering legs.** The first message is the brief, the instruction to name on
every `inspect_external` request the claims it serves, and the claims. Each
request is one run of the code subagent
(`src/chat/subagents/code_subagent.py`), which writes one trace file whose
query line carries the claim ids. A subagent that reaches its step cap stores
its state under `subagent_continuations/` and returns an id; the request can
be resumed once with `continue_id`. A leg that ends in `yield` is followed by
`continue` plus a line of engagement state; any other ending stops gathering.
A leg cut by the action cap counts as a yield if it filed a new request and
ends the run as invalid if it did not. `--max-turns` (default 25) caps all
legs, chases included; reaching it is recorded as `gathering_capped`, not as
an error.

**Adjudication.** `evidence_batches` walks claims in id order and closes a
batch at `--batch` claims (default 10) or when the full traces of its claims
would exceed `--evidence-budget` (default 400,000 characters). Each call gets
only the traces filed under its claims: full, else trimmed to the cited lines
with a 16-line band, else query and answer only. Findings are written to
`findings.partial.json` batch by batch. A route that dropped
`response_format`, or findings that do not parse, end the run as failed.
Claims that got no finding are asked for once more.

**Three chases.** Before adjudication, claims with no request filed get a
gathering leg naming them, repeated until a pass tags nothing new; claims
still untagged are not adjudicated and are named in `not_completed`. After
adjudication, `schemas.missing_search_kinds` names findings that rest on
searches and record only one of the two kinds (an `unverifiable` finding on a
claim about the seller is exempt); a leg asks for the missing search and those
claims are adjudicated again, two passes at most. Then
`schemas.candidate_files` names files that a finding's searches named and the
run never opened; a leg opens them and those claims are adjudicated again,
until a pass opens nothing new. What remains after each chase is in
`run_meta.chase` and recorded as an issue.

**Output check and record.** `schemas.check_output` resolves every citation
against the target, strips line-number prefixes from quotes, checks the fields
each evidence form requires, the `not_examined` rule against the files the run
read, and records citations into excluded documents. A check problem is a
blocking issue, not a failed run; the exit code is non-zero only when the run
did not complete. The runner copies into `working_record/` the method as
delivered, the first message, the resolved scenario, the reasoning trace and
every subagent trace. `run_meta.json` records the resolved model, temperature
and `top_p`, the target's and the harness's commit, the materials listing,
files read and matched, the legs, the chases, the batches, and the route's
transient errors.

## The passes between enumeration and the freeze

`duplicates.py` embeds every statement (`BAAI/bge-small-en-v1.5`, CPU) and
shows the model, for each claim, up to five earlier claims with similarity of
at least 0.70 (`NEAREST`, `FLOOR`); a claim with none is not sent. The model
decides `same_as` or `within`; the similarity figure decides only what is
shown. A behaviour claim is compared only with behaviour claims. A pairing is
kept only if it points to an earlier source or a smaller id. A source that
already has a draft or a frozen surface gets no new marks.

`reliance.py` makes one call and overwrites `surface/reliance.json`,
corrections included. The surface page saves a person's correction through
`site.save_reliance`, with who corrected it.

`tiers.py` rates fifteen claims per call with a 32,768-token limit. For each
source it reads the frozen surface, else the draft, else the latest
enumeration, and writes `tier` and `tier_basis` into the draft or the
enumeration. It never writes to a frozen surface, skips claims marked
`same_as`, and leaves a claim with `tier_by` alone. A frozen surface is still
rated and the ratings go to `tiers.json` only. A claim the call did not rate
keeps no tier and is therefore tested. If a batch does not parse the program
stops and that source's claims are not marked; `tiers.json` is written after
each source, so sources rated earlier in the same invocation keep their marks
and their record.

## Review

`statistics()` recomputes the mechanical properties of the audit's output from
`claims.json`, `findings.json` and the target; it does not read the audit's
own checks. Reading legs follow (default cap 12), with the same `subagent_map:
false` and the run's `evidence_excludes`. Then schema-constrained calls in
batches of ten: one claim check per claim, one finding review per finding with
four observations (`evidence_relevant`, `evidence_supports`,
`verdict_calibration`, `searches_adequate`) and a required exception when one
is not clean, and one record check. `schemas.derive_outcomes` sets `holds`
when all four observations are clean; the reviewer never writes it. `retest`
gives the findings with adverse observations, and a sample of held ones
(default 3), to a second reviewer in its own world that sees none of the first
review; `--retest-model` defaults to `--model`. The runner refuses a run whose
`review/review.json` exists.

`adverse_recall` is a separate check in the same program: for findings rated
`real` it reads the full traces filed under the claim and names lines adverse
to the claim that the finding does not cite. It writes `adverse_recall.json`
and issues and does not change `holds`. `--adverse-recall-only` runs it on an
existing run. `handback.py` takes those rows, adjudicates exactly those claims
again with a note that names the location and not the reviewer's reading, and
writes a copy of the run, `<run>_handback_<ts>`, without `review/`; the review
runner is then run on the copy. No job calls it.

## Materiality

`merge.py` concatenates the named runs' findings with their review outcomes
and citation problems. It removes nothing; identical quotes across sources are
reported as an issue. Findings with verdict `contradicted`, `partial` or
`real_with_caveat` are rated for `materiality` (`not_material`, `material`,
`decisive`); `unverifiable` findings are rated for `exposure`; `real` findings
are not rated. Each call gets the transaction, the thresholds, and the
rendered reliance statement where `surface/reliance.json` exists. Every rating
is made twice in independently shuffled batches (default 20); a finding rated
differently gets three more samples and `schemas.combine` ships the majority
with its count, or the plurality marked borderline with an issue. `--single`
rates once. `meta.json` pins the intake id and the SHA-256 of the thresholds,
the transaction and the reliance statement. A rated finding the review did not
uphold is recorded as issue `rated_but_not_upheld`.

## Report

`render.py` assembles the document from the record and re-judges nothing:
transaction, executive summary, scope and approach (a table of tested and
not-tested counts per claim source, including a source with no run), what the
review showed by materiality, unsettled claims by exposure, unsettled claims
about the seller, claims not examined, claims that hold, questions for the
seller, observations the seller did not claim, coverage, limitations, an
appendix of every tested claim with its verdict, and a second appendix of
claims listed and not tested. `not_tested` reads the engagement's frozen
surfaces and `covered_by` reads `surface/duplicates.json`, both by path from
the merged directory; a claim marked `within` is reported with the wider claim
beside it and its verdict is never derived.

One schema-constrained call writes seven passages (`summary`, `conclusion`,
`scope_note`, `shown_note`, `unsettled_note`, `not_examined_note`,
`limitations`). The report reads the intake that `meta.json` pinned. The
conclusion is placed only when that intake's blocks carry `conclusion: true`
and thresholds exist; `schemas.check_prose` records a conclusion written when
none was asked, an empty required passage, and a claim id no finding carries.
The runner refuses a merged directory that already has `prose.json`;
`--rerender` rebuilds the document from it with no model call, and
`--no-prose` assembles without a model. `printable.py` writes the HTML and the
PDF. `worklist.md` gathers every step's `issues.jsonl`.

## The client site

`src/client_ui/site.py` is one FastAPI process for every engagement (default
port 8803). `access.py` verifies the Cloudflare Access JWT from the header or
cookie and gives a role: practice (`PRACTICE_EMAILS`), client
(`client_emails`), seller (`seller_emails`). `--no-access` takes the identity
from `?as=` for loopback use and tests. `cf_access.py` adds new client emails
to the Access policy and never removes one; unconfigured it does nothing.

Client pages are under `/e/<engagement>/`: home with the stages and
`next_step`, letter acceptance, the intake chat with upload and finish, the
surface with comments, and the report chat. The seller has only
`/e/<engagement>/materials/` (`materials.py`): upload, mkdir, delete under the
target, and for the practice marking a file as a claim source or an evidence
exclude; writes are refused while a job runs and, for the seller, after the
materials are marked ready; paths that leave the target root are refused.
Practice pages are under `/p/`: engagements and settings, current and cancel
for intakes and runs, the three stage buttons (`materials` ready, `release`,
`closed`; release is refused without a `report.md` in the current run), the
job buttons and logs, the sorting page with confirm, the scrub guidance
(`SCRUB.md`), and the surface page: draft save, reliance correction,
decompose, freeze and unfreeze. `freeze` writes `surface/<slug>.surface.json`
from the draft or the enumeration and marks the stage when every source is
frozen. `unfreeze` archives the frozen file, makes it the draft, and marks a
finished chain `superseded`; it is refused while a job runs.

`jobs.py` runs three job kinds as subprocesses in a thread, logging to
`jobs/<id>.log`: `sort`; `enumerate` (one `--enumerate-only` run per claim
source, then duplicates on the low-reasoning model file, reliance, tiers);
`chain` (per source an audit on the frozen surface and its review, then
materiality over those runs with the current intake, then the report). The
chain is refused if any source has no frozen surface or no claim would be
tested, and it skips a source with nothing to test. A step that exits non-zero
ends the job as failed. `mail.py` sends the practice a notice when a job ends
and the client a notice on release; without `SMTP_PASS`, or with
`MAIL_DRY_RUN`, it logs the notice and sends nothing.

`registry.py` keeps one live session per (kind, engagement) with a worker
thread each, evicts the least recently used, and limits turns in flight.

`src/client_ui/app.py` (one client, a token on the URL, ports 8800 and 8801)
holds the earlier single-session client pages. They still run, and `site.py`
imports two helpers from it. The single-session practice page, `practice.py`,
was deleted on 2026-09-19; its static files are the ones `/p/` serves.

## Post-delivery conversation and the demo

`PostSession` binds `inspect_external` to the target, `inspect` to the
engagement directory, and `CONTINUATION.md` as the system text, over the
current run of the current intake. Its world is `post_<…>` per (intake, run)
and is resumed. `record.py` registers a `claim` action on that loop only: one
call returns a claim's finding, review outcome, rating, report lines and
seller questions, with each citation checked against the target at call time.

`src/demo/app.py` (port 8810) serves one delivered run to many visitors: a
cookie per visitor, a world `demo_<engagement>_<run>_<sid>`, a cap on live
sessions, turns in flight, turns per visitor and sessions per address, and its
own sweep of demo worlds (`keep_days`, default 7). `redact.py` substitutes the
identifiers listed in the engagement's `demo.yaml` in every string sent to a
browser. The public site is the static folder `site/`.

## Controls the code enforces

- **Sampling.** `src/chat/model_params.py`: `TOP_P` 0.95 for every call;
  temperature per model; a model with no entry raises. The audit runner logs
  and records the resolved values. It also accepts `--temperature`, which
  overrides the per-model value for action emission; the jobs never pass it.
- **Schema-constrained deliverables.** A route that drops `response_format`
  fails the audit and the report; `emit` records parse state for every call.
- **A fresh world per audit run**, refused otherwise. Review, materiality and
  report name their worlds from a timestamp or the run. Intake and
  post-delivery worlds persist by design.
- **One review per run; one report per merged directory** unless `--rerender`.
- **Issues.** `issues.py` appends to `issues.jsonl` in the run or merged
  directory at the moment a step notices something a person must decide.
- **Route reliability.** `transient_events` (retried statuses by code, calls
  that spent the retry budget) is in each step's metadata.
- **Method lint.** `lint_workflow.py` checks METHOD, REVIEW, MATERIALITY,
  REPORT and INTAKE for retired tokens, dates, section references, counts and
  vocabulary against the schemas. It checks form only. The other method
  documents are not in its list.
- **World sweep.** `sweep_worlds.py` lists, and with `--delete` removes,
  worlds under `scenarios/` older than `--days` (default 30), keeping names
  that start `jill_`, `post_`, `client_` (the intake worlds), `intake_` or
  `demo_`. It never touches run or merged directories.

## Routes

`jobs.MODEL` is `measure/models/fw_glm53flash.yaml` (GLM-5.3-Flash on
Fireworks) for every job step except the repeats pass, which uses
`fw_glm53flash_low.yaml`. `measure/models/local_qwen38flashnext.yaml` is the
local route and the one the demo is written for. Every program takes
`--model`; there is no per-step model choice inside a job beyond the repeats
pass. Provenance of temperatures: `docs/model-settings.md`; route choice:
`docs/model-prescreen.md`.

## Not built, or not connected

- The report does not derive a `within` claim's verdict from the wider
  claim's; both are audited.
- No button rates the tiers again after the reliance statement is corrected;
  the practice runs `tiers.py`.
- Hand-back is not a job step.
- `retention:` in `engagement.yaml` is read and shown; no program acts on it,
  and none deletes an engagement.
- `workflowsv2/security_audit/` is a separate runner with its own method and
  report; no job and no page reaches it.
- `workflowsv2/blocks.py` is used by the security audit runner, the linter
  and a test; `coverage.py` is imported only by a test, and `citations.py`
  only by `coverage.py`. No program in the chain uses any of the three.
  `workflowsv2/audit_postprocess/` holds no source.
