# Tuuyi Claims Review: Technical Report for Prospective Acquirers

Prepared by Tuuyi, September 2026. This report describes the claims-review product as it exists in the repository at the date of writing. It is written for a technical diligence team. Where a figure is quoted, it comes from our own run records.

## 1. What the product does

Claims Review takes the documents a software seller hands a buyer, the README, the technical overview, the sales one-pager, and checks every factual assertion in them against the seller's source code. The output is a finding per claim, with a verdict, the evidence the verdict rests on, and a citation into the code for every piece of evidence. An independent second pass reviews each finding against its own evidence and records whether it holds. The buyer receives a report that reads against thresholds the buyer set at intake: what they are paying for, what would change the price, what would end the deal.

The product is a pipeline of stages, each a separate process with its own method document and its own model configuration. The stages, in order, are enumeration of the claim surface, the audit, the review, hand-back, materiality, and the report. All of them live under `workflowsv2/` in the repository.

## 2. Architecture

Each stage is a runner script that drives a ReAct agent through a written method. The method is a markdown document the agent receives as its instructions. The runner enforces everything the method cannot: output shape, closed vocabularies, mechanical checks. The split is deliberate. A method document says what a finding is and when a verdict is right. The runner says what a well-formed output looks like and refuses one that is not.

The agent gathers evidence through a code subagent. The subagent is the only component that reads the target. It can list directories, read files, search with regular expressions, and cite line ranges. It cannot read anything under `.git`, and it cannot read files the engagement has excluded as documentation. Every read, search and citation it makes is written to a trace file, one file per evidence request, so the record of what was looked at is complete and separate from the agent's reasoning.

The audit stage works from a frozen claim surface. Enumeration produces the surface, a human at the practice scrubs it, and the audit adjudicates exactly those claims and no others. One finding per frozen claim, always. A claim that the materials cannot settle gets the verdict unverifiable, with the searches that were made recorded as its evidence.

## 3. Evidence rules

Evidence is source. Documentation, including the claim source itself, is never accepted as evidence for a claim about what the software does; the only exception is a claim about the document itself, such as a licence statement. This rule is enforced in the subagent, not only in the method: a citation into an excluded document is refused at the point of citation.

Every citation carries the document, the line range, the quoted text, and a sentence saying what the lines show. Before the review starts, the runner verifies mechanically for every citation that the document exists, the line range lies inside it, and the quoted text appears at those lines. All output is schema-constrained JSON, so a malformed finding is impossible.

Claims of absence, "no visitor data is stored", "the only logs are debug logs", are the hardest class. For these the method requires two searches over the whole tree, one lexical and one structural, and a finding for an absence claim always records both. A finding that records only one is sent back for the missing search before the audit closes.

## 4. The review

The review is a separate stage, run by a separate process, with a method of its own. It can be run with a different model from the audit. The reviewer receives the findings, the frozen claims, the target, and for every citation the cited lines placed under the finding. It does not compute statistics; the runner recomputes every count from the artifacts, so the review never reads the audit's account of itself.

The review is fully independent: the reviewer never sees the auditor's reasoning. It records four observations per finding: whether the cited material bears on the claim, whether it shows what the finding says it shows, whether the verdict is calibrated to the evidence, and whether the searches were adequate. A finding holds when all four are clean. The outcome is derived by the runner from the observations; the reviewer never writes it.

Every exception the reviewer raises is retested by a second reviewer that has not seen the first review. A sample of findings that hold is retested as well, so a reviewer that passes everything is caught.

The review also runs a recall pass: for every finding rated fully real, it reads the evidence requests filed under that claim and names lines in the audit's own record that qualify the claim and were not cited. Those claims are handed back to the auditor for re-adjudication with the named lines in front of it.

## 5. Measured performance

Our reference target is a small open-source Rust web service with a 58-claim README. On it, the audit-review-hand-back chain consistently holds 55 to 58 of 58 findings under independent review; the last three chains held 56, 58 and 55. Over 95% of findings hold. Every finding the search-completion step sent back has held on review, twelve of twelve across three runs.

Enumeration on the same README is stable to within a few claims across runs. Verdicts are stable in kind: the one claim in that README that the code contradicts, a cookie the README says is not set, has been found contradicted or partial in every run.

The pipeline has been exercised end to end on two external open-source targets and on a set of fixture documents with planted defects, where every planted defect was found.

## 6. Models and cost

The pipeline is model-agnostic: any OpenAI-compatible endpoint can serve any stage. Production runs use GLM-5.3-Flash on a hosted provider; a cheaper model has passed our qualification gates and is available as an alternative. Local serving on our own GPUs is supported for development. Sampling settings are fixed per model in code and recorded in every run record, together with the target commit, the harness commit, and the resolved model name, so a run can be reproduced. Temperature has never varied between runs of the same model.

An engagement's automated stages complete in about an hour on a target the size of the reference service. The dominant cost is the audit's evidence gathering; review and hand-back are a fraction of it.

## 7. Engagement handling

An engagement is a directory: the target, the claim sources by path, the buyer's transaction description and thresholds, the runs, and a state file that records which stage each run has reached. Intake is captured through a guided form on the client site. The site runs as a single process behind Cloudflare Access; a client opens their engagement by email identity, and a seller can upload materials to a dataroom the buyer never sees. Reports are delivered as self-contained printable pages, with a letter that states the liability posture of the practice.

The pipeline runs unattended end to end, from intake to delivered report.

## 8. Testing

A test suite of over 130 tests covers the workflow code: schemas, the chase logic, the output checks, batching, the hand-back, and the state machine. The measurement harness under `measure/` holds the fixtures, the graded runs, and the scripts that produced the figures in section 5.

## 9. Limitations we know about

The reviewer's own verdicts move between runs on identical text; two or three findings per run flip for no change in the finding. The recall pass names different lines on successive runs of the same claim, so the uncited-adverse figure is a sample, not a count. Calibration of caveated verdicts is the largest remaining failure class and no mechanism addresses it yet. Runs on the reference target take between fifty and ninety minutes for the audit alone. Larger targets have not been measured.

## 10. Roadmap

A second workflow, a security audit over the same method template, exists and has been run but its enumeration is not yet stable enough for delivery. Best-of-N selection on the pre-review output check, and a stronger model at hand-back, are designed and not built.
