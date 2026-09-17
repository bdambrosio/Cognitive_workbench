# A Software Claims Review, Stage by Stage

The previous post described how a written method and a program—the runner—organize an agent workflow. Intermediate work products are preserved, and explicit checks determine when to continue, correct an output, or initiate review. This post follows those choices through a software claims review, using figures from one recent run to illustrate what the stages produce and what reaches the client.

A prospective buyer of a software business may have a README, a sales listing, or a technical description explaining what the software does. Each specific assertion to be examined is a claim. The review compares those claims with the supplied evidence and records what is supported, what differs from the description, and what cannot be established from the available materials.

The scope is narrower than a general code review or a penetration test. It does not value the business or recommend whether to proceed. The intended result is a documented assessment that a buyer or another reviewer can inspect, question, and use in further diligence.

## Establishing the scope

Before the automated work begins, the engagement identifies the repository and commit to examine, the documents containing the claims to be reviewed, and any restrictions on the evidence the agent may inspect. The transaction context and the buyer's thresholds for material concerns are gathered during intake and recorded by a person.

The distinction between claim sources and evidence matters. A README establishes what was asserted; repeating that assertion elsewhere in the documentation does not establish that the software implements it. The review therefore specifies which materials may support a finding, rather than leaving the model to decide this during investigation.

These decisions define the engagement. They also limit its conclusions: the review concerns the supplied version and materials, within the agreed scope.

## Building the claim list

The first stage reads the claim sources section by section. Each extracted claim has an identifier, a verbatim quotation and its location, and a plain-language statement of the assertion to be evaluated.

Navigation, courtesy, intentions, and vague promotional language are excluded. “Blazing fast” is not a sufficiently specific claim; “handles 10,000 requests a second” is. Repeated assertions within a source can share one claim record with multiple locations. Assertions that can receive different judgments are separated: daily backups and thirty-day retention are two properties, even when they appear in one sentence.

Broad claims can be decomposed into more specific subclaims for human acceptance. The workflow also distinguishes claims about an implemented mechanism from claims about operation. Code may show a path for transferring a conversation to a human; whether conversations are successfully transferred in use is a further question. Throughput and uptime claims similarly require evidence beyond the existence of an implementation.

The claim list is reviewed and edited by a person, then held fixed as the input to assessment. The runner uses its identifiers to account for the investigation and result of each item. This is the first important intermediate work product: later stages work against the same list, although its completeness still depends on extraction and human review.

In the example run, a 625-line README yielded 163 claims. Separating operational assertions added 34, giving 197 claims for assessment.

## Collecting evidence and assessing claims

Evidence collection and verdict assignment are separate operations. During collection, the agent uses a code-reading subagent to list, search, read, and cite repository material. Each request identifies the claims it serves, and the runner stores the returned evidence under those identifiers. The agent must exercise judgment about where to look, but this stage does not produce the final verdicts.

For assessment, a separate call receives a batch of claims and their collected evidence, with no tools available. It produces a structured finding for each claim. This makes the evidence available at the point of judgment explicit and preserves it for subsequent review.

The runner uses the recorded work to detect specific omissions and initiate follow-up. A claim with no evidence request receives a targeted investigation. If an assessment says a claim cannot be verified but refers to a potentially relevant file that was never opened, the runner requests that file and sends the additional evidence for reassessment. These continuations respond to identifiable missing work, rather than relying on a general instruction to investigate thoroughly.

The method defines five verdicts: supported, supported with a caveat, partially supported, contradicted, and unverifiable. In the structured record, the corresponding labels are `real`, `real_with_caveat`, `partial`, `contradicted`, and `unverifiable`. The distinctions matter. A caveat describes a qualification to an otherwise supported claim, not uncertainty disguised as support. A contradiction requires evidence of a conflict; failing to find an implementation is not by itself sufficient.

Claims of absence require particular care. For a claim such as “no telemetry,” the method calls for both lexical and structural searches and examination of candidate files. The resulting finding must be understood within the scope of that investigation. Evidence requirements for operational claims also need to be explicit: code that supports a capability does not establish its observed performance.

Each citation identifies a file, a line range, and quoted text. The runner checks that these match the supplied material before substantive review.

The initial findings in the example run were 103 supported, 46 supported with caveats, 13 partially supported, nine contradicted, and 26 unverifiable. These were the assessment stage's results before review.

## Reviewing and correcting findings

A separate review agent receives the record and repository. It checks whether each claim faithfully represents its source and examines each finding for relevant evidence, support for the verdict, appropriate qualification, and adequate searches. The runner derives whether a finding passes review from these recorded observations.

Selected findings are also assessed by another reviewer without the first reviewer's judgment, providing information about agreement. When review questions a finding, the runner returns it to the assessment stage with the recorded objection, then submits the resulting record for review again. The objection is a model judgment; the routing and requirement for another review are program controls.

In the example run, 175 of 197 findings passed the first review. Correction changed eight verdicts, all from supported to supported with a caveat, reflecting features gated by a plan or setting. But the review itself was variable: nine findings that had not been changed acquired a different review status on the second pass.

The unchanged findings show why a separate review is not treated as ground truth. Unresolved objections remain visible for human review.

## Assessing significance for the buyer

The next stage considers findings against the buyer's stated thresholds. A discrepancy may be immaterial in one transaction and consequential in another. Claims that remain unverifiable are assessed separately for the exposure created by that uncertainty; they are not treated as established defects.

Ratings are repeated in independently shuffled batches. Disagreements trigger further ratings, with close results marked as borderline for human attention. The record retains the competing explanations. This measures stability under repeated assessment of the same inputs, not whether the rating is correct.

A materiality rating also does not repair a weak underlying finding. Where a finding has not passed review or has citation problems, that qualification accompanies its rating in the report.

## Delivering a report that can be explored

The report is assembled from the structured record. Code determines the figures and ordering; a constrained model call supplies explanatory passages. The report brings the transaction context, significant findings, unresolved claims, and questions for the seller to the front. The detailed claim records and their evidence remain available behind that account.

The example report ran to 111 pages. The intent is to be comprehensive about the work performed: readers should be able to inspect individual claims, verdicts, evidence, and qualifications. That does not mean expecting a buyer to read every page. The core account is up front, and the full document serves as a reference for questions that arise during diligence.

The live agent is an important part of that use. It answers from the review record and has access to the underlying materials, allowing the reader to ask about a particular claim, inspect its evidence, or explore the implications of a changed assumption. Citations can be checked against the materials again when a question is asked. The buyer can follow an issue into the detail without first working through the entire document.

The report is read and released by a person before the client can access it. Subsequent answers must distinguish explanations of the reviewed record from new analysis. The delivered report remains fixed; an answer generated later does not inherit its reviewed status.

The workflow aims to make the basis of each assessment available for inspection, while keeping the initial reading manageable. Its limitations remain consequential: an omitted claim will not receive a finding, and repeated assessments can disagree. Explicit scope, preserved evidence, automated checks, and human review make those limitations easier to examine, but do not eliminate them.
