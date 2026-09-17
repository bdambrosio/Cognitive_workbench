# Methods and runners: a design pattern for organizing an agent workflow

The previous post looked at verification inside a chat agent: how it refers to evidence and justifies an answer. This one concerns a longer task, carried out across multiple model calls, with intermediate results that must be checked before a final product can be delivered.

The application is software claims review. Given statements about what a software system does, the workflow examines the supplied materials and produces a report of what they support, contradict, or leave unresolved. Its automated stages run unattended; a person reviews the result before release. The application provides the examples here, but the subject is how its instructions and execution controls fit together.

I have found it useful to separate a written **method** from a **runner**. The method specifies the work, its constraints, and the criteria for acceptable results. The runner is the program that supplies inputs, invokes the model, preserves work products, and uses explicit checks to decide when to continue, request corrections, or proceed to another stage.

This arrangement developed through implementation and testing. Several failures helped clarify the division of responsibilities.

## Instructions and execution controls

The method gives the model the context needed to exercise judgment. It describes what to investigate, what evidence is required, how to handle uncertainty, and what the output must contain. It also specifies procedural constraints, such as completing a work list before assessing its items. Within those constraints, the model chooses how to investigate and interpret what it finds.

The runner manages the parts of that process that can be expressed as program operations. It can supply documents in sections, assign identifiers, store results, and check that each requested item has a corresponding response. It can validate output structure and check references against the supplied materials.

Some requirements belong in both places. If each conclusion must include supporting evidence, the model needs instructions about what constitutes appropriate support. The runner needs checks for the presence and validity of the references. Implementing one does not remove the need for the other.

Reading the method alongside the runner exposed requirements that looked reasonable in isolation but were ineffective in practice. An instruction to consult the client was unusable without a communication channel. A check that rejected malformed references could still accept a report containing none. Other instructions left the model to infer decisions about scope that should have been supplied as inputs.

For each requirement, I now ask what the model is expected to do, what the program can observe, and what should happen if the requirement is not met. Some answers still require human judgment. Making that explicit is preferable to leaving the responsibility unassigned.

## Stable intermediate work products

A long task produces more than a final answer. It may produce a work list, collected evidence, assessments, and review comments. In this implementation, stages exchange structured records with identifiers, so later work can refer to particular items without reconstructing them from a conversation.

One useful control is to freeze an intermediate product before the next stage uses it. In the review application, the statements to be examined are collected and checked before assessment begins. The resulting list defines the work to be accounted for. The runner can then detect an item with no investigation or no result.

Freezing the list prevents it from drifting as the investigation proceeds. Otherwise, difficult items can disappear, new interpretations can replace earlier ones, and a report can appear complete simply because its account of the task has changed. A fixed list makes omissions and disagreements easier to identify.

It does not establish that the list is correct or complete. An omitted item remains omitted. Freezing also need not mean that revision is forbidden: a deliberate revision should be identifiable as such, with attention to any downstream work it invalidates. The useful property is that later stages have an explicit, stable input rather than an evolving understanding hidden in conversational context.

Preserving intermediate results also helps locate failures. A missing result, insufficient evidence, an unsupported conclusion, and an incorrect review objection are different problems. They are easier to distinguish when the inputs and outputs of each stage remain available.

## Continuing on observable conditions

An early execution failure concerned the meaning of a completed model turn. The agent runtime allowed the model either to respond or to yield with work to resume later. My runner interpreted a response without a yield as completion of the whole task.

One model did not use yield. It returned an initial list of statements to investigate, and the runner treated that list as its finished report. A later review rejected the output for lacking evidence that the model had never reached the stage of collecting. I had attributed the failure to the model when the runner had stopped it prematurely.

The correction was to look for the required work products. A turn could end without those products being present; the runner would then need to continue the work rather than submit the partial result for review. That change altered the model's qualification result in the small set of runs tested. It did not establish broad reliability, but it exposed an invalid assumption in the execution logic.

The same principle applies to more specific omissions. The runner can use an observed condition to select a defined next action:

| Observed condition | Next action |
| --- | --- |
| A required work product is absent | Continue with a request for the missing output |
| An item has no associated evidence request | Request a targeted investigation |
| An assessment relies on a potentially relevant file that was never opened | Collect that file and reassess the item |
| A reference does not match the supplied material | Return the mismatch for correction |
| Review identifies an unsupported conclusion | Return the objection for reconsideration, then review the result |

These actions have different purposes. Some complete missing work, some correct mechanically detected errors, and others respond to a judgment made during review. Recording which condition caused a continuation makes it possible to inspect what the runner actually contributed.

Repeated requests are not themselves evidence of progress. A runner also needs stopping conditions and a way to preserve unresolved failures. The goal is to make the response to a detected problem explicit, including when further automatic attempts are no longer warranted.

## Validation and substantive review

A mechanically valid output may still be wrong. A program can check that a referenced file exists, that a line range is valid, and that the quoted text appears there. Those checks do not establish that the passage supports the conclusion.

The workflow therefore separates structural validation from substantive review. A reviewing model examines the relationship between the evidence and the assessment. The runner can initiate that review, collect its objections, and send affected items back for correction. It cannot turn the reviewer's judgment into a mechanical guarantee merely by placing it inside an automated process.

Reviewers also vary. In one run, nine unchanged assessments received a different review status on a second pass. That observation is a reason to retain objections and review history, and to distinguish an unresolved disagreement from a settled correction. Human review remains part of release.

Presentation is another separate concern. A structured analytical record can be suitable for checking while being difficult for a reader to use. The application therefore prepares its report from the record in a later stage. This allows the evidence and reasoning to remain available in detail while the report is organized around the reader's needs. The companion post describes that separation and the interface for subsequent questions.

## What this changes about evaluation

Execution controls affect what a model evaluation measures. Before the runner enforced completion of the initial work list, following that instruction was a useful behavioral test. After enforcement, successful completion reflected both the model and the runner's interventions.

I now count how many requests were needed to obtain the required output. This retains some information about the model's behavior while evaluating the supported workflow. Performance without intervention is a different question and needs a different test.

The execution environment also needs to be recorded accurately. In one failure, the tool response carrying the method was truncated, so the model received only its opening sections. In another, a documented sampling setting was in a file the runner never read. Neither problem could be diagnosed reliably from the final output alone.

The practical benefit of the arrangement has been a clearer account of what happened: which instructions were supplied, which work products were produced, what checks failed, and what caused another attempt. The model still performs the investigation and interpretation. The runner makes selected obligations observable and manages the response when they are unmet.

The next post follows the software review itself, from the initial statements through evidence collection, assessment, correction, and the delivered report.
