# What an Agent Contract Looks Like

Imagine we are considering acquiring a small SAAS company. A README says that a software product makes daily backups and retains them for thirty days. We ask an AI agent to verify; it finds a backup job in the repository and reports that the claim is supported.

Several questions remain. Did it check the schedule? Did it examine the retention setting? Does the repository show a configured job or evidence that backups actually ran? If retention was never investigated, would that omission be visible in the result?

An instruction to “check the claims and cite your evidence” leaves these decisions largely to the agent. For a buyer evaluating software, they affect what the review establishes. 

A previous article (Methods and Runners) proposed a written 'method' paired with a runner. In the new programming-for-AI model illustrated here, METHOD.md is the *contract*: it defines the work the agent must perform and the criteria its results must meet. The runner drives the agent through that work, checks compliance where it can, and routes substantive judgments for human review. Specified failures trigger limited retries or further investigation.

I have implemented this arrangement in a software claims-review workflow. Excluding the sections on professional practice, its METHOD.md for the core 'audit' step contains 13 sections and approximately 5,800 words. These cover scope, claim extraction, verdict definitions, evidence requirements, searches, observations outside the claim list, corrections, execution, and output records. The contract defines both what the agent must do and what it must leave to other stages.

That is considerably more specification than a one-line request. But the same decisions still have to be made when they are left unstated. For due diligence, how much should depend on whichever interpretation of a brief prompt the model happens to adopt?

The three excerpts below show how particular clauses work, what the runner does to support them, and where enforcement still depends on judgment. The records and filenames in the examples are illustrative; the quoted requirements and described controls come from the implementation.

## 1. Give every claim an identifiable result

METHOD.md, section 4:

> Every frozen claim produces exactly one finding, and every finding adjudicates exactly one claim. A claim carries one verdict, so two findings on one claim are either redundant or in conflict.

> **A finding names its claim by `claim_id` and does not restate it.** The frozen surface is what the claim says; the finding is what you concluded about it. Two copies of one assertion can disagree, and a reader would have no way to tell which was authoritative.

This requirement depends on establishing the claim list before investigating the evidence. The method calls that list the “claim surface.” Freezing it means that assessment uses the recorded assertions without adding, removing, or rewriting them.

The runner first supplies the designated claim source to the model section by section, with line numbers and previously extracted claims. This call is allowed no tool calls. Its job is to identify assertions, not to decide whether they hold.

For the backup sentence from the Readme described at the opening, the method requires two claims because frequency and retention can receive different verdicts: 

> **Divide by what is asserted.** One claim per property asserted about one subject. The properties are few: that something exists or is done; a default; a quantity or limit; a boundary — "only", "no", "all", "never"; a provenance fact — licence, author, origin. A sentence asserting two properties is two claims. The same property asserted in two places is one claim with two locations. The quote is the smallest contiguous span that carries the property and its subject; a lead-in that carries no property — a bullet marker, "i.e.", "and" — is left out of it.

A simplified list would be:

| Claim identifier | Source quotation | Assertion to assess |
| --- | --- | --- |
| 17 | “daily backups” | Backups run daily. |
| 18 | “retains them for thirty days” | Backups are retained for thirty days. |

The actual records also carry source locations. The runner assigns the identifiers and preserves the list for assessment.

Suppose the investigation returns a finding for claim 17 but none for claim 18. The runner can detect that omission by comparing the expected identifiers with those in the findings. It requests the missing finding once and checks the assembled results. Duplicate findings and findings referring to identifiers outside the list are also detectable.

There is a separate case: no evidence request was ever filed for claim 18. The runner first requests investigation of that claim. If it remains unattempted, it records the work as incomplete rather than allowing the model to label the claim unverifiable. “Unverifiable” means an investigation could not settle the assertion; it does not mean the agent never got to it.

The contract therefore requires more than a well-formed list of findings. It establishes a correspondence between the work assigned and the results returned. The runner can test that correspondence because it retains both.

This check has a specific limit. If extraction never identified the retention assertion, claim 18 would not exist, and no missing-finding check could recover it. The wider workflow includes human review of the claim list. The runner checks coverage of that list, not completeness of the original interpretation.

## 2. Turn an unfinished search into a request for further work

METHOD.md, section 8:

> **Every candidate is opened before a claim resting on searches gets its verdict.** A file the searches named and nobody read does not show that the thing is absent, or that the materials cannot settle the claim; it shows that the engagement has not looked. *Opened* means a read of the file was requested and what came back is in the evidence requests: its contents, or the failure to read them. A candidate that was opened and could not be read — a binary, a compiled archive, a truncated or encrypted file — settles nothing, and where no other evidence settles the claim, the disposition — the `unresolved_because` field, from the table below — is `present_but_not_readable`. If a candidate of any search on the claim was not opened, and no citation settles the claim, the verdict is `unverifiable` with the disposition `not_examined`; the client's process then asks you to open those files and to adjudicate the claim again, per §10. The other three dispositions are recorded only when every candidate has been opened. `not_examined` requires a named file: where the searches named no file, one of the other three applies.

The surrounding text defines a candidate as a file identified by a search where material that could settle the claim would appear. It requires the finding to record those paths.

Suppose the agent investigates claim 18 and searches for retention settings. Its search identifies a file named `config/backup-policy.yaml`, but it does not read that file. It then proposes that thirty-day retention cannot be verified.

The relevant part of its search record might read:

- Kind: lexical.
- Performed: searched configuration files for “retention” and “backup.”
- Result: found a possible retention setting.
- Candidates: `config/backup-policy.yaml`.

The method distinguishes four reasons an assertion remains unsettled: the supplied records do not settle it; relevant material is present but unreadable; the required kind of material was not supplied; or a named candidate was not examined.

Here, the last reason applies. The record points to unfinished work.

The runner compares candidate paths with the file-read records preserved from evidence requests. When the candidate has not been opened, it sends a further request naming the claim and the file. After that investigation, it submits the affected claim for assessment again, with the additional evidence.

If the file shows a seven-day retention setting, the assessment must consider that evidence. If the file cannot be read, the reason for uncertainty changes. Neither outcome is determined merely by issuing the request; the subsequent finding must account for what the request returned.

This follow-up is bounded. It stops when no relevant unopened candidates remain, when a pass opens nothing new, or when the execution limit is reached. Remaining omissions stay in the record.

The choice of output fields is essential here. “I searched thoroughly” gives the runner little to check. A list of candidate paths can be compared with the execution record and used to construct a specific request.

The program still cannot establish that the agent chose good search terms or identified every relevant file. An empty candidate list may reflect a thorough search or a poor one. Those questions require substantive review. The implemented check addresses the narrower case in which the agent has already identified a file and then left it unread.

## 3. Define judgments that a schema cannot enforce

METHOD.md, section 6:

> **A caveat is not for weak evidence.** If what you have does not settle the claim, the verdict is `unverifiable` and §8 governs it. A framework version inferred from directory names, where no manifest or lockfile was supplied, is `unverifiable` — not a claim that holds with a caveat. The verdict says how the claim fared against the evidence; it does not say how confident you are.

The method distinguishes a supported claim that needs qualification from a claim the available evidence cannot settle.

Consider the assertion “The backup service defaults to thirty-day retention.” Suppose the supplied source establishes that the service creates backups, but its default retention policy is supplied by a deployment file that is absent.

An abbreviated proposed finding might say:

- Verdict: supported with a caveat.
- Caveat: the default retention setting could not be checked.
- Evidence: the function that creates a backup.

The fields are present, and the citation may be accurate. The finding nevertheless violates the contract. Backup creation does not establish default retention, and the caveat acknowledges that the asserted property remains unknown.

Under the method, the result should instead remain unsettled, with the missing deployment material identified as the reason. The record must also account for the investigation that established what was available.

Now change the evidence. Suppose a supplied default configuration specifies thirty days, and the implementation shows that an administrator can override that value. A finding of supported with a caveat can be appropriate: the claimed default is present, and the qualification explains that configured retention can differ.

These examples require interpretation of both the assertion and the evidence. They cannot be distinguished simply by requiring a verdict field, a qualification field, and a citation.

The runner gives this judgment a defined setting. Evidence gathering uses tools and records the requests and responses under claim identifiers. Assessment is a separate call, with no tools, receiving the fixed claims and their collected evidence. Its output can then be checked and reviewed against those inputs.

Programmatic checks can identify malformed records, invalid line ranges, and quotations that do not resolve against the cited files. They do not establish that the quoted material demonstrates thirty-day retention. A subsequent reviewer must evaluate that relationship, using the same verdict definitions.

This is a contract requirement whose substantive enforcement depends on review. Keeping it explicit lets a reviewer identify a particular error: the finding treats missing evidence as a qualification to a supported claim.

## What the runner enforces

These examples involve three different kinds of control.

The fixed claim list lets the runner check whether assigned items have corresponding results. Candidate paths and file-read records let it detect particular unfinished investigations and request further work. Verdict definitions provide criteria for judgments whose correctness still needs substantive review.

A written prohibition alone provides a weaker control. METHOD.md also says that the audit must not value the business or recommend whether to proceed. The runner asks for individual findings and provides no dedicated field for an acquisition recommendation. That reduces the opportunity for such an output, but a model could still insert advice into a free-text field. The instruction remains subject to review.

Calling the method a contract does not imply that every clause is enforced in code. It makes the obligations explicit so that the implementation can identify which are prevented by the way inputs and calls are arranged, which are checked afterward, which trigger correction, and which require judgment.

## Writing the contract and runner together

In this implementation, the written method and the runner depend on each other's design. “Assess every claim” becomes checkable only when the claim list exists independently of the findings. “Examine the files your search identifies” becomes actionable only when the output names those files and the execution record preserves which were opened.

The semantic rules serve a different purpose. “A caveat is not for weak evidence” defines what an acceptable judgment means, even where a program cannot establish compliance on its own.

For each requirement, the useful design questions are concrete: what record would reveal a violation, who or what can recognize it, and what happens next? An answer might be a programmatic check, another investigation, substantive review, or an explicit unresolved limitation.

That is the role of the contract in this example. It specifies the work closely enough to assign those responsibilities. The runner implements the controls it can support, while preserving the evidence and unresolved questions needed to review the rest.
