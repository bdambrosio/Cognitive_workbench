# Approved answers

Answers the practice has approved to questions people ask about Tuuyi. When a
reply asks one of these questions, the answer may use the wording here. Nothing
may be stated about Tuuyi beyond this file and PROSPECT.md §2.

Each entry is a question, the approved answer, and a line saying where its facts
come from and when they were last checked. An answer that gives a figure names
the review the figure comes from; when that review is replaced, the entry is
updated or removed.

No price, fee, discount, turnaround time or other commercial term appears in
this file. The practice states those itself, in a reply to a particular person.

Bruce edits this file directly. New entries are proposed from answers he has
sent, and he edits or cancels each one before it is added.

## We already ask Claude or GPT about the data room. How is this different?

Many teams already do. A model answers the questions you think to ask it.
Tuuyi starts from the other end: it identifies every claim the seller's
documents make and settles that list before testing any of them, adds the
buyer's own questions and a standard list of questions a buyer of a small
software company should ask, then tests the material ones against the code.
Every finding names the file and lines it rests
on. What it could not settle, and what it did not examine, are listed too, so
you can see where the answer stops. A second pass that did not do the review
re-reads the evidence behind every finding, and a person reads the report
before it is released.

Source: PROSPECT.md §2; site /how-it-works. Checked 2026-10-02.

## Why are only some of the claims tested?

Every claim in the seller's documents is listed and rated, before any testing,
by what it would change for the buyer if it were false. The claims that could
change the price, the terms or the decision to close are tested against the
code. The rest are listed with the reason they were not tested, and the buyer
can ask for any of them to be tested. In the public demo, 312 of the seller's
claims were listed and 38 were tested; each of the 38 has a finding with its
evidence. The buyer's 5 questions and 14 of the 15 standard questions were
tested too.

Source: the demo review of 2026-10-02 (report scope table and second
appendix), served at demo.tuuyi.com. Checked 2026-10-02.

## Does it only check what the seller claims?

No. At intake the buyer can raise questions of their own and say what a "no"
to each would do to the deal, and the review adds a standard list of questions
a buyer of a small software company expects answered. Each is tested against
the code like a claim, and a gap is reported as a finding about the software,
not about what the seller said. In the demo, the buyer asked whether each
customer's links could be kept separate; the code has no way to do that.

Source: PROSPECT.md §2; site /how-it-works ("Your questions, and the standard
ones"); the demo review of 2026-10-02. Checked 2026-10-02.

## Is this a PCI DSS, SOC 2 or GDPR compliance assessment?

No. Those are formal assessments by accredited assessors, and they cover the
organisation as well as the code. The review adds questions drawn from
published requirements for the kind of software it is (OWASP ASVS, the OWASP
Top 10 for LLM applications, GDPR and PCI DSS among them) where the software
has that kind of data or feature and a "no" would matter to the buyer. Each is
tested against the code. A gap is a red flag to raise with the seller, or to
send a full assessment to look at, before you commit. The review does not
certify compliance with any standard.

Source: claims_audit method/PROPERTY_QUESTIONS.md and PROPERTY_SELECTION.md;
site /how-it-works ("Your questions, and the standard ones"). Checked
2026-10-02.

## How is this different from a code scanner or a code-analysis platform?

Scanners and code-analysis platforms start from the code: they check it
against their rules, score its quality and security, and some compare it with
other codebases. Tuuyi starts from what the buyer is being told and asked to
rely on. It identifies the claims in the seller's documents, adds the buyer's
questions and the questions published standards ask of that kind of software,
rates each by what a "no" would change for this buyer, and tests the material
ones against the code. Each answer is a verdict on one statement, with the
file and lines it rests on, and a person reads the report before it is
released. The two can sit side by side: the review can also include a scan of
the dependencies and their licences.

Source: PROSPECT.md §2; site /how-it-works. Checked 2026-10-02.

## Can I see an example?

Yes. The public demo at demo.tuuyi.com is a finished review of a small
open-source project, and you can put questions to it. For example, the
project's README promises the software will never use cookies, and the review
found the code that sets one when the administrator logs in.

Source: demo.tuuyi.com; PROSPECT.md §11. Checked 2026-09-27.

## What does a finding look like?

Each finding states one claim as the seller wrote it, gives one of five
verdicts (holds; true, with something to know; partly true; contradicted;
unsettled), and quotes the file and lines of the materials it rests on, so you
can check it yourself in a few minutes. A gap is rated by what it would change
for this buyer.

Source: PROSPECT.md §2; the demo report. Checked 2026-09-27.

## What happens when the materials can't settle a claim?

It is reported as unsettled, not as false, with what the review searched and
why that did not settle it. Where the claim is about something the materials
cannot reach, such as a running system or the seller's own conduct, the report
gives the question to put to the seller.

Source: the demo report, "Unsettled claims" and "Unsettled claims about the
seller". Checked 2026-09-27.

## Does it run the code?

No. The review reads the code and the other materials; nothing is compiled or
run. A claim that only a running system could settle, such as a memory figure
or an image size, is reported as unsettled.

Source: site /sellers ("The review reads the code and does not run it"); the
demo report. Checked 2026-09-27.

## Is this a replacement for technical due diligence?

No. For a smaller deal, where full technical diligence would cost too much for
the size of the transaction, it can be a lower-cost first review. For a larger
deal, it can come before the main diligence and show where that work should
concentrate. It is not a penetration test and not a review of code quality or
architecture.

Source: PROSPECT.md §2; site /pricing ("What it does not include"). Checked
2026-09-27.

## What about a seller checking their own claims?

A seller can have the claims in their own README, documentation or deck checked
against their code privately, before a buyer, investor or large customer does,
so they can correct the words or the code first. Only the people the seller
admits to the engagement can open the report, and each of them can read and
download it. A seller can admit a buyer, so a buyer can ask a seller for one
before the letter of intent, while the seller is not yet sharing code: the
buyer sees the report, not the repository. It is not an independent
assessment for anyone else to rely on. It is described at tuuyi.com/sellers.

Source: PROSPECT.md §2 and §10; site /sellers; client_ui/access.py (who may open
an engagement). Checked 2026-10-02.

## Where does the code go? Who sees it?

For a buyer's paid claims review, the repository is examined on single-tenant
hardware the practice controls, in an environment created for the engagement
and deleted at the end of the period set in the engagement letter. The buyer
never sees the environment or the materials in it. The code goes to a cloud
model service only when the client and the seller both agree, and then only to
one that retains no data. A review done during the beta may run on a hosted
model instead; the engagement letter says which applies. A seller's own claims
check is handled privately.

Source: site /pricing ("Where the work runs"); site /privacy. Checked
2026-09-27.

## Does the seller have to take part?

The seller supplies the repository and the documents, on a page that only the
seller and the practice can open. The seller is not consulted on how the
claims are read, and the report says so.

Source: site /privacy ("Engagements"); the demo report ("Responsibilities").
Checked 2026-09-27.

## Who is behind Tuuyi?

Bruce D'Ambrosio, Professor Emeritus of Computer Science/Artificial Intelligence at Oregon State
University, has worked in artificial intelligence for four decades. He
founded CleverSet, acquired by Art Technology Group in 2008, and has done
technical due diligence from both sides of software acquisitions.

Source: site /about. Checked 2026-09-27.
