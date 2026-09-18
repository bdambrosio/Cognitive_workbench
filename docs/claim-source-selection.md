# Sorting the materials: claim sources and evidence excludes

Status: working procedure. First written 2026-09-17 after the `cw-site`
engagement, generalized the same day from a second look at the `chattermate`
and `demo-chhoto` engagements. Steps 2, 3, 5 (for files inside the target),
6, 7, 8 and 9 are carried out by `workflowsv2/materials_sorting/runner.py`,
which the practice page starts with "Sort the materials" and whose proposal a
person confirms on the sorting page; enumeration is refused until then. The
runner reads `workflowsv2/materials_sorting/method/SORTING.md`, not this
document. What the runner does not do is still done by whoever prepares the
engagement, at present Claude working with Bruce in a terminal session:
reading what intake said against the proposal (step 1), comparing copies and
pinning a snapshot (step 4), fetching anything outside the target, and
deciding regions of a web page beyond the default (title, meta description,
main content). Change this document when the procedure changes, and add a
dated line under "Changes" saying what changed and why.

## What this step decides

After intake, and after the seller's materials are marked ready, two lists
have to be exact before enumeration can run:

- `claim_sources:` the text files the seller's claims are enumerated from;
- `evidence_excludes:` the paths the review may not cite as evidence.

Intake gives both in general words or not at all: "the README and their
website", "whatever they sent us". Each list decides what the seller is held
to and what may be used to settle it, so both are proposed in one record and
confirmed by a person before anything is extracted or enumerated.

The two lists come from one sorting of the materials. They are not the same
list: on `demo-chhoto` the claim source was `README.md` and the project's own
marketing page sat in the target at `site/index.html`, neither selected as a
claim source nor excluded as evidence.

## Terms

- **Candidate**: anything the seller wrote, or published under the product's
  name, that a buyer could read: a file in the target, a file inside an
  archive in the target, a page on a host the materials link to, the text of
  a listing (repository description and topics, a package registry page, an
  app-store page, a container registry page), a wiki, a video.
- **Kind**: what a candidate is, decided by reading it, never by its name or
  extension. Four kinds:
  - **description**: says what the software is or does, or what the seller
    does. A README, an installation guide, a marketing page, a store listing,
    `llms.txt`, a test suite's README. A description is a possible claim
    source and is never evidence.
  - **instrument**: the file is itself the fact. A licence, a NOTICE, a
    contributor agreement, a CI workflow, a compose file, a package manifest.
    An instrument is evidence. It is a claim source only if the client asks.
  - **product text**: words the running product shows its users: interface
    templates, built-in help, error messages. Evidence of what the product
    says. Not a claim source unless the client asks.
  - **neither**: third-party text, test fixtures, issue templates, generated
    files.
- **Mixed**: a candidate that is part description and part instrument.
  ChatterMate's `CONTRIBUTING.md` sets the contribution terms (instrument)
  and also says that enterprise features are sold separately (description).
- **Source file**: one text file under `target/claim_sources/`, one entry in
  `claim_sources:`. One enumeration run reads one source file.
- **Region**: a part of one candidate: for a web page the main content,
  header, navigation, footer, forms, `<title>` and meta description, caption
  track, alt text; for a document its body, front matter, badges, embedded
  images.
- **Selection record**: the document written in step 6, kept as
  `SELECTION.md`.

## Procedure

1. **Read what intake said.** Quote the client's words for the claim sources
   and for anything they said should not count as evidence. Quote the
   transaction's subject: it says what the buyer is relying on. A buyer taking
   over a hosted service relies on the pricing page and the store listing; a
   buyer of source code may not.

2. **List every candidate, from more than one listing.** The listings are
   mechanical and deliberately too wide:
   - every tracked file in the target that is not source code, with its word
     count, including files inside archives;
   - every file the already-known claim sources link to by relative path (a
     README that says "see INSTALLATION.md" makes that file part of what the
     buyer was told);
   - every host those documents link to, with the number of links;
   - the repository's hosting metadata: description, topics, homepage, wiki,
     release notes;
   - for a website: the sitemap, the site directory when there is one, every
     link and media reference in the pages.
   Say which listings were used and where they disagree. A candidate found by
   one listing and missing from another is reported.

3. **Decide each candidate's kind by reading it.** Open it. A file's name
   does not settle its kind: `chattermate-test/README.md` and
   `frontend/index.html` have to be read to know what they are. For a mixed
   candidate, say which parts are which.

4. **Identify the copy to be read, and pin it.** Where a candidate exists in
   more than one copy (the live site and a copy in the repository, a caption
   file and a printed transcript), compare every one, not a sample, and say
   which are identical and how the others differ. Read the copy that can be
   pinned: a file at a commit, or a snapshot taken now. For anything fetched
   from the network keep the fetched file, its URL, the time of the fetch and
   a SHA-256, under `target/claim_sources/`. The live copy will change; the
   published copy and the repository copy may already differ, and then the
   client chooses which one the seller is held to.

5. **Propose the two lists.**
   - *Claim sources.* Every description inside the target is proposed as
     included. Every description outside the target is listed with its size
     and proposed as the client's choice, because the fee follows the number
     of claims and an outside host can be larger than everything in the
     repository. "It has no claims" is not a reason to leave a candidate out.
     Reasons that count: not the seller's text; a separate product or
     deliverable agreed out of scope; no text that can be quoted (say what
     would be needed); every word already in an included candidate, checked
     by comparison; an orphan page (Bruce, 2026-09-17): no link leads to it
     from the site's home page, directly or through other pages, and the
     sitemap does not list it, so no buyer reached it except by a wrong
     address. An orphan stays in the record with that reason and, if it is
     the seller's description, is still excluded as evidence. The client can
     include one they were sent the address of, such as a landing page
     reached from an advertisement. The sorting runner does not trace links
     from a home page; for a website this check is made by hand.
   - *Evidence excludes.* Every description is excluded, whether or not it
     was selected as a claim source, together with the extracted source
     files. Instruments and product text stay in. A mixed candidate is put to
     the person with both readings; the default is to keep it as evidence and
     say in the brief which of its statements are the seller describing
     itself. Delivered reports and earlier claim surfaces of the same target
     are excluded.
   - *Regions.* For each included candidate say which regions are kept and,
     for each region dropped, give an example of the text lost in the seller's
     words. Check each time: header or navigation text that is a statement;
     `<title>` and meta description; footer; text written by script; alt text
     and words inside images (open each image); badges; video and audio, where
     a printed transcript or caption file is the claim source for narration and
     what appears only on screen is out unless someone transcribes it. A
     statement repeated on every page is included once.

6. **Write the selection record and put it to the person.** One row per
   candidate: what it is, which listing found it, its kind, its size, claim
   source or not, excluded as evidence or not, the reason, the source file it
   becomes. Below the table: differences from what intake said (sources or
   excludes the client and the practice did not name), the judgements the
   table does not show, and an estimate of the claims added by each optional
   candidate. Nothing is extracted until the person has agreed the record or
   changed it.

7. **Extract.** `utils.doc_extract.extract_to_markdown` handles PDF and HTML;
   markdown and text files are copied as they are. One source file per
   candidate under `target/claim_sources/<source>/`. Split a candidate only
   when its parts are different kinds of text, and record the split.

8. **Check the extraction against the original.** Compare the word count of
   each source file with the visible text of the regions it came from. Any
   shortfall is looked at, however small. Read each source file once.

9. **Record.** `engagement.yaml`: both lists, with a comment saying where the
   text came from. `brief.md`: what was included, what was left out and why,
   and which statements in a mixed evidence file are the seller's own, in
   words the auditor and a client can both read. `SELECTION.md`, with who
   confirmed it and what they changed, in `<engagement>/sorting/`, which is
   outside the target because the seller can see the target. (`cw-site`'s
   record predates the runner and was written by hand; it sits in the same
   place.)

## What the procedure does not decide

Which enumerated claims are about the software and which about the seller:
that is tagged at enumeration and corrected at the scrub. Whether two source
files repeat a claim: they will, and the report carries the repeats. Whether
comments and docstrings inside source files are descriptions: open question,
not addressed by the current excludes, which work on paths.

## Worked examples

### `cw-site`, 2026-09-17

Instruction: "use the tuuyi.com website". Copy read: `site/*.html` in the
repository at 3b57e91d. The record is at
`engagements/cw-site/sorting/SELECTION.md`. First-pass
errors, found by looking back: the sitemap was not checked until after the
proposal; two pages were dropped on a reading of their headings; a linked host
was left out without being mentioned; the header banner and the meta
descriptions were dropped by the extraction without being mentioned; the
first extractor lost text that sat directly in a `<div>`; the live comparison
covered six pages of nine and the one that differed was among the three not
compared; the video's poster frame, not opened at first, carries a sentence the
narration does not.

### `demo-chhoto`, looked at again 2026-09-17 (nothing re-run)

As run: claim source `README.md` (932 words, 58 claims); excludes
`README.md`, `docs/`.

Descriptions in the target that were not claim sources: `docs/INSTALLATION.md`
(2,220 words), `docs/CLI.md` (1,155), `docs/TOOLS.md`, `docs/SECURITY.md`,
and `site/index.html` (440 words of page text, the project's public landing page, titled
"Blazingly Fast, Selfhosted URL Shortener"). The README links to the first
three by relative path. Together they are about five times the README.

Missed exclude: `site/`. The landing page was available as evidence. No
finding cited it (checked in the 2026-09-07 merged findings), so no verdict
is affected.

Outside the target: the GitHub description ("A simple, blazingly fast,
selfhosted URL shortener with no unnecessary features; written in Rust"),
the Docker Hub page, `chhoto.link` (differs today from the pinned
`site/index.html`), `demo.chhoto.link`.

Correctly left as evidence: `backend/Cargo.toml`, `deploy/compose.yaml`,
`LICENSE`, the CI workflows (instruments); `frontend/index.html` (product
text). Two findings cite `.github/ISSUE_TEMPLATE/bug_report.md` for who is
assigned bug reports; an issue template was sorted here as "neither", and that
use shows a "neither" file can still be an instrument for a claim about
process. Kind is decided per candidate, and the record should say so when a
file is kept for that reason.

### `chattermate`, looked at again 2026-09-17 (nothing re-run)

As run on 2026-09-15: engagement `chattermate` names four claim sources
(`README.md`, `llms.txt`, `HELP_CENTER_INFRA.md`,
`backend/app/knowledge/README.md`); the chain enumerated `README.md` only
(197 findings after the behaviour split). No `evidence_excludes:` key, so the
excludes were those four files. The sibling engagement `chattermate-readme`
has `evidence_excludes: []`, which excludes nothing, the README included.

Descriptions in the target never listed: `frontend/README.md`,
`backend/tests/README.md`, `backend/tests_live/README.md`,
`chattermate-test/README.md`, and `README.md` plus `readme.txt` inside
`wordpress/chattermate-chat.zip` (the WordPress plugin's listing text; found
only by looking inside the archive). None was excluded as evidence.

Mixed: `CONTRIBUTING.md`, cited eight times. For claim 163 (contributions
under Apache-2.0 with DCO sign-off) it is the instrument. For claim 161
(enterprise features are sold separately), verdict real, it is the seller's
description confirming the seller's claim.

Outside the target, by links from the README: `docs.chattermate.chat` (nine
links; its sitemap lists 34 pages), `chattermate.chat` (the marketing site),
`app.chattermate.chat`, a Shopify app listing, an npm package page, a Docker
Hub page, a YouTube video; the GitHub description ("Self-hosted alternative
to Intercom & Zendesk") and topics; a wiki is enabled. The transaction's
subject includes the hosted service and its customer contracts, so the
marketing site and the Shopify listing are what this buyer relied on, and
neither was proposed to them.

## Changes

- 2026-09-17: first version (websites only).
- 2026-09-17, after the first run of the procedure as written: compare every
  candidate with its other copy, not a sample; checked duplication as a reason
  to leave a candidate out; open each image.
- 2026-09-17, after looking again at `demo-chhoto` and `chattermate`: the step
  now decides the evidence excludes as well as the claim sources, from one
  sorting of candidates into four kinds; candidates include files in archives,
  files linked by relative path, hosting metadata and outside hosts; intake's
  words and the transaction's subject are read first; outside descriptions are
  the client's choice because they change the fee; the published and the
  pinned copy of a page can differ and the client chooses.
- 2026-09-17: the step became a stage, `sorting`, between `materials` and
  `enumeration`, with a runner and a method document. Run on a copy of the
  chhoto target with GLM-5.3-Flash it took 52 seconds for 21 files and proposed
  the same five missed descriptions and the missed `site/` exclude that the
  hand review found. The record moved out of the target. The extraction check
  now counts the original page's words without using the extractor; the first
  version compared the extractor's output with itself.
- 2026-09-17, evening: orphan pages left out by default (the not-found page of
  tuuyi.com was the case; the privacy page, which looked unreachable, is linked
  from every page's footer and stays in). The order of `claim_sources:` matters
  to the duplicates pass, which keeps the earlier document's copy of a repeated
  claim: the primary document goes first.
