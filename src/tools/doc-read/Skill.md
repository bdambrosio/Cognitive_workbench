---
name: doc-read
description: Read a PDF already on this machine. Returns a section index first, then any named section in full — the same way fetch-text reads a paper from a URL, for the case where the file is a local path instead. Use it when the user says they have downloaded or saved a document.
args:
  path: required string — path to the file (absolute, or relative to the user's Downloads folder)
  section: optional string — a name from the index; omit on the first call to get the index
---

# doc-read

For papers that arrive as a file rather than a URL. A portal that 403s a
fetch does not stop the user downloading the PDF themselves, and until
this existed that ended the line: the text was on the disk and there was
no way to read it.

Same shape as `fetch-text` on a paper. Call it once with just `path` to
get the title, abstract and section index; call it again naming a section
to read that section in full. Read the index before deciding what to
read — a survey can run to a hundred pages and the section names tell you
which five you actually want.

## When it earns its keep

- The user says "I downloaded it" or gives you a path.
- A fetch was blocked and the user saved the file instead.
- Re-reading one section of something long without pulling all of it.

## What it will not do

Only paths under the user's Downloads folder and this repository can be
read — it is a document reader, not a way to read arbitrary files on the
machine, and a request for anything outside those roots comes back as an
error rather than silently.

PDFs only. Anything else says so plainly.

## Reading it honestly

The index comes from the document itself, so the title it reports is the
document's own — not what a search result said the document probably is.
If that title is not what you expected from the user's description, say
so before reviewing it. Reviewing a paper the user did not ask about,
accurately, is still the wrong answer.

Then act on the mismatch, because noticing it is not concluding
anything. The document in front of you is the authority on what it is. A
title you remember, or that a search called the closest match, is a
guess about it and never evidence — so when the two disagree, the
remembered one is what gives way. Do not go looking inside this document
for the paper you expected; say plainly that this is a different
document and ask, rather than reviewing the one you have as though it
were the one you wanted.

## Examples

```json
{"thought": "get the structure before choosing what to read", "tool": "doc-read", "path": "53157_Diving_into_Reliable_Sel.pdf"}
```

```json
{"thought": "the taxonomy is the part that matters here", "tool": "doc-read", "path": "53157_Diving_into_Reliable_Sel.pdf", "section": "Introduction"}
```
