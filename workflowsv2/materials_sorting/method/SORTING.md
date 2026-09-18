# Sorting one file of the seller's materials — method

## 1. Purpose

A claims review tests what the seller says about the software against the software. Before it starts, every file in the seller's materials that holds prose is sorted, because two lists depend on the sorting. The **claim sources** are the files the seller's claims are read from. The **evidence excludes** are the files the review may not cite as evidence. A file in which the seller describes the software belongs on both lists: its claims are to be tested, and it cannot be used to settle them, because a seller's description confirming a seller's claim settles nothing.

> Say what kind of file this is.

You decide one file at a time. You propose. A person at the practice sees your answer for every file, with your reason, and confirms or changes it before anything else runs.

## 2. What you are given

The path of one file in the materials. Where a listing found it: in the repository, inside an archive in the repository, or both. The documents in the materials that link to it, when any do. The text of the file with line numbers; a long file is cut after a stated number of words and the text says so.

Decide from what the text says. The path is given so that you can name the file in your reason, and it does not decide the kind: a file named `README.md` can be a test fixture, and a file named `index.html` can be a marketing page or a screen of the product.

## 3. The four kinds

- **`description`**: the file says what the software is or does, how to install, configure or use it, or what the seller does for its users. It is written for a reader who is deciding about the software or learning to use it. A README, an installation guide, a command reference, a marketing page, a store listing, a security policy that says how reports are handled, a file written to tell AI agents what the product is, the README of a test suite.
- **`instrument`**: the file is itself the fact that a claim would be about; it does not report a fact that lies somewhere else. A licence text, a NOTICE file, a contributor agreement, a continuous-integration workflow, a container or compose file, a package manifest. The test: a claim "the project is licensed under Apache-2.0" is settled by reading the licence file, and nothing behind the licence file could show otherwise. A claim "the importer handles every upload" is not settled by reading a guide that says so, because the importer's code could show otherwise; the guide is a description.
- **`product_text`**: words the running software shows to its own users. A page template, a screen of the interface, built-in help, an email the product sends. It shows what the product says on screen. A page that presents the product to people who do not yet use it, such as a landing page, is a `description`, even when it sits beside the interface files.
- **`neither`**: text that is not the seller speaking about this software. A third party's text, a test fixture, sample data, a form for reporting bugs, a generated file, an empty or near-empty file.

## 4. Mixed files

Some files are one kind in one part and another kind in another. A contribution guide sets the terms under which contributions are accepted, which makes those lines an instrument, and in another paragraph says that enterprise features are sold separately, which is the seller describing its own business.

When the file has a part of at least a few lines whose kind differs from the rest, give `kind` as the kind of the larger part and list every part of another kind in `parts`, each with its line range, its kind, and what that part says in one plain sentence. A badge, a single sentence, or a licence line at the foot of a README does not make a file mixed.

## 5. The output

One JSON object. Its shape is enforced; this section says what makes a field correct.

| Field | Contents |
|---|---|
| `kind` | One of `description`, `instrument`, `product_text`, `neither`: the kind of the file, or of its larger part when it is mixed |
| `parts[]` | Empty unless the file is mixed; then one entry for each part whose kind differs from `kind` |
| `parts[].lines` | The first and last line numbers of the part, as numbered in the text you were given |
| `parts[].kind` | The kind of that part, one of the same four values |
| `parts[].says` | What that part says, one sentence in plain words |
| `reason` | One or two sentences a person can check against the file: what the file says or is that makes it this kind. Name what you saw, and quote a few words of it when that is the quickest way to show it |

Emit nothing outside the JSON object.
