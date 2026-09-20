# Selection record: `chhoto-full`

Status: **CONFIRMED 2026-09-20T23-31-09Z by claude 2026-09-20, for Bruce: all six descriptions as proposed; nothing outside the target (illustrative buyer)**. Prepared by `workflowsv2/materials_sorting/runner.py` with accounts/fireworks/models/glm-5p3-flash; the procedure is `docs/claim-source-selection.md`.

## 1. What intake and the engagement said

The brief, whole:

```
(no brief)
```

`claim_sources:` in engagement.yaml before this step: none

`evidence_excludes:` before this step: none (defaulted to the claim sources)

## 2. Files that hold prose, and their kind

Target at commit 6a1b51cdeafdaadd46560e30a7c1a36bd1f08388. 21 files read. Passed over without reading, by type: .css 2, .ico 2, .js 2, .lock 1, .png 5, .rs 20, .svg 2, .svgz 2, .toml 2, .webp 4, .yaml 3, .yml 10.

| File | Words | Linked from | Kind | Claim source | Claims, estimated | Excluded as evidence | Reason |
|---|---|---|---|---|---|---|---|
| `.gitattributes` | 45 |  | instrument |  |  |  | This is a git attributes file configuring large-file storage for nine asset paths ('screenshot-desktop.webp filter=lfs ...'); it is itself the configuration fact, not prose about the software, so any claim that these files are LFS-tracked is settled by reading it alone. |
| `.github/ISSUE_TEMPLATE/bug_report.md` | 154 |  | neither |  |  |  | This is a GitHub issue template — a form for users to fill in when reporting bugs ("name: Bug report", "**Describe the bug**"), not text in which the seller describes the software. It is a blank form template, so no claim of the seller's can be tested against it and it is neither claim source, instrument, nor product text. |
| `.github/ISSUE_TEMPLATE/feature_request.md` | 91 |  | neither |  |  |  | This is a GitHub issue template that collects feature requests from users, listing prompts like "Describe the solution you'd like"; it is a form for reporting input to the project, not the seller speaking about the software. |
| `.github/pull_request_template.md` | 60 |  | neither |  |  |  | The file is a pull request form: prompts for a change summary and a checklist ('I have reviewed and understood all the changes included in this PR') governing how a contributor submits a change, not text in which the seller describes this software. |
| `.gitignore` | 24 |  | instrument |  |  |  | The file is a .gitignore, a version-control configuration whose lines are themselves the fact about which files are kept out of the repository — like a CI workflow or compose file, nothing behind it could show otherwise. It reports no claim lying elsewhere and says nothing about what the software is or does. |
| `LICENSE` | 169 |  | instrument |  |  |  | This is the MIT License text itself: a licence granting rights to 'use, copy, modify, merge, publish, distribute, sublicense, and/or sell' the software with no warranty. It is the fact that a licensing claim such as 'the project is MIT-licensed' would be checked against, not a description reporting facts held elsewhere. |
| `Makefile` | 472 |  | instrument |  |  |  | The file is a build automation recipe — targets like build, test, podman-run, deploy and publish define the commands the project actually uses to build, test and release the software, so it is itself the fact that claims about building or deploying would rest on, not a report of facts living elsewhere. It reports nothing for a reader deciding about the software, and the SPDX licence notice in lines 1–2 is a mere header line that does not make the file mixed. |
| `README.md` | 932 |  | description | `README.md` | about 62 | yes | The file is the project's README: it says what the software is and does for a reader deciding about it — "A simple selfhosted URL shortener", a Features list, a Demo link, and pointers to installation and CLI docs. The SPDX licence lines at the top and the badges do not make it mixed. |
| `deploy/Containerfile` | 106 |  | instrument |  |  |  | This is a container build recipe, given in the SORTING.md list of instruments: it is itself the fact that a claim like 'the project publishes a minimal container image' would be about, and reading it settles such a claim. Its content is build instructions (e.g. 'FROM scratch', 'RUN cargo build --release'), not prose describing the software. |
| `deploy/Containerfile.alpine` | 55 |  | instrument |  |  |  | This is a Containerfile: a build recipe that itself is the fact a claim like 'the image is built from alpine and runs the chhoto-url binary as its entrypoint' would be about. Reading it settles such a claim directly, with nothing behind it that could show otherwise, which fits 'the file is itself the fact that a claim would be about'. |
| `deploy/Containerfile.debug` | 27 |  | instrument |  |  |  | This is a Containerfile — a container build definition that builds a debug image from a copied binary ("FROM alpine AS builder", "ENTRYPOINT [\"/chhoto-url\"]"). It is itself the fact a claim about the deployment or build process would be about, not a report of facts lying elsewhere, and it says nothing describing the software to a reader. |
| `deploy/Containerfile.scratch` | 46 |  | instrument |  |  |  | A container build definition that copies the built binaries and frontend into scratch images and sets the entrypoint; it is itself the fact that claims like "the image contains only the binary and frontend" would be about, not a report of a fact lying elsewhere. |
| `deploy/chhoto-url.container` | 130 | docs/INSTALLATION.md | instrument |  |  |  | This is a container (quadlet) unit file that itself configures how the software is deployed — image, ports, capabilities, environment variables, volumes and service restart behaviour. A claim about such configuration is settled by reading this file; it does not report facts that lie elsewhere. The explanatory comments, including the licence headers, are a few isolated lines that do not make the file mixed. |
| `docs/CLI.md` | 1155 | README.md | description | `docs/CLI.md` | about 77 | yes | The file is documentation written for users of the software: it introduces the official CLI app and then explains how to call the API, with sections like '## Instructions for CLI usage' and per-endpoint guides such as '#### `/api/expand`' showing curl examples and expected responses. A command reference is a description; its claims about the API are to be tested against the code. |
| `docs/CONTRIBUTING.md` | 227 |  | instrument |  |  |  | This is a contribution guide that sets the terms under which contributions are accepted — e.g. "No AI contribution is allowed", discussions required for new features, and formatting rules — so it is itself the fact a claim about contribution policy would rest on, not a report of a fact elsewhere. No part of a few lines or more speaks of the software in another register. |
| `docs/INSTALLATION.md` | 2220 | README.md, docs/CLI.md | description | `docs/INSTALLATION.md` | about 148 | yes | This is the seller's installation and configuration guide, written for readers deciding how to deploy and run the software: it explains docker compose setup, image flavors, and every environment variable ('Location for the database file...', 'The address Chhoto URL will bind to. Defaults to 0.0.0.0.'). Such claims about behaviour and defaults are to be tested against the code, so the file is a claim source, not evidence. |
| `docs/SECURITY.md` | 56 |  | description | `docs/SECURITY.md` | about 4 | yes | A security policy that tells readers how vulnerability reports are handled ('report it using one of the following ways', by email or GitHub security advisory) and which release is supported; this is the seller speaking to people about the software, not a fact settled by the file itself. |
| `docs/TOOLS.md` | 178 | README.md | description | `docs/TOOLS.md` | about 12 | yes | The file is a page in the seller's own docs, linked from the README, that tells readers what tools exist around the software: an official CLI ('maintained by me') and third-party browser, Raycast, FreeBSD and NixOS options. It is written for a reader learning what they can use with Chhoto URL, which makes it a description; no section is of a different kind. |
| `frontend/index.html` | 70 |  | product_text |  |  |  | The file is the interface screen of the running shortener: it shows the links table with columns like "Short URL / Long URL / Hits" and interface words such as "Loading links table...", which the product displays to its own users. The one-line self-description on line 3 is a page metadata tag, not a separate prose section, so the file is not mixed. |
| `frontend/static/404.html` | 26 |  | product_text |  |  |  | This is a custom 404 error page — words the running software shows to its own users when a page is missing, as in "# Error 404!" followed by a haiku. It shows what the product says on screen, not the seller describing the product to outsiders. |
| `site/index.html` | 440 |  | description, mixed | `claim_sources/site_index_html.md` | about 29 | yes | This is a landing page presenting Chhoto URL to people deciding about the software: it pitches the product ('Blazingly fast self-hosted URL shortener', line 5), lists features (lines 23-49), and gives a quick-start guide (lines 55-69). Lines 71-75 are an 'About me' biography of the maintainer, not text about the software. |

### Mixed files

Parts whose kind differs from the file's. Where a file kept as evidence has a part that is a description, the brief should name that part as the seller's own statement. Decide each one.
- `site/index.html` lines 71-75, neither: The maintainer introduces himself personally, describing his research background and programming hobbies rather than the software.

## 3. Outside the target: the client's choice

Nothing here was fetched. Each is the seller's text a buyer could have read; including one adds its claims to the review and to the fee.

- Hosting description: not read (no GitHub origin, or the host did not answer).
- `github.com`: 32 link(s), in `.github/pull_request_template.md`, `README.md`, `docs/CLI.md`, `docs/CONTRIBUTING.md` and others
- `img.shields.io`: 19 link(s), in `README.md`, `site/index.html`
- `localhost:4567`: 13 link(s), in `docs/CLI.md`
- `chhoto.link`: 8 link(s), in `README.md`, `site/index.html`
- `hub.docker.com`: 7 link(s), in `README.md`, `docs/INSTALLATION.md`, `site/index.html`
- `fonts.googleapis.com`: 7 link(s), in `frontend/index.html`, `frontend/static/404.html`, `site/index.html`
- `codeberg.org`: 6 link(s), in `README.md`, `site/index.html`
- `sqlite.org`: 5 link(s), in `docs/INSTALLATION.md`, `site/index.html`
- `fonts.gstatic.com`: 3 link(s), in `frontend/index.html`, `frontend/static/404.html`, `site/index.html`
- `ghcr.io`: 2 link(s), in `README.md`, `site/index.html`
- `spdx.org`: 2 link(s), in `README.md`, `site/index.html`
- `en.wiktionary.org`: 2 link(s), in `README.md`, `site/index.html`
- `demo.chhoto.link`: 2 link(s), in `README.md`, `site/index.html`
- `actix.rs`: 2 link(s), in `README.md`, `site/index.html`
- `docs.rs`: 2 link(s), in `docs/INSTALLATION.md`
- and 10 more hosts with fewer links, in selection.json

## 4. Web pages: what extraction drops

- `site/index.html`: kept the title, the meta description and the main content. Dropped header, navigation and footer text: "Chhoto URL Menu Features Screenshots Quick Start About Me Links / Features Screenshots Quick Start About Me Links / Chhoto URL - Blazingly fast self-hosted URL shortener. About Me Codeberg GitHub"

## 5. Proposed lists, and how they differ from before

Every description is excluded as evidence. Which descriptions are claim sources is a choice: each one adds its claims to the review, to its length and to the fee. The estimates are of what enumeration will list, at one claim per 15 words; claims repeated between documents are counted in each and are folded after enumeration.

Claim sources:
- `README.md`: about 62 claims
- `docs/CLI.md`: about 77 claims
- `docs/INSTALLATION.md`: about 148 claims
- `docs/SECURITY.md`: about 4 claims
- `docs/TOOLS.md`: about 12 claims
- `claim_sources/site_index_html.md`: about 29 claims
- in all: about 332 claims

Evidence excludes:
- `README.md`
- `docs/CLI.md`
- `docs/INSTALLATION.md`
- `docs/SECURITY.md`
- `docs/TOOLS.md`
- `site/index.html`
- `claim_sources/`

Not named before, proposed now (claim sources): `README.md`, `docs/CLI.md`, `docs/INSTALLATION.md`, `docs/SECURITY.md`, `docs/TOOLS.md`, `claim_sources/site_index_html.md`.
Named before, not proposed now (claim sources): none. These are kept unless removed at confirmation.
Not named before, proposed now (evidence excludes): `README.md`, `docs/CLI.md`, `docs/INSTALLATION.md`, `docs/SECURITY.md`, `docs/TOOLS.md`, `site/index.html`, `claim_sources/`.
Named before, not proposed now (evidence excludes): none. These are kept unless removed at confirmation.

## 6. Confirmation

Confirmed by claude 2026-09-20, for Bruce: all six descriptions as proposed; nothing outside the target (illustrative buyer) at 2026-09-20T23-31-09Z.

Claim sources as confirmed:
- `README.md`
- `docs/CLI.md`
- `docs/INSTALLATION.md`
- `docs/SECURITY.md`
- `docs/TOOLS.md`
- `claim_sources/site_index_html.md`

Evidence excludes as confirmed:
- `README.md`
- `docs/CLI.md`
- `docs/INSTALLATION.md`
- `docs/SECURITY.md`
- `docs/TOOLS.md`
- `site/index.html`
- `claim_sources/`

Changed from the proposal: nothing.

Extraction check (words in the extracted file, less its two heading labels, against the words a reader sees in the original, counted separately):
- `claim_sources/site_index_html.md` from `site/index.html`: 423 extracted, 423 in the original
