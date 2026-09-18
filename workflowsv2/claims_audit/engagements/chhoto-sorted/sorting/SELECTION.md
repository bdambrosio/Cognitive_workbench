# Selection record: `chhoto-sorted`

Status: **CONFIRMED 2026-09-17T18-17-43Z by claude-test**. Prepared by `workflowsv2/materials_sorting/runner.py` with accounts/fireworks/models/glm-5p3-flash; the procedure is `docs/claim-source-selection.md`.

## 1. What intake and the engagement said

The brief, whole:

```
(no brief)
```

`claim_sources:` in engagement.yaml before this step: `README.md`

`evidence_excludes:` before this step: `README.md` (defaulted to the claim sources)

## 2. Files that hold prose, and their kind

Target at commit 6a1b51cdeafdaadd46560e30a7c1a36bd1f08388. 21 files read. Passed over without reading, by type: .css 2, .ico 2, .js 2, .lock 1, .png 5, .rs 20, .svg 2, .svgz 2, .toml 2, .webp 4, .yaml 3, .yml 10.

| File | Words | Linked from | Kind | Claim source | Claims, estimated | Excluded as evidence | Reason |
|---|---|---|---|---|---|---|---|
| `.gitattributes` | 45 |  | instrument |  |  |  | The file is a git configuration that itself establishes how repository files are handled — it marks images like 'screenshot-desktop.webp' and 'favicon.svgz' as stored in Git LFS with 'filter=lfs'. Like a CI workflow or container file, a claim such as 'assets are stored in LFS' is settled by reading this file, and nothing behind it could show otherwise. |
| `.github/ISSUE_TEMPLATE/bug_report.md` | 154 |  | neither |  |  |  | This is a GitHub issue template, a form for reporting bugs: it prompts the filer for 'a clear and concise description of what the bug is' and reproduction steps. It does not describe the software, settle any claim, or contain product-facing text, so it is neither. |
| `.github/ISSUE_TEMPLATE/feature_request.md` | 91 |  | neither |  |  |  | This is a feature request form: lines 9-19 are prompts like "Describe the solution you'd like" telling a would-be filer how to fill it in. It is a form for reporting requests, not the seller speaking about the software, so it is neither a claim source nor a fact a claim would rest on. |
| `.github/pull_request_template.md` | 60 |  | neither |  |  |  | This is a blank form that a contributor fills in when opening a pull request ("Please include a summary of the change", checkbox list), not text in which the seller describes the software or sets any term. Like a bug-report form, it is scaffolding for the development process rather than a claim, a fact a claim would be about, product text, or the seller speaking about the software. |
| `.gitignore` | 24 |  | instrument |  |  |  | The file is a .gitignore configuration: it does not describe the software but is itself the fact that a claim about version-control configuration would be settled by — its lines ("backend/target", ".env", "*.sqlite") are directives, not statements for a reader deciding about the product. |
| `LICENSE` | 169 |  | instrument |  |  |  | The file is the MIT licence text itself — a claim such as 'the project is MIT licensed' is settled by reading this file, and nothing behind it could show otherwise. It states the grant ('Permission is hereby granted, free of charge...') and the warranty disclaimer ('THE SOFTWARE IS PROVIDED "AS IS"'), making it the fact itself rather than a report of a fact elsewhere. |
| `Makefile` | 472 |  | instrument |  |  |  | This is the project's build automation: recipes that define how the software is built, tested, containerised, version-tagged and deployed (e.g. `build`, `test`, `tag`, `deploy: minify`). Like a CI workflow, it is itself the fact that a claim about how the project is built and published would be about, rather than a description reporting facts that lie elsewhere. |
| `README.md` | 932 |  | description | `README.md` | about 62 | yes | This is a README in which the seller describes its software to prospective users: 'A simple selfhosted URL shortener with no unnecessary features', followed by a Features list, screenshots, and pointers to installation and CLI guides. The SPDX licence header lines and the badge block at the top are small mark-up, not parts of a different kind that make the file mixed. |
| `deploy/Containerfile` | 106 |  | instrument |  |  |  | This is a build recipe for the shipped container, not a report about it: a claim such as "the image is built from scratch with the Rust binary and frontend only" is settled by these instructions themselves, and nothing behind the file could show otherwise. SPDX lines 1–2 state licensing terms, which likewise are the fact rather than a description of it. |
| `deploy/Containerfile.alpine` | 55 |  | instrument |  |  |  | This is a container build recipe: it copies prebuilt binaries for several architectures into alpine base images, installs tzdata, and sets the entrypoint (lines 4–21). It is itself the fact that a claim about how the software is packaged or deployed would rest on, rather than a report of a fact lying elsewhere; the SPDX copyright and licence lines at the top do not make it mixed. |
| `deploy/Containerfile.debug` | 27 |  | instrument |  |  |  | The file is a container build recipe: it copies a debug binary of the software into an alpine image ("COPY ./backend/target/x86_64-unknown-linux-musl/debug/chhoto-url"), installs tzdata, and sets the entrypoint. It is itself the fact that claims about the debug deployment container would be about, such as a claim that a debug image builds on alpine, so it is an instrument, not a description of the software. |
| `deploy/Containerfile.scratch` | 46 |  | instrument |  |  |  | A container build file that defines how the software is packaged: it stages prebuilt binaries from scratch images, copies the frontend, and sets ENTRYPOINT ["/chhoto-url"]. It is itself the artifact a deployment claim would be about — a claim like "the project ships a minimal container image" is settled by reading this file, not by any prose elsewhere. |
| `deploy/chhoto-url.container` | 130 | docs/INSTALLATION.md | instrument |  |  |  | This is a quadlet container definition file: it is itself the deployment configuration a claim about how the software is deployed or configured would be settled by, with settings like `Image=docker.io/sintan1729/chhoto-url:latest` and environment variables such as `CHHOTO_PASSWORD`. Its comments point readers elsewhere ("Take a look at README for the explanation of the configs") rather than describing the software itself. |
| `docs/CLI.md` | 1155 | README.md | description | `docs/CLI.md` | about 77 | yes | This is a usage guide written by the maintainer for people using the software: it says "There's an official CLI app for Linux" and "The instructions below describe how to use all the features using `curl`", then documents each API endpoint's requests, responses and authentication. It reports how the software behaves (e.g. "These routes are accessible without any authentication"), so its claims are to be tested against the code rather than settled by this file. |
| `docs/CONTRIBUTING.md` | 227 |  | instrument |  |  |  | The whole file sets the terms under which contributions to Chhoto URL are accepted — rules 1-7 ('No AI contribution is allowed...', 'Everything must remain backwards compatible') are themselves the conditions a contribution must satisfy, the very facts claims about contribution policy would be about, and no text behind them could show otherwise. Nothing in the file describes what the software is or does, so it is not mixed. |
| `docs/INSTALLATION.md` | 2220 | README.md, docs/CLI.md | description | `docs/INSTALLATION.md` | about 148 | yes | The file is an installation and configuration guide for Chhoto URL: it walks the reader through deploying the software ("Using docker compose (Recommended method)", "Building and running with docker") and documents every environment variable such as CHHOTO_DB_URL and CHHOTO_PASSWORD. This is the seller describing how to install, configure and use the software, written for someone deploying it, so it is a description; the SPDX header lines are a licence notice too small to make the file mixed. |
| `docs/SECURITY.md` | 56 |  | description | `docs/SECURITY.md` | about 4 | yes | The file is a security policy telling readers how vulnerability reports are handled ('How to Report a Vulnerability', 'Please do not report security vulnerabilities through public GitHub issues'), which the guide names as a description; it sets no terms or facts that a claim could be settled by, since the contact channels could be checked no further than the policy itself states them. |
| `docs/TOOLS.md` | 178 | README.md | description | `docs/TOOLS.md` | about 12 | yes | The file is the seller's own documentation page, 'Software related to Chhoto URL', telling readers what tools exist for the software: an 'Official CLI application' maintained by the seller and third-party options such as a browser extension and a NixOS package. It speaks about the software's ecosystem to a reader learning to use it, so it is a description; none of its few lines are of a different kind. |
| `frontend/index.html` | 70 |  | product_text |  |  |  | The file is the running link-shortener's own interface: it shows a links table that is 'Loading links table...', column headers like '/ # / Short URL / Long URL / Hits /' and form fields 'Short:', 'Long:', 'Hits:' — words the product displays to its own users. The title and one-line description in lines 1–3 are page metadata of this screen, not a separate passage describing the product to outsiders, so the file is not mixed. |
| `frontend/static/404.html` | 26 |  | product_text |  |  |  | The file is the text of a 404 error page — 'Error 404!' with a short verse — that the running software displays to users who reach a missing page, which makes it words the product shows on screen. |
| `site/index.html` | 440 |  | description, mixed | `claim_sources/site_index_html.md` | about 29 | yes | This is a landing page presenting the product to people who do not yet use it: it introduces Chhoto URL as a 'lightweight URL shortener written in Rust', lists features like 'Link expiry' and 'Hit counting', and gives a getting-started guide for deployment. The About me section (lines 71-75) is a personal biography rather than text about the software, so it is listed as a separate part. |

### Mixed files

Parts whose kind differs from the file's. Where a file kept as evidence has a part that is a description, the brief should name that part as the seller's own statement. Decide each one.
- `site/index.html` lines 71-75, neither: A personal introduction to the maintainer, Sayantan Santra, describing his academic background and programming hobbies rather than the software.

## 3. Outside the target: the client's choice

Nothing here was fetched. Each is the seller's text a buyer could have read; including one adds its claims to the review and to the fee.

- Hosting description of `SinTan1729/chhoto-url`: "A simple, blazingly fast, selfhosted URL shortener with no unnecessary features; written in Rust"; homepage https://chhoto.link; topics: actix-web, compose, container, docker, link-shortener, podman, quadlets, rust, self-hosted, shortener, url-shortener, webapp; wiki not enabled.
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

Not named before, proposed now (claim sources): `docs/CLI.md`, `docs/INSTALLATION.md`, `docs/SECURITY.md`, `docs/TOOLS.md`, `claim_sources/site_index_html.md`.
Named before, not proposed now (claim sources): none. These are kept unless removed at confirmation.
Not named before, proposed now (evidence excludes): `docs/CLI.md`, `docs/INSTALLATION.md`, `docs/SECURITY.md`, `docs/TOOLS.md`, `site/index.html`, `claim_sources/`.
Named before, not proposed now (evidence excludes): none. These are kept unless removed at confirmation.

## 6. Confirmation

Confirmed by claude-test at 2026-09-17T18-17-43Z.

Claim sources as confirmed:
- `README.md`
- `docs/CLI.md`
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

Changed from the proposal: claim source removed: docs/INSTALLATION.md.

Extraction check (words in the extracted file, less its two heading labels, against the words a reader sees in the original, counted separately):
- `claim_sources/site_index_html.md` from `site/index.html`: 423 extracted, 423 in the original
