# Selection record: `synthetic-hospital`

Status: **CONFIRMED 2026-09-25T18-56-13Z by claude 2026-09-25, for Bruce: full end-to-end run, the model's proposals unchanged**. Prepared by `workflowsv2/materials_sorting/runner.py` with accounts/fireworks/models/glm-5p3-flash; the procedure is `docs/claim-source-selection.md`.

## 1. What intake and the engagement said

The brief, whole:

```
(no brief)
```

`claim_sources:` in engagement.yaml before this step: none

`evidence_excludes:` before this step: none (defaulted to the claim sources)

## 2. Files that hold prose, and their kind

Target at commit 77cc57e9c595ce731bbdde4fabdcbd25a8458ae2. 20 files read. Passed over without reading, by type: .db 2, .ini 3, .json 1, .py 189, .sh 5, .sql 1, .svg 1, .yaml 11, .yml 2.

| File | Words | Linked from | Kind | Claim source | Claims, estimated | Excluded as evidence | Reason |
|---|---|---|---|---|---|---|---|
| `.dockerignore` | 53 |  | instrument |  |  |  | A `.dockerignore` is part of the container build configuration: it is itself the fact a claim about what goes into the built image would be about, like a container or compose file, not prose reporting facts that lie elsewhere. The opening comment lines ("Data is mounted at run time by the default target") merely explain the patterns and do not make the file a seller's description of the software. |
| `.gitignore` | 80 |  | instrument |  |  |  | The file is a .gitignore: version control configuration that is itself the fact of what the repository excludes, such as the environment files listed under "Secrets / environment". A claim about what is or is not committed would be settled by this file alone, with nothing behind it to consult, which is the test for an instrument. |
| `CITATION.cff` | 106 | README.md | instrument |  |  |  | This is a machine-readable citation record (Citation File Format) that states the software's title, version "1.3", release date 2026-09-25, and license pointer itself; it is the fact a claim would be about, not a report pointing elsewhere, like a package manifest. It does not describe what the software does or how to use it, so it is not a seller's description. |
| `DATA_CARD.md` | 1118 | README.md | description | `DATA_CARD.md` | about 75 | yes | This is a data card that describes the released dataset to a reader deciding how to use it: it summarises contents ('1,268 synthetic patients and 5,602 clinical encounters'), documents files, schema, provenance, and limitations. It reports facts about the data rather than being the data or the licence itself — the licensing section only characterises the terms ('carries no copyright concern') and points to requirements, so it is description, not instrument. |
| `Dockerfile` | 438 |  | instrument, mixed |  |  |  | The file is a container build recipe: its FROM, COPY, RUN and ENV instructions (lines 15-95) are themselves the fact a claim about the shipped images would be settled by, e.g. that the with-data target bakes in benchmark_v1.3.db (line 67). The comment block at lines 1-13 instead describes the product and its build targets for a reader deciding how to deploy it, which is the seller speaking. |
| `LICENSE` | 168 | README.md | instrument |  |  |  | The file is the MIT licence text itself — it grants rights to 'use, copy, modify, merge, publish, distribute, sublicense, and/or sell' the software. A claim that the project is MIT-licensed is settled by reading this file, and nothing behind it could show otherwise, so it is itself the fact a licence claim would be about. |
| `README.md` | 2387 |  | description | `README.md` | about 159 | yes | A README that describes what the software is and does — 'Converts USMLE-style medical education source content into a ground-truth benchmark database … served through an Epic-faithful EHR simulation platform' — and tells a reader how to install, load data, and run evaluations. It is the seller's description of the product for people deciding about or using it, so its claims are claim sources and it cannot itself serve as evidence for them. |
| `apptainer/README.md` | 340 |  | description | `apptainer/README.md` | about 23 | yes | A build-and-run guide written for people preparing to run the software: it describes how to build the image ('apptainer build synthetic_hospital.sif ...'), how to run and configure it, and what the container carries ('the simulator, the benchmark database, PostgreSQL and Redis'). It reports facts about the software for a reader deciding how to use it, so its claims are to be tested and it cannot serve as evidence for them. |
| `apptainer/synthetic_hospital.def` | 181 |  | instrument, mixed |  |  |  | The file is an Apptainer definition that builds the container itself (Bootstrap: docker, %environment, %runscript); a claim about how the container is built or what it runs is settled by this file, making it an instrument. Lines 30-37 are a %help block the software shows its own users, so that part is product_text. |
| `apptainer/synthetic_hospital.local.def` | 178 |  | instrument |  |  |  | This is an Apptainer container definition file; it is the build recipe itself, so a claim such as "the software ships as a container" is settled by this file and nothing behind it could show otherwise. Its embedded %help text is metadata carried in the definition, not prose about the product in another kind of document. |
| `epic_sim/alembic/script.py.mako` | 75 |  | neither |  |  |  | This is Alembic's standard Mako template for generating new migration files, consisting of placeholder variables like "${message}" and "${up_revision}" with no seller prose. It is scaffolding with no text speaking about the software, so it is neither a claim source nor excluded evidence. |
| `etl/deck_profiles/README.md` | 148 |  | description | `etl/deck_profiles/README.md` | about 10 | yes | The file explains how to configure the pipeline: it tells the reader to add their own deck by copying a profile and editing it, and documents the meaning of each YAML field (e.g. 'Which extractor Stage 3/4 uses'). That is seller prose about how to use the software, written for someone configuring it, not a configuration file itself or text the product shows its users. |
| `etl/pdf_profiles/README.md` | 140 |  | description | `etl/pdf_profiles/README.md` | about 9 | yes | This README explains how the PDF profile feature works and how to configure it: "Stage 1d reads these instead of hardcoding filenames. To add your own document, copy a file below and edit it", and it documents each config field and layout. It is written for a user of the software learning to configure it, so it is a description, and its claims (e.g. about which parser each layout uses) are to be tested against the code rather than settled by this file. |
| `etl/stages/EXTENDING.md` | 738 |  | description | `etl/stages/EXTENDING.md` | about 49 | yes | This is an extension guide for the ETL pipeline: it tells a developer how to add new sources ('To run the pipeline on your own material you add a profile — no code changes') and walks through profile kinds, parser keys, and a checklist. It describes how the software is configured and used, so claims it makes (e.g. that stages are 'source-agnostic' and data-driven) are to be tested against the code itself, which means the file is a description, not an instrument. The YAML blocks are illustrative examples inside the guide, not a part of a different kind. |
| `harbor/README.md` | 408 |  | description | `harbor/README.md` | about 27 | yes | A README that explains what the exported Harbor tasks are and how to run them — it walks through the task directory layout and gives export and run commands ('uvx harbor run -p harbor_tasks -a oracle'), written for someone deciding whether to use or learning to use the export. It describes the software throughout rather than itself constituting a fact a claim would rest on. |
| `harbor/templates/Dockerfile.main` | 70 |  | instrument |  |  |  | This is a Dockerfile: it is the build recipe itself, not a report of a fact lying elsewhere — a claim such as 'the agent image is built FROM python:3.12-slim and installs curl and jq' is settled by reading these lines, and nothing behind the file could show otherwise. Its opening comments ('this image only needs a shell, curl/jq for ad-hoc calls') are brief incidental notes, not a separate descriptive part. |
| `patient_profiles.jsonl` | 1682774 |  | neither |  |  |  | The file is generated sample data, not the seller speaking about the software: each record carries "generation_seed": 6235 and a "generation_method" of "template" or "hybrid", and the content is synthetic patient profiles and encounter notes (fictional patients such as "Dr. Elena Rodriguez" with invented histories). It is sample data for exercising the software, so no claim source or evidence exclusion applies. |
| `requirements-core.txt` | 81 |  | instrument |  |  |  | The file is a pinned dependency manifest listing package versions such as "fastapi==0.135.1" and "sqlalchemy==2.0.47"; it is itself the fact that claims like "the product depends on these packages at these versions" would be about, and no prose elsewhere is needed to read it. |
| `requirements.txt` | 105 |  | instrument |  |  |  | The file is a package manifest pinning dependency versions (e.g. "torch==2.10.0", "fastapi==0.135.1"); it is itself the fact that a claim like "the project depends on these libraries at these versions" would be about, and reading it settles such a claim. |
| `scripts/README.md` | 183 |  | description | `scripts/README.md` | about 12 | yes | The file is documentation telling a reader how to run the scripts: it says they 'query the full benchmark database', explains that the shipped `patient_profiles.db` is a 'stripped two-table subset' and that they must first 'rebuild the full benchmark with the ETL pipeline'. That is the seller explaining what the materials contain and how to use them, not the file itself being the fact a claim rests on, so it is a description throughout with no distinct-kind part of a few lines or more. |

### Mixed files

Parts whose kind differs from the file's. Where a file kept as evidence has a part that is a description, the brief should name that part as the seller's own statement. Decide each one.
- `Dockerfile` lines 1-13, description: A header comment describes the product as an EHR simulator with an evaluation harness and RL environment, and explains how to build, pull and run its images.
- `apptainer/synthetic_hospital.def` lines 30-37, product_text: Help text the container prints to its own users, describing what it runs and how to start and use it.

## 3. Outside the target: the client's choice

Nothing here was fetched. Each is the seller's text a buyer could have read; including one adds its claims to the review and to the fee.

- Hosting description of `sparkcpark/synthetic_hospital`: "None"; homepage none; topics: none; wiki not enabled.
- `localhost:8000`: 7 link(s), in `README.md`, `apptainer/README.md`
- `harborframework.com`: 2 link(s), in `README.md`, `harbor/README.md`
- `www.nlm.nih.gov`: 1 link(s), in `README.md`
- `loinc.org`: 1 link(s), in `README.md`
- `www.cms.gov`: 1 link(s), in `README.md`
- `localhost:8080`: 1 link(s), in `README.md`

## 4. Web pages: what extraction drops

No HTML file is proposed as a claim source.

## 5. Proposed lists, and how they differ from before

Every description is excluded as evidence. Which descriptions are claim sources is a choice: each one adds its claims to the review, to its length and to the fee. The estimates are of what enumeration will list, at one claim per 15 words; claims repeated between documents are counted in each and are folded after enumeration.

Claim sources:
- `DATA_CARD.md`: about 75 claims
- `README.md`: about 159 claims
- `apptainer/README.md`: about 23 claims
- `etl/deck_profiles/README.md`: about 10 claims
- `etl/pdf_profiles/README.md`: about 9 claims
- `etl/stages/EXTENDING.md`: about 49 claims
- `harbor/README.md`: about 27 claims
- `scripts/README.md`: about 12 claims
- in all: about 364 claims

Evidence excludes:
- `DATA_CARD.md`
- `README.md`
- `apptainer/README.md`
- `etl/deck_profiles/README.md`
- `etl/pdf_profiles/README.md`
- `etl/stages/EXTENDING.md`
- `harbor/README.md`
- `scripts/README.md`
- `claim_sources/`

Not named before, proposed now (claim sources): `DATA_CARD.md`, `README.md`, `apptainer/README.md`, `etl/deck_profiles/README.md`, `etl/pdf_profiles/README.md`, `etl/stages/EXTENDING.md`, `harbor/README.md`, `scripts/README.md`.
Named before, not proposed now (claim sources): none. These are kept unless removed at confirmation.
Not named before, proposed now (evidence excludes): `DATA_CARD.md`, `README.md`, `apptainer/README.md`, `etl/deck_profiles/README.md`, `etl/pdf_profiles/README.md`, `etl/stages/EXTENDING.md`, `harbor/README.md`, `scripts/README.md`, `claim_sources/`.
Named before, not proposed now (evidence excludes): none. These are kept unless removed at confirmation.

## 6. Confirmation

Confirmed by claude 2026-09-25, for Bruce: full end-to-end run, the model's proposals unchanged at 2026-09-25T18-56-13Z.

Claim sources as confirmed:
- `DATA_CARD.md`
- `README.md`
- `apptainer/README.md`
- `etl/deck_profiles/README.md`
- `etl/pdf_profiles/README.md`
- `etl/stages/EXTENDING.md`
- `harbor/README.md`
- `scripts/README.md`

Evidence excludes as confirmed:
- `DATA_CARD.md`
- `README.md`
- `apptainer/README.md`
- `etl/deck_profiles/README.md`
- `etl/pdf_profiles/README.md`
- `etl/stages/EXTENDING.md`
- `harbor/README.md`
- `scripts/README.md`
- `claim_sources/`

Changed from the proposal: nothing.

Extraction check (words in the extracted file, less its two heading labels, against the words a reader sees in the original, counted separately):
- nothing needed extracting
