# Standard questions, version 2

<!-- What a buyer of a small software business expects any technical review
     to answer, kept to what reading the code, build and deployment files can
     settle: nothing here needs the software run, the team interviewed or a
     legal opinion. One statement per line; its source is on the comment line
     above it. In an engagement's copy, an item that does not apply moves
     under a last heading "# Not applicable", written "statement | reason";
     the report lists it with the reason.

     Each item is drawn from a published standard and cited by requirement,
     or marked "practice" where no standard covers it. Sources: OWASP ASVS
     5.0.0 (requirement ids as published, github.com/OWASP/ASVS 5.0/en);
     OpenSSF Scorecard checks. "The materials" means the repository and any
     other files the seller supplies, such as a separate infrastructure
     repository; a statement that says "the repository" is about the
     repository alone.

     Questions for particular kinds of software (personal data, payments,
     language models and others) are in PROPERTY_QUESTIONS.md and are chosen
     per engagement by properties.py; none of them repeats an item here.
     Version 1 (2026-10-01) had 15 items; this version replaced it on
     2026-10-02. -->

# Credentials and access
<!-- ASVS 5.0 13.3.1: secrets not included in source code or build artifacts -->
No credential, API key or private key is written into the source files or the git history.
<!-- ASVS 5.0 11.4.2: passwords stored with an approved password hashing function -->
User passwords are stored only as hashes made with a password-hashing function.
<!-- ASVS 5.0 8.2.1 / 8.3.1: function-level access restricted, enforced at a trusted service layer -->
Every request handler that reads or changes stored data checks on the server that the caller is allowed to.

# Handling input
<!-- ASVS 5.0 1.2.4: parameterized queries or ORM -->
Database queries built from user input pass it as parameters, not by joining it into the query text.
<!-- ASVS 5.0 3.2.2: text rendered with safe rendering functions -->
Text a user supplies is shown in a web page only through rendering that escapes it.

# Data leaving its place
<!-- ASVS 5.0 16.2.5: sensitive data logged according to its protection level -->
No password, token or session identifier is written to the software's logs.
<!-- ASVS 5.0 14.2.3: sensitive data not sent to untrusted parties such as trackers -->
The software sends no user data to a third party (analytics, telemetry, error reporting) unless the shipped configuration enables it.

# The build and its dependencies
<!-- OpenSSF Scorecard: Pinned-Dependencies -->
Every dependency's version is fixed by a lockfile in the repository.
<!-- OpenSSF Scorecard: Dangerous-Workflow and Token-Permissions -->
The repository's CI workflows give their tokens read-only permissions and do not run untrusted input as code.
<!-- OpenSSF Scorecard: CI-Tests -->
Automated tests run in a continuous-integration workflow in the repository.
<!-- practice: licence obligations are a buyer's question; no security standard covers them -->
Every dependency's licence can be read from the materials, and none is from the GPL, AGPL or SSPL families.
<!-- OpenSSF Scorecard: Dependency-Update-Tool. No standard sets how recently dependencies must have been updated, and the auditor does not read git history, so the question asks for the tool that keeps them updated. -->
A dependency update tool (Dependabot or Renovate) is configured in the repository.

# Running it after the sale
<!-- practice: the questions in this section ask whether someone other than the seller can run the software from what is handed over -->
The software can be built from the materials without a private registry or any file the materials do not contain.
<!-- practice -->
A new server can be set up to run the software by following scripts, container or infrastructure files, or written instructions in the materials.
<!-- practice -->
Every task the software needs run on a timer (billing, invoicing, reports, clean-up, sending email) is scheduled by a definition in the materials, such as a cron file, a worker schedule or a CI schedule.
<!-- practice; carried from version 1 -->
Changes to the database schema are made by versioned migrations kept in the materials.
<!-- practice -->
The stored data can be backed up or exported by a mechanism in the code or the deployment files in the materials.
