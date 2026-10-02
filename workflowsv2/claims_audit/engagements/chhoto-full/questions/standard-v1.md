# Standard questions, version 1

<!-- What a buyer of a small software business expects any technical review
     to answer, kept to what reading the code, build and deployment files can
     settle: nothing here needs the software run, the team interviewed or a
     legal opinion. One statement per line. In an engagement's copy, an item
     that does not apply moves under a last heading "# Not applicable",
     written "statement | reason"; the report lists it with the reason. -->

# Credentials and access
No credential, API key or private key is written into the source files or the git history.
Every request handler that changes stored data requires an authenticated caller.

# Handling input
Database queries built from user input pass it as parameters, not by joining it into the query text.
Text a user supplies is escaped by default when the software shows it in a web page.

# Personal data
No password, token, email address or IP address is written to the software's logs.
The software sends no data to a third party (analytics, telemetry, error reporting) unless the shipped configuration enables it.

# Dependencies and licences
Every dependency's version is fixed by a lockfile in the repository.
No dependency declares a licence from the GPL, AGPL or SSPL families.
The repository carries a licence file that covers its code.
Code copied into the repository from another project names its origin and licence.

# Building and running
The software can be built from the repository without a private registry or any file the repository does not contain.
Automated tests exist for the software's core function and run in a continuous-integration workflow in the repository.
Changes to the database schema are made by versioned migrations kept in the repository.
The stored data can be exported or backed up by a mechanism in the code or the deployment files.

# Not applicable
Passwords of the software's users are stored only as salted hashes made with a password-hashing function. | The software has no user accounts: one admin password, set by the operator.
