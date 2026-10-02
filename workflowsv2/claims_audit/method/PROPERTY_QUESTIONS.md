# Questions by software property, version 0 (draft 2026-10-02)

<!-- Statements for each property in PROPERTIES.md. An engagement
     uses the sections for the properties it identified, together with the
     baseline list (draft baseline v2, logs/BASELINE_QUESTIONS.v2.draft.md), which applies to every engagement.
     Each statement is testable by reading the code and the files supplied
     with it; questions only the seller can answer are not in this file. One
     statement per line; its source is on the comment line above it.

     Citation marks: [checked] means the cited text was read on 2026-10-02.
     A note after the mark says where the statement asks more than the
     source requires, or where only a secondary source or a requirement's
     title could be read (IEC 62443, SOC 2, CIS). "Practice, because X"
     means X is why a buyer cares but does not require the mechanism the
     statement tests. All 177 statements were kept by the practice on
     2026-10-02.

     Duplicates: each statement appears under one property only. A statement
     the baseline list already makes is not repeated here; the section notes
     which baseline item covers it. Where a statement could belong to two
     properties, the section that owns it is the one whose software needs it
     most, and the other section names it. -->

# A1. Holds personal data about people other than its operator
<!-- covered by baseline: no user data sent to third parties unless configuration enables it; no password, token or session id in logs -->
<!-- practice, because GDPR Art. 17 gives a right to erasure [checked: the right exists; no software function is required] -->
A person's personal data can be deleted on request by a function in the software.
<!-- practice, because GDPR Art. 17 [checked: the article does not name stores; complete erasure is an inference] -->
Deleting a person removes their data from every table, file store and search index that holds it, not only from the main record.
<!-- practice, because GDPR Art. 15(3) (copy in a commonly used electronic form) and Art. 20(1) (machine-readable, where processing rests on consent or contract) [checked] -->
A person's personal data can be exported on request, in a machine-readable format, by a function in the software.
<!-- practice, because GDPR Art. 16 gives a right to rectification [checked] -->
A person's personal data can be corrected by a function in the software.
<!-- ASVS 5.0 14.2.7 [checked]; practice, because GDPR Art. 5(1)(e) storage limitation [checked: a principle, not a mechanism] -->
Personal data is deleted automatically after a retention period the operator can set.
<!-- practice, because GDPR Art. 32(1)(a) names encryption as an example measure, 'as appropriate' [checked] -->
Personal data stored by the software is encrypted at rest, by the application or by a setting in the deployment files.
<!-- ASVS 5.0 14.3.3 [checked] -->
The browser front end keeps no personal data in local storage, session storage or IndexedDB.
<!-- ASVS 5.0 14.3.1 [checked] -->
Personal data shown in the browser is cleared from client storage when the user logs out or the session ends.
<!-- ASVS 5.0 16.2.5 [checked]; practice, because GDPR Art. 5(1)(c) and 32 [checked: neither names logs] -->
Personal data (names, email addresses, phone numbers, addresses, message text) is not written to the software's logs.

# A2. Holds health or other special-category data
<!-- HIPAA 45 CFR 164.312(b), audit controls, a standard [checked: requires recording activity; every read and change with user and time is stricter] -->
Every read and every change of a health record is logged with the user and the time.
<!-- HIPAA 45 CFR 164.312(a)(2)(iv), encryption, addressable [checked: 'at rest' and 'by the application' are stricter]; GDPR Art. 32(1)(a) [checked] -->
Health data is encrypted at rest by the application, not only by the disk or the database server.
<!-- HIPAA 45 CFR 164.312(a)(2)(iii), automatic logoff, addressable [checked]; ASVS 5.0 7.3.1 [checked] -->
A session that can read health data ends after a fixed period of inactivity.
<!-- practice, because HIPAA 45 CFR 164.312(c)(1) requires protection against improper alteration [checked: version history is one way to meet it, not a requirement] -->
A change to a health record keeps the previous value and who changed it.
<!-- HIPAA 45 CFR 164.312(e)(1), a standard; encryption in transit 164.312(e)(2)(ii) is addressable [checked: 'only over encrypted connections' is stricter]; ASVS 5.0 12.3.1 [checked] -->
Health data travels only over encrypted connections, including between the software's own services.

# A3. Stores files that users upload
<!-- ASVS 5.0 5.2.1 [checked] -->
Every upload feature enforces a maximum file size.
<!-- ASVS 5.0 5.2.2 [checked] -->
Every upload feature checks that the file's extension is an expected one and that its content matches that type.
<!-- ASVS 5.0 5.2.3 [checked] -->
Compressed files are checked against a maximum unpacked size and file count before they are unpacked.
<!-- ASVS 5.0 5.3.1 [checked] -->
Uploaded files are stored where the server cannot execute them as code.
<!-- ASVS 5.0 5.3.2 [checked] -->
File paths for stored uploads are generated by the software, not built from the filename the user supplied.
<!-- ASVS 5.0 3.2.1 and 5.4.1 [checked] -->
Uploaded files are served for download with a Content-Disposition header or from a separate domain, so a browser does not render them inside the application.
<!-- ASVS 5.0 8.2.2 [checked] -->
An uploaded file can be read only by users allowed to see the record it belongs to; it is not in a publicly readable bucket or folder.
<!-- ASVS 5.0 5.4.3 [checked] -->
Uploaded files are scanned for malware before they are served to other users.

# A4. Keeps financial or accounting records the business relies on
<!-- practice -->
Money amounts are stored and calculated as integers of the smallest unit or as fixed-point decimals, never as floating-point numbers.
<!-- practice -->
An issued invoice or ledger entry is never edited in place; a correction is recorded as a new entry that refers to the original.
<!-- ASVS 5.0 2.3.3 [checked] -->
An operation that changes several financial records changes them in one database transaction.
<!-- practice -->
Every change to a financial record records who made it and when.

# B1. Takes payments through a payment processor
<!-- covered elsewhere: webhook signature and repeated deliveries are under D3 -->
<!-- PCI DSS v4.0.1 SAQ A eligibility: no account data on merchant systems [checked: necessary, not sufficient; eligibility is the merchant's, not the software's] -->
Card numbers never reach the software's servers; cards are entered only in the processor's hosted page or embedded fields.
<!-- practice -->
An order or subscription is marked paid only on confirmation from the processor, not on the customer's browser returning from checkout.
<!-- ASVS 5.0 2.2.2 and 8.3.1 [checked] -->
The amount charged is computed on the server from the server's prices, not taken from the request.
<!-- PCI DSS v4.0.1 req. 6.4.3, payment page scripts inventoried, authorised and integrity-checked [checked: 'from a fixed source' is not required; SAQ A r1 replaces 6.4.3 with a script eligibility criterion] -->
Every script loaded on a page that hosts or embeds the payment form is listed in the materials and loaded from a fixed source.

# B2. Handles or stores card data itself
<!-- PCI DSS v4.0.1 req. 3.3.1.2 [checked] -->
The card security code is never stored after authorisation.
<!-- PCI DSS v4.0.1 req. 3.5.1 [checked: also allows hashes and truncation] -->
A stored card number is encrypted or replaced by a token wherever it is stored.
<!-- PCI DSS v4.0.1 req. 3.4.1 [checked: the limit is the BIN and last four digits] -->
A card number is shown masked, with no more than the first six and last four digits visible.
<!-- PCI DSS v4.0.1 req. 3.5.1 and 3.2.1 [checked: PAN in logs must be unreadable; 'never written' is stricter]; ASVS 5.0 16.2.5 [checked] -->
Card numbers are never written to logs.
<!-- PCI DSS v4.0.1 req. 4.2.1 [checked: open, public networks only; 'only over encrypted connections' is stricter] -->
Card data is sent only over encrypted connections.
<!-- PCI DSS v4.0.1 req. 3.4.1 and 7.2.6 [checked] -->
Only roles that need full card numbers can retrieve them.

# B3. Bills its own customers on a schedule
<!-- covered by baseline: billing jobs are scheduled by a definition in the materials -->
<!-- practice -->
A billing run can be run again for the same period without charging any customer twice.
<!-- practice -->
A failed charge moves the account to a defined state (retry, notice to the customer, suspension) by code in the software.
<!-- practice -->
The software compares its own billing records with the processor's records and reports differences.
<!-- practice -->
A customer whose subscription has ended loses access to paid features by code in the software.

# C1. Has user accounts with a login of its own
<!-- covered by baseline: passwords stored only as hashes made with a password-hashing function -->
<!-- ASVS 5.0 6.2.1 [checked] -->
Passwords shorter than 8 characters are rejected.
<!-- ASVS 5.0 6.2.4 [checked] -->
New passwords are checked against a list of common or breached passwords.
<!-- ASVS 5.0 6.3.1 [checked] -->
Repeated failed logins are limited by rate limiting or lockout.
<!-- ASVS 5.0 6.3.2 [checked] -->
The software ships no default user account such as "admin" with a fixed password.
<!-- ASVS 5.0 6.3.3 [checked] -->
Users can turn on a second factor for login.
<!-- ASVS 5.0 6.4.3 and 6.5.5 [checked] -->
Password-reset links expire after a set time and work only once.
<!-- ASVS 5.0 6.3.8 [checked] -->
Login and password-reset responses do not reveal whether an account exists.
<!-- ASVS 5.0 7.2.4 [checked] -->
A new session token is issued at login.
<!-- ASVS 5.0 7.4.1 [checked] -->
Logging out ends the session on the server, so the old token no longer works.
<!-- ASVS 5.0 7.4.2 [checked] -->
Disabling or deleting an account ends that account's active sessions.
<!-- ASVS 5.0 3.3.1 and 3.3.4 [checked] -->
Session cookies are set with the Secure and HttpOnly attributes.
<!-- ASVS 5.0 7.5.1 [checked] -->
Changing the account's email address, password or second factor requires the user to authenticate again.

# C2. Lets users log in through another identity provider
<!-- ASVS 5.0 10.2.1 [checked] -->
The login flow with the outside provider is protected against forged requests by a state value or PKCE.
<!-- ASVS 5.0 6.8.2, 10.5.1 and 10.5.4 [checked] -->
The provider's ID token is accepted only after its signature, audience and nonce are checked.
<!-- ASVS 5.0 10.5.2 and 6.8.1 [checked] -->
A user is identified by the provider's subject identifier, not by email address, so an account at one provider cannot take over an account made through another.
<!-- ASVS 5.0 10.2.3 [checked] -->
The login requests only the scopes the software uses.

# C3. Serves several customer organisations from one installation
<!-- covered by baseline: every request handler checks on the server that the caller is allowed to -->
<!-- ASVS 5.0 8.4.1 [checked] -->
Each customer organisation's data can be read and changed only by that organisation's users.
<!-- ASVS 5.0 8.4.1 and 8.3.1 [checked] -->
The organisation a request acts on is taken from the authenticated user, never from a value the client sends.
<!-- ASVS 5.0 8.4.1 [checked] -->
Background jobs and caches keep each organisation's data apart: every job runs for one organisation and every cache key includes the organisation.
<!-- ASVS 5.0 8.4.1 and 8.2.2 [checked] -->
Files stored for one organisation cannot be read through another organisation's account.
<!-- ASVS 5.0 8.2.1 [checked] -->
Functions that act across organisations are available only to the operator's own staff accounts.
<!-- GDPR Art. 28(3)(g) [checked: delete or return at the controller's choice, as a contract term] -->
An organisation's data can be exported and deleted as a whole when it leaves the service.

# C4. Issues its own tokens or API keys to callers
<!-- ASVS 5.0 9.1.1 and 9.1.2 [checked] -->
Tokens the software issues are verified by signature against a fixed list of allowed algorithms.
<!-- ASVS 5.0 9.2.1 [checked] -->
A token is rejected after its expiry time.
<!-- ASVS 5.0 9.2.2 and 9.2.3 [checked] -->
A token is accepted only for the purpose and service it was issued for.
<!-- ASVS 5.0 7.2.3 and 11.5.1 [checked] -->
API keys and reference tokens are generated by a cryptographically secure random generator with at least 128 bits of randomness.
<!-- practice -->
API keys are stored only as hashes.
<!-- practice -->
A user can revoke an API key or token, after which it stops working.

# D1. Has a web front end that runs in a browser
<!-- covered by baseline: text a user supplies is shown only through rendering that escapes it -->
<!-- ASVS 5.0 3.5.1 and 3.5.3 [checked] -->
Requests that change data use POST, PUT, PATCH or DELETE and are protected against cross-site request forgery.
<!-- ASVS 5.0 3.4.1 [checked] -->
Responses carry a Strict-Transport-Security header.
<!-- ASVS 5.0 3.4.3 [checked] -->
Responses carry a Content-Security-Policy that limits where scripts load from.
<!-- ASVS 5.0 3.4.6 [checked] -->
Pages set frame-ancestors so other sites cannot embed them, except where embedding is intended.
<!-- ASVS 5.0 3.4.4 [checked] -->
Responses carry X-Content-Type-Options: nosniff.
<!-- ASVS 5.0 3.4.2 [checked] -->
The Access-Control-Allow-Origin header is a fixed value or checked against a list of allowed origins.
<!-- ASVS 5.0 3.5.5 [checked] -->
Messages received through postMessage are discarded unless they come from a trusted origin.
<!-- ASVS 5.0 3.7.2 [checked] -->
The software redirects users to other domains only when the destination is on an allowed list.
<!-- ASVS 5.0 14.3.2 [checked] -->
Responses that carry sensitive data set Cache-Control: no-store.

# D2. Offers an API that others call
<!-- covered by baseline: every request handler checks on the server that the caller is allowed to -->
<!-- ASVS 5.0 14.2.1 [checked] -->
API keys, tokens and personal data are never placed in URLs or query strings.
<!-- ASVS 5.0 15.3.3 and 8.2.3 [checked]; OWASP API Security Top 10 2023, API3 [checked] -->
A caller cannot set fields it is not allowed to change by adding them to a request.
<!-- ASVS 5.0 15.3.1 [checked] -->
API responses return only the fields the caller needs, not whole stored objects.
<!-- ASVS 5.0 2.4.1 [checked]; OWASP API Security Top 10 2023, API4 [checked] -->
The API limits how many requests a caller can make in a period.
<!-- ASVS 5.0 13.2.4 and 13.2.5 [checked]; OWASP API Security Top 10 2023, API7 [checked] -->
When the server fetches a URL a caller supplied, the destination is checked against an allowed list and internal addresses are refused.
<!-- ASVS 5.0 16.5.1 [checked] -->
Error responses do not include stack traces, queries or internal paths.
<!-- ASVS 5.0 13.4.5 and 13.4.2 [checked] -->
API documentation, debug and monitoring endpoints are not reachable in the production configuration unless intended.
<!-- ASVS 5.0 4.3.1 [checked] -->
If the API uses GraphQL, query depth or cost is limited.

# D3. Receives webhooks or messages from outside services
<!-- practice; GitHub and Stripe webhook documentation recommend it [checked] -->
Every inbound webhook verifies the sender's signature or shared secret before it acts on the content.
<!-- practice -->
A webhook delivered twice has its effect only once.
<!-- practice -->
Webhook signatures are checked together with a timestamp or event id, so an old delivery cannot be replayed.
<!-- ASVS 5.0 2.2.1 [checked] -->
Webhook content is validated against the expected structure before it is used.

# D4. Is installed as an app on another company's platform
<!-- covered elsewhere: verifying requests from the platform is under D3; storing the platform's tokens is under D5 -->
<!-- Shopify mandatory compliance webhooks: acknowledge with a 200 status, then complete the action within 30 days [checked]; equivalent requirements of other platforms -->
Each privacy request the platform sends (data request, customer redaction, shop redaction) is carried out, not only acknowledged.
<!-- ASVS 5.0 10.2.3 [checked] -->
Each platform integration requests only the access scopes the software's features use.
<!-- practice -->
When the app is uninstalled, the software stops using the platform's data and deletes its access tokens.

# D5. Holds credentials for outside services on its operators' behalf
<!-- ASVS 5.0 13.3.1 [checked] -->
Stored credentials for outside services are encrypted with a key that is not stored in the same database.
<!-- practice; ASVS 5.0 14.2.6 [checked] -->
A credential, once saved, is never sent back to the browser in full.
<!-- practice -->
Disconnecting an integration deletes its stored credentials.
<!-- practice -->
An outside service's credential that has expired or been revoked is detected and reported to the operator, not retried indefinitely.

# D6. Sends email or text messages to people
<!-- CAN-SPAM, 15 U.S.C. 7704(a)(3) [checked: a reply address also satisfies it]; ePrivacy Directive 2002/58/EC Art. 13(2) [checked]; GDPR Art. 21(2) and 21(4) [checked] -->
Every marketing email contains a working unsubscribe link.
<!-- Google and Yahoo bulk-sender requirements, 2024: one-click unsubscribe [checked]; headers defined by RFC 2369 and RFC 8058 [checked] -->
Marketing emails carry List-Unsubscribe and List-Unsubscribe-Post headers.
<!-- CAN-SPAM, 15 U.S.C. 7704(a)(4) [checked: allows 10 business days]; Yahoo: within 2 days [checked] -->
No further marketing is sent to an address after it unsubscribes.
<!-- GDPR Art. 7(1) [checked: where consent is the legal basis] -->
The software records when and how each recipient consented before it sends them marketing.
<!-- CAN-SPAM, 15 U.S.C. 7704(a)(5)(A)(iii) [checked] -->
Marketing emails include the sender's postal address.
<!-- Google and Yahoo bulk-sender requirements, 2024 [checked: bounces yes; complaints are a spam-rate ceiling, not a suppression rule] -->
Bounced and complaining addresses are suppressed from further sending.
<!-- Google and Yahoo bulk-sender requirements, 2024: SPF and DKIM for bulk senders [checked; the DNS-records alternative is practice] -->
Outgoing email is signed with DKIM, or the materials state the DNS records the sending domain needs.
<!-- TCPA, 47 CFR 64.1200(a)(10) [checked]; CTIA Messaging Principles and Best Practices 5.1.3 [checked] -->
A text-message reply of STOP ends further messages to that number.

# D7. Carries live audio or video between users
<!-- ASVS 5.0 17.1.1 [checked] -->
The TURN relay refuses to relay to internal, loopback or other reserved addresses.
<!-- ASVS 5.0 17.2.3 [checked] -->
The media server checks SRTP authentication on incoming media.
<!-- ASVS 5.0 17.2.8 [checked] -->
The DTLS certificate is checked against the fingerprint in the session description, and the stream ends if they differ.
<!-- ASVS 5.0 17.3.1 [checked] -->
The signalling server limits the rate of incoming messages.

# D8. Is a mobile app
<!-- OWASP MASVS v2, MASVS-STORAGE-1 [checked: requires protection, not a particular store] -->
Tokens and personal data on the device are stored only in the platform's secure storage (Keychain, Keystore).
<!-- OWASP MASVS v2, MASVS-STORAGE-2 [checked] -->
Sensitive data is excluded from device logs and backups.
<!-- OWASP MASVS v2, MASVS-NETWORK-1 [checked] -->
The app connects only over TLS and does not disable certificate validation.
<!-- OWASP MASVS v2, MASVS-PLATFORM-1 and MASVS-CODE-4 [checked] -->
Deep links and data received from other apps are validated before use.
<!-- OWASP MASVS v2, MASVS-PLATFORM-2 [checked] -->
WebViews that show untrusted content do not expose native functions to JavaScript.
<!-- OWASP MASVS v2, MASVS-CODE-1 [checked: 'still receives vendor updates' is an inference] -->
The minimum operating-system version the app supports still receives security updates from its vendor.

# D9. Is used by the public, so accessibility law applies
<!-- WCAG 2.2, 1.1.1 [checked] -->
Images that carry meaning have text alternatives.
<!-- WCAG 2.2, 1.3.1 and 3.3.2 [checked] -->
Every form field has a label tied to it in the markup.
<!-- WCAG 2.2, 2.1.1 and 4.1.2 [checked] -->
Interactive elements are native controls, or carry a role and keyboard handling.
<!-- WCAG 2.2, 2.4.7 [checked] -->
The style sheets do not remove the visible focus indicator without replacing it.
<!-- WCAG 2.2, 3.1.1 [checked] -->
Each page declares its language.

# E1. Uses a language model to answer people or produce content
<!-- covered by baseline: user data sent to outside services only when configuration enables it -->
<!-- OWASP LLM01:2025 Prompt Injection [checked] -->
Text from users, documents or web pages is passed to the model separately from the instructions, never inserted into the instruction text.
<!-- OWASP LLM05:2025 Improper Output Handling [checked] -->
Model output shown in a web page is escaped or sanitised.
<!-- OWASP LLM05:2025 [checked] -->
Model output is never run as code, a database query or a shell command without validation by code.
<!-- OWASP LLM07:2025 System Prompt Leakage [checked] -->
Instructions given to the model contain no credentials or internal secrets.
<!-- OWASP LLM02:2025 Sensitive Information Disclosure [checked] -->
Data belonging to one customer is never placed in the model's context when answering another customer.
<!-- OWASP LLM10:2025 Unbounded Consumption [checked] -->
The number or size of model calls is limited per user or per organisation.
<!-- EU AI Act Art. 50(1) [checked: people must be told they are interacting with an AI system, unless it is obvious] -->
People talking to the model are told the answers come from an AI system.

# E2. Gives a language model tools that act or read data
<!-- OWASP LLM06:2025 Excessive Agency [checked] -->
Each agent can call only the tools on an explicit list for that agent.
<!-- OWASP LLM06:2025 [checked] -->
A tool that changes data, sends a message or spends money requires human approval, or is limited by checks in code that the model cannot change.
<!-- OWASP LLM06:2025 [checked]; ASVS 5.0 8.3.3 [checked] -->
Tools act with the permissions of the user or organisation the model is serving, not with service-wide permissions.
<!-- OWASP LLM01:2025 [checked] -->
Content a tool returns from an outside source is passed to the model as data, not as instructions.
<!-- OWASP LLM10:2025 [checked] -->
The number of tool calls per request or task is limited.
<!-- OWASP LLM06:2025 [checked] -->
Every tool call is logged with its arguments and outcome.

# E3. Retrieves from a store of customer content to answer
<!-- OWASP LLM08:2025 Vector and Embedding Weaknesses [checked] -->
Retrieval searches only the documents of the organisation or user being answered.
<!-- OWASP LLM08:2025 [checked] -->
A document that a user may not read is not retrieved to answer that user.
<!-- OWASP LLM08:2025 [checked]; practice, because GDPR Art. 17 where a document holds personal data [checked] -->
Deleting a document deletes its embeddings and stored chunks.
<!-- OWASP LLM04:2025 Data and Model Poisoning [checked] -->
Content ingested from websites or uploads is stored with its source, and the operator can remove it, after which it is no longer retrieved.

# F1. Is operated by the seller as a hosted service
<!-- covered by baseline: stored data can be backed up or exported by a mechanism in the materials -->
<!-- AICPA SOC 2, criteria A1.2 and A1.3 (backup and recovery testing) [checked, secondary source] -->
The materials describe how to restore the service from a backup.
<!-- ASVS 5.0 13.3.1 [checked] -->
Production secrets come from a secrets store or environment, not from files in the repository.
<!-- ASVS 5.0 13.4.2 [checked] -->
Debug modes are off in the production configuration.
<!-- ASVS 5.0 12.2.1 [checked] -->
Every external connection to the service uses TLS with no fallback to plain HTTP.
<!-- ASVS 5.0 16.4.3 [checked] -->
Logs and errors are sent to a system separate from the application servers.
<!-- AICPA SOC 2, criterion CC7.2 (monitoring for anomalies) [checked, secondary source: framed around security events, not service failure] -->
The materials define health checks and alerts that notify someone when the service fails.

# F2. Is distributed for others to install and run
<!-- ASVS 5.0 13.2.3 [checked] -->
The shipped configuration contains no fixed secret keys; each installation generates or must set its own.
<!-- practice -->
Releases are versioned, and each release states the changes it makes.
<!-- practice -->
Upgrading from one release to the next applies database changes by migrations, without manual steps.
<!-- OpenSSF Scorecard: Signed-Releases [checked]; SLSA v1.0 Build L1/L2, provenance [checked: signing without provenance is not SLSA] -->
Release artifacts are signed or published with build provenance.
<!-- OpenSSF Scorecard: Security-Policy [checked] -->
The repository states how to report a security vulnerability.
<!-- CIS Docker Benchmark v1.6.0, 4.1 [checked, secondary source] -->
Container images shipped with the software run as a user other than root.

# F3. Runs scheduled or background jobs the business depends on
<!-- covered by baseline: every timed task is scheduled by a definition in the materials -->
<!-- ASVS 5.0 15.4.2 [checked]; practice -->
A job cannot run twice at once; overlapping runs are prevented by a lock or by the scheduler.
<!-- ASVS 5.0 16.3.4 [checked] -->
A failed job run is logged as a failure and retried or reported.
<!-- practice -->
A job left unfinished by a crash is detected and recovered when the worker starts.
<!-- practice -->
A run missed while the service was down is caught up or recorded as missed.
<!-- ASVS 5.0 15.2.2 [checked] -->
Long-running jobs have a time limit.

# F4. Depends on one cloud provider's managed services
<!-- ASVS 5.0 13.1.1 [checked] -->
The materials list every managed service the software uses.
<!-- practice -->
Calls to each provider-specific service go through one module of the code.
<!-- practice -->
Account ids, regions and resource names come from configuration, not from the code.

# G1. Controls or monitors physical equipment
<!-- IEC 62443-4-2, CR 1.1 and CR 2.1 [checked, titles only from a secondary source] -->
Commands to equipment can be sent only by an authenticated user whose role allows it.
<!-- practice; IEC 62443-4-2 CR 3.5 input validation [checked, title only: safety limits go beyond it] -->
Safety limits on commands (speed, temperature, pressure, quantity) are enforced on the server before a command is sent, not only in the user interface.
<!-- IEC 62443-4-2, CR 2.8 auditable events [checked, title only] -->
Every command sent to equipment is logged with the user and the time.
<!-- IEC 62443-4-2, CR 3.1 RE(1) communication authentication and CR 4.1 [checked, titles only] -->
Communication with the equipment is authenticated and encrypted where the protocol supports it.
<!-- NIST SP 800-82 Rev. 3, 5.3.1 and SC-24 fail in known state [checked: guidance; 'or reports the loss' is practice] -->
When the connection to the equipment is lost, the software puts the equipment in a defined safe state or reports the loss.

# G2. Records stock, orders or other physical quantities the business acts on
<!-- ASVS 5.0 2.3.4 and 2.3.3 [checked] -->
Stock is reserved inside a database transaction with a lock, so two orders cannot take the same last unit.
<!-- practice -->
Every change in stock is recorded as a movement with its quantity, reason and time, so the quantity on hand can be rebuilt from the movements.
<!-- ASVS 5.0 2.2.3 [checked] -->
A change that would make stock negative is refused or flagged.
<!-- practice -->
Adjustments from a physical count are recorded separately, with who made them.

# G3. Must keep working when the network is down
<!-- practice -->
Transactions made while offline are stored on the device in a way that survives a restart.
<!-- practice -->
On reconnection, each offline transaction is sent once, and duplicates are refused.
<!-- practice -->
A conflict between an offline change and a change on the server is settled by a rule written in the code.
<!-- practice; PCI DSS v4.0.1 req. 3.5.1 where card numbers are stored [checked] -->
Data stored on the device for offline use is encrypted.

# G4. Depends on particular hardware, drivers or operating-system versions
<!-- ASVS 5.0 15.1.2 [checked] -->
The materials list each hardware driver, vendor SDK and operating-system version the software needs.
<!-- practice -->
Code that talks to hardware is kept in one module per device type.
<!-- ASVS 5.0 3.7.1 [checked]; practice -->
The operating systems and runtimes the software targets still receive security updates from their vendors.

# H1. Includes commercial components under licence
<!-- practice -->
The materials list each commercial component, its vendor and the licence under which it is used.
<!-- practice -->
Each commercial component's licence terms are in the materials.

# H2. Is open source with outside contributors
<!-- practice -->
Outside contributions are accepted under a contributor licence agreement or a Developer Certificate of Origin sign-off, and the repository enforces it.
<!-- practice -->
Code copied from another project keeps that project's licence notice.
<!-- practice -->
The licence stated in file headers and package metadata matches the repository's licence file.
