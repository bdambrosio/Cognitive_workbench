# Software properties, version 0 (draft 2026-10-02)

A property is a fact about the software that decides which questions a buyer
should ask of it. An engagement identifies its properties after the seller's
claims are enumerated and the code has been examined; a piece of software
usually has several. Each property will carry a list of statements, written
like the baseline list's: one testable statement per line, each citing a
requirement of a published standard or marked "practice". This file lists
the properties only. Statements are drafted after this list is approved.

The baseline list (draft baseline v2, logs/BASELINE_QUESTIONS.v2.draft.md) applies to every engagement and is
not a property.

Two things are deliberately not properties:
- Jurisdiction (EU residents, a US state). It is a fact about the
  transaction and the buyer, recorded at intake. It decides which law a
  personal-data statement cites, not whether the statement is asked.
- Size of the business. It changes how much a finding matters, which the
  materiality stage rates against the engagement's thresholds.

Column "Shows as" describes what in the claims or the code indicates the
property, for the model that identifies properties. It is a description for
judgement, not a list of strings to match.

Citations: "checked" means the source's structure was read on 2026-10-02
(ASVS 5.0 chapter list at github.com/OWASP/ASVS/tree/master/5.0/en; OWASP Top
10 for LLM Applications 2025 at genai.owasp.org/llm-top-10; OpenSSF Scorecard
checks at github.com/ossf/scorecard/blob/main/docs/checks.md). Every other
citation is from memory and must be checked before statements cite it.

## A. Data the software holds

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| A1 | Holds personal data about people other than its operator | Tables or models with names, emails, phone numbers, addresses, message text of customers, visitors, subscribers or employees | GDPR Arts 5, 15–17, 20, 25, 32; CCPA/CPRA; ASVS V14 Data Protection (checked) | ChatterMate visitors; listmonk subscribers |
| A2 | Holds health or other special-category data | Medical, biometric, religious, union, sexual-orientation fields; claims about patients or clinics | HIPAA Security Rule 45 CFR 164.312; GDPR Art 9 | Clinic booking system |
| A3 | Stores files that users upload | Upload endpoints, object storage, file parsing | ASVS V5 File Handling (checked) | ChatterMate chat attachments |
| A4 | Keeps financial or accounting records the business relies on | Invoices, ledgers, payouts, tax amounts, reconciliation jobs | practice; tax record-retention rules by jurisdiction | Subscription billing back office |

## B. Money

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| B1 | Takes payments through a payment processor | Stripe, Paddle, PayPal or similar SDK; checkout pages; payment webhooks | PCI DSS v4.0 (scope: card data never touches the software); processor's webhook-verification documentation | Most small SaaS |
| B2 | Handles or stores card data itself | Card number fields, tokenisation code of its own, a card-present terminal integration | PCI DSS v4.0 full scope | In-store point of sale |
| B3 | Bills its own customers on a schedule | Subscription plans, invoice generation, dunning, usage metering | practice | The Acquire post's cron-on-a-laptop case |

## C. Users and access

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| C1 | Has user accounts with a login of its own | Password storage, login and reset flows, sessions | ASVS V6 Authentication, V7 Session Management (checked); NIST SP 800-63B | Almost any web app |
| C2 | Lets users log in through another identity provider | OAuth or OIDC client code, SSO, SAML | ASVS V10 OAuth and OIDC (checked) | "Sign in with Google" |
| C3 | Serves several customer organisations from one installation | Organisation or tenant ids on most tables; per-organisation settings | ASVS V8 Authorization (checked); requirement 8.4.1 on cross-tenant controls (not checked) | ChatterMate hosted service |
| C4 | Issues its own tokens or API keys to callers | JWT issuing, API-key tables, token minting endpoints | ASVS V9 Self-contained Tokens (checked) | ChatterMate CLI token minting |

## D. How it is reached and what it connects to

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| D1 | Has a web front end that runs in a browser | HTML templates, a JavaScript front end | ASVS V3 Web Frontend Security (checked) | Almost any web app |
| D2 | Offers an API that others call | Documented endpoints, an OpenAPI file, SDKs, a CLI against the API | ASVS V4 API and Web Service (checked); OWASP API Security Top 10 2023 | ChatterMate API |
| D3 | Receives webhooks or messages from outside services | Inbound webhook routes, message-queue consumers | ASVS V4 (checked); each sender's signature-verification documentation | ChatterMate channels |
| D4 | Is installed as an app on another company's platform | Shopify, Slack, Meta, Salesforce app manifests; platform-mandated webhooks | Each platform's published app requirements (e.g. Shopify mandatory compliance webhooks) | ChatterMate Shopify app |
| D5 | Holds credentials for outside services on its operators' behalf | Stored API keys or OAuth tokens for third-party accounts | ASVS V13 Configuration, V11 Cryptography (checked) | ChatterMate CRM and channel integrations |
| D6 | Sends email or text messages to people | SMTP or SMS provider code, campaigns, transactional mail | CAN-SPAM; TCPA; GDPR consent (Art 6–7); RFC 8058 one-click unsubscribe | listmonk |
| D7 | Carries live audio or video between users | WebRTC code | ASVS V17 WebRTC (checked) | Video support widget |
| D8 | Is a mobile app | iOS or Android project | OWASP MASVS | Companion app |
| D9 | Is used by the public, so accessibility law applies | Public-facing pages or widgets | WCAG 2.2; European Accessibility Act; ADA case law | ChatterMate widget on shop sites |

## E. Language-model features

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| E1 | Uses a language model to answer people or produce content | Calls to model providers; prompts in the code | OWASP Top 10 for LLM Applications 2025 (checked): LLM01 Prompt Injection, LLM02 Sensitive Information Disclosure, LLM05 Improper Output Handling, LLM07 System Prompt Leakage, LLM09 Misinformation, LLM10 Unbounded Consumption | ChatterMate answers |
| E2 | Gives a language model tools that act or read data | Tool or function definitions passed to a model; agents with database, API or messaging access | OWASP LLM06 Excessive Agency (checked) | ChatterMate ticket investigator with SQL and observability tools |
| E3 | Retrieves from a store of customer content to answer | Embeddings, a vector store, knowledge-base ingestion | OWASP LLM04 Data and Model Poisoning, LLM08 Vector and Embedding Weaknesses (checked) | ChatterMate knowledge base |

## F. How it runs

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| F1 | Is operated by the seller as a hosted service | Production deploy files, a hosted sign-up, claims about uptime or a free tier | AICPA SOC 2 Trust Services Criteria (availability, confidentiality); practice | ChatterMate cloud |
| F2 | Is distributed for others to install and run | Installers, Docker images, packages, upgrade notes | SLSA; OpenSSF Scorecard Signed-Releases, Packaging (checked) | ChatterMate self-host; listmonk |
| F3 | Runs scheduled or background jobs the business depends on | Workers, queues, cron definitions, periodic tasks | practice | ChatterMate workers; the Acquire post's billing cron |
| F4 | Depends on one cloud provider's managed services | Provider-specific SDKs for queues, storage, functions; infrastructure files for one provider | practice | Serverless app on one provider |

## G. The physical world

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| G1 | Controls or monitors physical equipment | Serial, Modbus, OPC UA, PLC or device protocols; sensor readings; commands to machines | IEC 62443-4-2; NIST SP 800-82 | Factory line controller |
| G2 | Records stock, orders or other physical quantities the business acts on | Inventory, warehouse, order-fulfilment models | practice | Warehouse system of a distributor |
| G3 | Must keep working when the network is down | Local caches, offline queues, sync on reconnect; claims about offline use | practice | Point of sale in a shop |
| G4 | Depends on particular hardware, drivers or operating-system versions | Vendor SDKs, device drivers, pinned OS images, Windows-only components | practice; vendor end-of-life notices | Label printer or scale integration |

## H. What it is built from

| # | Property | Shows as | Sources | Example |
|---|---|---|---|---|
| H1 | Includes commercial components under licence | Vendored SDKs, licence keys, paid libraries, fonts or data feeds | practice: licence transfer on sale | Reporting component sold per seat |
| H2 | Is open source with outside contributors | Public repository, contributors other than the seller, CLA or DCO | OpenSSF Scorecard Contributors, Code-Review (checked); practice: ownership of contributed code | ChatterMate |

## Questions for review

1. Granularity: A4 and B3 overlap, as do D3 and D4, and E1–E3. Keep them
   separate (smaller, more precise lists) or merge them (fewer properties to
   identify)?
2. Missing: does the list lack a property a buyer of in-house software for a
   physical business would expect? G covers equipment, stock, offline
   operation and hardware; it does not cover scheduling of people, food
   safety or other industry-specific records, which may be better handled as
   questions for the seller than as testable statements.
3. A2 (health data) and B2 (card data itself) bring regulated regimes whose
   full requirements cannot be settled from code. Keep them, with statements
   limited to what the code shows, or drop them from v0?
