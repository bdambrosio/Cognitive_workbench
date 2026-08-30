# Technical claims audit — ChatterMate

Materials as of 2026-08-30. Limited assurance; see Limitations.

## Summary

**ChatterMate** — open-source AI customer-support platform (chat widget, AI agent, human handover, integrations). Python backend, Vue frontend.

**Conclusion: Material** — the security-critical claims hold; three advertised features do not exist.

Key findings:
- **Cross-session AI memory: claimed, not real.** Agent memory is per-session only; the customer-level history query exists but is never wired in.
- **Auto-translation: claimed, not real.** Only a `default_language` column exists; no translation code anywhere.
- **ChatterMate CLI: claimed, not real.** No CLI implementation exists in the repository or as an entry point.
- **Workflow version control: partial.** Builder and branching are real; save/restore versioning is not — only an unused version column.
- **Knowledge worker recovery: partial.** Jobs stuck in `processing` after a crash are never reaped (the FAQ queue and ticket investigator both have reapers; the knowledge queue does not).

What held: all six Tier 1 SQL-connector isolation claims (read-only AST gate, masked columns, row scoping, cross-customer redaction, audit logging) are supported by the guardrail code; RBAC, multi-model adapters, Shopify, Jira, all claimed channels, CRM sync, KB ingestion, and widget auth are real and tested.

Coverage: 47 of 48 identified claims resolved; 43 of 47 resolved claims supported. One claim unverifiable (enterprise submodule not supplied); nothing unattempted.

A professional-judgment note, Gap Map only: the three false claims are absent features rather than broken implementations, and the documentation pattern (docs written ahead of code) suggests the buyer should price the gap as missing scope, not hidden defects. The full cited report is available.

technical claims verification · not a pen-test · not legal advice

## About this audit

This document is a claims audit of ChatterMate, a customer-support chat platform. It examines the seller's product claims against the codebase of an August 2026 snapshot. It was a static audit: the code was read, not run.

What you are holding: an executive summary, this introduction, the audit's conclusion, twelve findings with citations, a coverage statement in prose, questions for the seller, the audit's limitations, and two appendices. Appendix A lists the claims the audit found supported. Appendix B lists the full claim surface.

The audit's conclusion is Material: the security-critical claims hold, but three seller claims describe features that do not exist, and two substantially true claims have material gaps.

Every finding cites the specific files and lines that settle it. Any finding can be checked against the evidence it cites.

## Conclusion — Material

The Tier 1 isolation and data-protection claims (12–17) are fully supported by the guardrail code, and most of the platform's advertised functionality is real. But three seller claims are contradicted by the evidence — cross-session AI memory, auto-translation, and the advertised CLI — and none of these is a small bug; each is an absent feature described as present. A buyer is acquiring less functionality than the claim sources describe, which bears directly on valuation.

## How to read a finding

Each finding names the seller's claim, the evidence that settles it, and the gap between them. The reference after each piece of evidence is a file and line number in the materials examined, so any finding can be checked.

The bracket at the end of a finding's title is the audit's verdict on that claim:

- `[real]` — the materials bear the claim out
- `[real, minor caveat]` — the claim holds, with a discrepancy that does not affect the decision
- `[real, operational caveat]` — the claim holds today, but how the system is operated qualifies it
- `[partial]` — the claim is substantially true and has a specific, cited gap
- `[delta]` — the claim is not true — the claim source says one thing and the materials show another
- `[unverifiable]` — the audit tried and the supplied materials could not settle it

## Claims the evidence contradicts

Three of the seller's claims describe features that are not in the materials: cross-session conversation memory, auto translation, and the ChatterMate CLI. Each is an absent feature described as present, not a defect in something that exists.

**Finding 1: Cross-session conversation memory — [delta]**
Claim (README.md): AI responses have conversation memory across sessions.
Evidence: backend/app/agents/chat_agent.py:898 (agent built with a single `session_id`), :904-907 (`add_history_to_messages` from agno per-session storage); backend/app/repositories/chat.py:307-314 — `get_user_history` returns customer-level history across sessions but is never called from backend/app/agents; backend/app/agents/encrypted_storage.py:40-75 encrypts per-session blobs only.
Gap: Memory is per-session only. A returning customer's prior conversations are invisible to the agent.

**Finding 2: Auto translation — [delta]**
Claim (README.md): Auto translation with configurable default language per workflow.
Evidence: backend/app/models/workflow.py:41 — `default_language` column exists and is passed through CRUD (backend/app/api/workflow.py:86,141,197); lexical search of backend/app for 'translat' finds no translation service, API call, or code applying the setting to generated responses.
Gap: The configuration field exists; the translation implementation does not.

**Finding 3: ChatterMate CLI — [delta]**
Claim (llms.txt): A `chattermate` CLI exists with commands signup, agent create, knowledge add-url, knowledge add-file, widget create.
Evidence: repo-wide search — no CLI package, console_scripts entry point, or command implementation anywhere; README.md:246-250 advertises `pip install chattermate-cli`, but no such package exists in this repository.
Gap: The CLI is documentation-only.

## Real features with material gaps

Two claims are substantially true — the features exist and work — but are missing a part the seller's description implies. The gap is in what is absent, not in what is broken.

**Finding 4: Workflow version control — [partial]**
Claim (README.md): Visual workflow builder with node types Start / AI Response / Human Transfer / Condition / End, real-time preview, and version control (save/restore).
Evidence: backend/app/models/workflow_node.py:26-37 — NodeType enum covers the named nodes (plus seven more); branching via WorkflowNode/WorkflowConnection (workflow_node.py:53-58, workflow_connection.py:25). backend/app/models/workflow.py:39 — `version = Column(Integer, default=1)` is never incremented, appears only as a read-only echo (api/workflow.py:84,139,195); no restore endpoint, no `workflow_versions` table (repo-wide grep: none).
Gap: Drag-and-drop builder and branching are real; version control (save/restore) has no implementation.

**Finding 5: Knowledge worker stuck-job recovery — [partial]**
Claim (README.md:420): knowledge worker polls every 60s and fails any job left stuck in `processing` on startup.
Evidence: deployed entrypoint knowledge_processor.py:248 does sleep 60; but no reaper for stuck `processing` knowledge jobs exists anywhere (lexical + structural search) — only the FAQ queue has one (faq_processor.py:138-147, `fail_orphaned_processing`). The 10s-poll entrypoint (run_knowledge_processor.py:37) is not the deployed one.
Gap: A crashed worker leaves knowledge jobs stuck in `processing` permanently. The ticket investigator's equivalent reaper is real (ticket_investigator.py:628-635).

## Unverifiable

No numbered findings fall in this group. One claim could not be settled either way, because it concerns a separate repository that was not supplied for review. It is accounted for in the coverage statement.

Claim 41 (help_center plan gating in the enterprise submodule): [unverifiable] — the enterprise submodule is a separate repository not supplied; this repo contains only the integration surface.

## Supported, with minor caveats

Seven features exist and work as the seller describes. Each carries a small qualification the audit records alongside it — a detail of configuration, an inconsistency, or a narrower scope than the claim suggests. None changes what the feature is.

**Finding 6: Shopify integration — [real, minor caveat]**
Claim (README.md:49): native Shopify integration answering order, shipping, product questions from live store data.
Evidence: backend/app/tools/shopify_toolkit.py — list_products:123, get_product:204, search_products:266, search_orders:503, get_order_status:800 (with fulfillments/tracking via GraphQL, shopify.py:780-782, 853-868); OAuth in shopify_auth_service.py:40,66.
Gap: None material. Caveat: ShopifyShop.access_token is stored unencrypted (shopify_shop.py:31-37), unlike CRM/channel credentials which are Fernet-encrypted.

**Finding 7: Human handoff context — [real, minor caveat]**
Claim (README.md): transfers with full context; detects frustration or explicit human requests.
Evidence: chat_agent.py:590-606 — transfer triggers include explicit human request (l.595) and frustration (l.596), signalled via structured output (l.77-80) and acted on at :1413-1423.
Caveat: the transfer prompt carries the last 5 messages (transfer_agent.py:132); full session history remains in the record but the handoff summary is thinner than "full context" implies.

**Finding 8: MCP investigation sources — [real, minor caveat]**
Claim (README.md): investigation sources connect via MCP: Grafana, Elasticsearch, Sentry, CloudWatch, or any MCP server.
Evidence: MCPTool model (mcp_tool.py:29-57), REST API (api/mcp_tool.py:65,158), manager (mcp_manager.py:42-66), investigator wiring (ticket_investigator.py:366-394).
Caveat: there are no named presets for Grafana/Elasticsearch/Sentry/CloudWatch — connectors are fully user-configured.

**Finding 9: File attachments — [real, minor caveat]**
Claim (README.md): images, PDFs, Word docs, spreadsheets; S3 with magic byte validation.
Evidence: file_upload_service.py:28-33 (types), :121-169 (magic bytes), :347-355 (S3), :38-39 (10MB/5MB limits).
Caveat: api/file_upload.py:37 also allows image/svg+xml, which the service layer deliberately excludes (XSS) — an API-layer/service-layer inconsistency.

**Finding 10: Jira integration — [real, minor caveat]**
Claim (README.md): creates and tracks issues from chat via OAuth 2.0; native tickets escalate one-way.
Evidence: OAuth2 with CSRF state (services/jira.py:36-100, api/jira.py:168-263); JiraTools.create_jira_ticket (jira_toolkit.py:31-44,129-136); escalation one-way by design (ticket_jira.py:16-18, "nothing syncs back").
Caveat: no automated status/comment sync back; Jira status is readable on demand only (jira_toolkit.py:447).

**Finding 11: Knowledge defaults — [real, minor caveat]**
Claim (knowledge/README.md): defaults max_depth 5, max_links 25, min_content_length 100, timeout 30s, max_retries 3, max_workers 10, batch_size 20.
Evidence: config.py:153-154 (KB_MAX_DEPTH=5, KB_MAX_LINKS=25, applied at enhanced_website_kb.py:51-52); reader fields min_content_length=100 (:68), timeout=30 (:99), max_retries=3 (:100), max_workers=10 (:102).
Gap: batch_size default is 5, not 20 (config.py:159, KB_BATCH_SIZE; enhanced_website_kb.py:57) — the module README's "20" is stale. Non-material.

**Finding 12: Knowledge-module tests — [real, minor caveat]**
Claim (knowledge/README.md): comprehensive unit tests in the three named files.
Evidence: all three exist — test_enhanced_website_reader.py (18 test methods), test_enhanced_website_kb.py (12), test_optimized_pgvector.py (3).
Caveat: three tests for the vector layer is thin; "comprehensive" overstates that one.

## Coverage

The audit identified 48 seller claims. It resolved 47 of them.

Of the 47 resolved claims, 43 were supported by the evidence. Three claims are not true — the features described do not exist. Two claims are substantially true but have material gaps. That leaves one claim the audit could not settle: it concerns help-center plan gating in the enterprise submodule, a separate repository that was not supplied.

Eleven externally-facing assertions in the seller's materials were not counted as claims and were not tested.

The full list of claims, and how each was resolved, is in Appendix B.

## Questions the client should ask the seller before closing

1. Is the Shopify access token stored unencrypted by design, and is encryption on the roadmap? (CRM credentials are encrypted; this one is not.)
2. Are cross-session memory, auto-translation, and the CLI (README claims 22, 8; llms.txt claim 34) planned, deprecated, or errors in the documentation? If deprecated, when were the docs last reviewed?
3. Why does api/file_upload.py permit image/svg+xml when the service layer deliberately excludes it for XSS reasons — and which layer actually enforces on the deployed path?
4. Is the missing stuck-job reaper for the knowledge queue (README claim 31) a known gap? What happens operationally when the knowledge worker crashes mid-job?
5. Is the published `chattermate-cli` PyPI package the same code as this repository, or maintained elsewhere?

## Limitations

1. Materials examined: the ChatterMate repository as bound to this session (backend, frontend, wordpress, chattermate-test, deploy files), with claim sources README.md, llms.txt, HELP_CENTER_INFRA.md, and backend/app/knowledge/README.md. Conclusions apply to this snapshot as of August 2026; the repository is a development tree and may not reflect any deployed version.
2. The seller was not consulted at any point in this engagement and has not confirmed the audit's interpretation of the seller's claims. Wording such as "full context", "comprehensive tests", and "version control" was interpreted from the documents alone.
3. This is a limited-assurance audit. Coverage: 47 of 48 identified claims resolved; 43 of 47 resolved claims supported. Eleven externally-facing assertions in the claim sources (hosted-service pricing and uptime, the Shopify App Store listing, the WordPress plugin distribution, YouTube demos, and competitive comparisons) are outside the supplied evidence and were neither claimed-counted nor tested. The enterprise submodule was not supplied, so one claim (41) is unverifiable. No dynamic testing was performed: this is a static reading of the code and documents, not a running-system verification.

## Appendix A — supported claims, with citations

Every remaining resolved claim, with the evidence that resolves it. These are listed rather than written up because none of them is a gap.

### Findings — supported

3. Business-hours awareness + real-time availability — [real]: business_hours.py:51-83 (org hours + timezone); transfer_agent.py:231-239 (is_online/is_active count).
5. Frustration/explicit-request transfer — [real]: chat_agent.py:595-596, :1413-1423 (see Finding 7 for the context caveat).
6. Ask Anything Mode — [real]: widget.py:169,357,377-388,429-442 — anonymous tokens, `{timestamp}@noemail.com` placeholder customers.
9. AI Ticketing triage/dedup — [real]: chat + webhook creation paths (ticket_toolkit.py:67/137-147; ticket_webhooks.py:82-133); embedding dedup against open tickets, thresholds 0.90/0.95 (ticket.py:64,178-187; ticket_webhooks.py:43,105-123).
10. Investigation evidence trail + versioned RCA — [real]: EvidenceRecorder writes InvestigationEvent rows per tool call (ticket_investigation.py:69-151; investigation.py:190,219-222); hypotheses persisted (investigation.py:148); RCADocument versioned with uniqueness constraint and inline citations (investigation.py:278-325).
12. SQL connector structurally read-only — [real]: sql_guardrails.py:219-352 (single plain SELECT via AST, blocked node types :40-73, dangerous-function denylist, allowlist through CTEs/joins/subqueries, forced LIMIT); db_connector_service.py:163-237 (read-only transaction, statement timeout).
13. Masked columns never readable — [real]: sql_guardrails.py masked-column reference blocking incl. WHERE clauses, whole-row smuggling blocked, mask_rows re-masking :355-369.
14. Nothing written to customer DB — [real]: single-SELECT AST gate plus read-only session; no write path.
15. Row-level scoping — [real]: sql_guardrails.py:139-205 `_apply_row_scope`, fail-closed when no customer known.
16. Cross-customer isolation — [real]: ticket_privacy.py:66-117 (scrub_outbound preserves recipient's own identifiers, redacts third-party; redact_reference_text before model context); ticket.py:755-767; ticket_investigator.py:84-92; tests test_ticket_privacy.py.
17. Query audit logging, rows never stored — [real]: guardrailed_db_toolkit.py:131-159 logs SQL and outcome without rows.
19. Widget token auth — [real]: JWT HS256 conversation tokens (security.py:123-167), Redis JTI revocation (:170-197), private mode 401 (widget.py:149-156), API-key issuance (token.py:150-348); tests test_token.py.
20. Slack internal channel — [real]: slack.py:222 SlackAdapter; api/channels/slack.py router.
23. Analytics dashboard — [real]: api/analytics.py — agent-performance:65, analytics:172, customer-analytics:380, and further endpoints.
24. KB training (URL crawl, PDF, sitemap) — [real]: knowledge_base.py:384→add_pdf_files:276, :444→EnhancedWebsiteReader:459, :505→SitemapReader:514; OptimizedPgVector :80.
25. Custom theming — [real]: AgentCustomization color/theme fields (agent.py:52-59).
26. RBAC with granular permissions — [real]: role.py:22-26, permission.py:44-86, require_permissions (auth.py:90-101), 50+ endpoint uses, role self-modification prevented (users.py:854-856).
27. Channel adapters — [real]: channels/base.py:70 ChannelAdapter ABC; whatsapp.py:32, messenger.py:31, instagram.py:31, slack.py:222, telegram.py:96, line.py:83, sms/adapter.py:34 (Twilio/Vonage/Plivo/MessageBird/Brevo/SNS), email.py:115 — all claimed channels present.
28. CRM sync HubSpot/Pipedrive — [real]: crm/hubspot.py:44, crm/pipedrive.py:61, workers/crm_sync.py:43 (10s poll, SKIP LOCKED batching), Fernet-encrypted credentials (crm.py:105,124).
29. Apache-2.0 licensing — [real]: LICENSE (Apache-2.0), NOTICE, DCO at repo root.
30. Self-hosted stack with separate workers — [real]: docker-compose.prod.yml:65 (knowledge_processor), :111 (crm_sync), :131 (ticket_investigator).
32. FAQ jobs drained by knowledge worker — [real]: run_knowledge_processor.py:67-73 and knowledge_processor.py:250-260 both run the FAQ loop.
33. Tests exist — [real]: substantial backend pytest suite (backend/tests/, dozens of files); frontend test:unit (vitest) and test:e2e (playwright) in package.json:17-18.
35. API under /api/v1 — [real]: config.py:37 API_V1_STR="/api/v1"; main.py router mounting.
36. Widget embedding — [real]: window.chattermateId + webclient/chattermate.min.js pattern (chattermate-test/index.html:376-432; backend/assets/widget.js shipped).
37. Help-center host-dispatched SSR — [real]: help_center_host.py:16-25; main.py:332-354 middleware; no frontend/nginx changes needed.
38. Custom domains with DNS verification + ssl_status lifecycle — [real]: domain_verification.py (TXT token :63, https /healthz probe :138, ssl_status on valid cert :23).
39. HELP_CENTER_TARGET_IPS frozenset JSON setting — [real]: config.py:220-222.
40. Help-center queries recorded, POST /ask answers — [real]: api/help_center_public.py + help_center_queries; test_help_center_public.py (27,636 bytes) corroborates.
42. FAQ tunables with stated defaults — [real]: config.py:224 (300), :225 (15000), :234 (100000), :235 (30) — all verbatim as claimed.
43. Multi-strategy extraction, exponential backoff, ThreadPoolExecutor — [real]: reader strategies and density analysis; backoff `2 ** retry_count` (enhanced_website_reader.py:792,800,809); ThreadPoolExecutor (:27, parallel crawl :908).
44. OptimizedPgVector — [real]: docstring :32-39; batched inserts (batch_size 100, :92-135); upsert-not-recreate via on_conflict_do_update (:196-209); HNSW/Ivfflat index support (:26,:49).
45. Crawl scope modes and schemes — [real]: SCOPE_PATH/HOST/DOMAIN (crawl_scope.py:45-47), host default (:50); only http/https followed (CRAWLABLE_SCHEMES :54, refusal check :151-152).
46. Migration and prune script — [real]: backend/alembic/versions/purge_mangled_crawl_pages_001.py exists (revision at :53); scripts/prune_offsite_pages.py deletes only with --apply.
1. Open-source AI support platform with AI-to-human handoff — [real]: Apache-2.0 core plus the handoff implementation cited under Findings 5/7.
2. Multi-model support — [real]: AIModelType enum (ai_config.py:23-34); per-provider factory branches (agno_utils.py:31-144, incl. OpenAI:51, Claude:56, Gemini:62, DeepSeek:59, Groq:69, Mistral:125, xAI:134); MODEL_CATALOG with custom model IDs (model_catalog.py:58-150); switching is DB configuration (ai_setup.py:368-400).
4, 11, 18, 31-poll — see Findings 6, 8, 10, 5 above.

## Appendix B — the claim surface

Every assertion identified in the claim sources and frozen before verification began. The claim numbers used above, and in the coverage statement, index this list.

48 claims

README.md (33 claims):
1. The product is an open-source AI customer support platform with AI-to-human handoff (Apache-2.0 core).
2. Multi-model AI support: OpenAI, Anthropic (Claude), Google Gemini, Mistral, xAI (Grok), DeepSeek, and Groq adapters exist, plus custom model ID entry, switchable without code changes.
3. Human handoff includes business-hours awareness and real-time agent availability detection.
4. Native Shopify integration answers order, shipping, and product questions from live store data.
5. Human handoff detects frustration or explicit requests for a human and transfers with full context.
6. Ask Anything Mode lets visitors chat with no signup or email required.
7. Chat file attachments support images, PDFs, Word docs, spreadsheets; uploads go to S3 with magic byte validation.
8. Auto translation with configurable default language per workflow.
9. AI Ticketing: tickets raised from chat (AI or agent) or from an alert webhook; each is triaged and deduplicated against open tickets.
10. AI Ticketing investigation: the AI proposes hypotheses, gathers evidence from connected tools, records each query as inspectable evidence, and produces a versioned root-cause analysis with citations.
11. Investigation sources connect via MCP: Grafana, Elasticsearch, Sentry, CloudWatch, or any MCP server.
12. The SQL connector is structurally read-only: queries parsed to AST and rejected unless a single plain SELECT; writes/DDL/multi-statement/dangerous-function denylist cannot pass; table allowlist enforced through CTEs/joins/subqueries; a LIMIT is forced; comments stripped before execution; connection is a read-only transaction with statement timeout.
13. Masked columns are never readable: blocked from reference anywhere including WHERE clauses; whole-row smuggling (to_jsonb(t), t::text, to_json(t.*)) is blocked; results re-masked on return.
14. Nothing is ever written to the customer's database by the AI.
15. Row-level scoping: queries against a scoped table are rewritten to the ticket customer's own rows, the filter is applied at the selected table (not bypassable in conditions), and a ticket with no known customer cannot query a scoped table at all.
16. Cross-customer isolation on outbound messages: other customers' identifiers are stripped from messages to a customer, and other customers' tickets are redacted before reaching the model.
17. Every SQL query attempt is logged with SQL and outcome; returned rows are never stored.
18. Jira integration creates and tracks issues from chat via OAuth 2.0; native tickets escalate to Jira one-way.
19. Widget authentication is token-based, supporting both public Q&A and private authenticated conversations.
20. Slack integration connects a workspace as an internal support channel.
21. Visual workflow builder: drag-and-drop nodes, branching logic, node types Start / AI Response / Human Transfer / Condition / End, real-time preview, and version control (save/restore).
22. AI responses have conversation memory across sessions.
23. An analytics dashboard provides real-time monitoring, conversation insights, and performance metrics.
24. Knowledge base training: AI agents are trained on domain-specific knowledge and FAQs (URL crawl, PDF, sitemap ingestion).
25. Custom theming: fully customizable chat widget.
26. Role-based access control with granular permissions for team members.
27. Messaging channels exist as adapters in backend/app/channels/ against a shared ChannelAdapter base: WhatsApp, Facebook Messenger, Instagram, Slack, Telegram, LINE, SMS (Twilio/Vonage/Plivo/MessageBird), and Email (inbound becomes a conversation, replies go back over email).
28. CRM sync pushes leads/contacts to HubSpot and Pipedrive via a crm_sync worker.
29. The codebase is Apache-2.0 licensed (with NOTICE, DCO contributions), free for commercial and SaaS use.
30. Self-hosting runs the full stack (Postgres, Redis, backend, frontend, worker) via chattermate-deploy / docker compose; the API server only enqueues jobs and separate worker processes execute them (knowledge+FAQ, ticket investigator, crm_sync), as documented.
31. Worker behavior as documented: knowledge worker polls every 60s and fails jobs stuck in 'processing' on startup; ticket investigator polls investigations every 15s and lifecycle/SLA sweeps every 5 min, needs Redis for live ticket_update frames with frontend polling fallback, and reaps orphaned runs on startup.
32. Help-center FAQ jobs (generation, migration, PDF import) are drained by the knowledge worker — there is no separate FAQ container, and multiple worker entrypoints share the FAQ queue.
33. Testing exists: backend pytest suite and frontend unit and e2e test commands.

llms.txt (3 claims):
34. A ChatterMate CLI (chattermate command) exists with the documented commands: signup, agent create, knowledge add-url, knowledge add-file, widget create.
35. The API serves routes under /api/v1.
36. Widget embedding works via a window.chattermateId script variable plus a webclient/chattermate.min.js script, produced by the frontend widget build.

HELP_CENTER_INFRA.md (6 claims):
37. The public help center is served by the backend itself via host-dispatched SSR (backend/app/core/help_center_host.py); frontend/nginx.conf and docker-compose files need no changes.
38. Per-org custom domains are supported with DNS verification (CNAME to cname.chattermate.chat or A record to HELP_CENTER_TARGET_IPS, plus a TXT verification token), an ssl_status lifecycle (pending → active via an https /healthz probe), and per-domain certbot issuance.
39. HELP_CENTER_TARGET_IPS is a frozenset pydantic-settings field requiring a JSON array value, and it powers per-org custom-domain A-record verification (unused by the wildcard site).
40. Help-center queries are recorded (help_center_queries; POST /ask returns an answer from the page).
41. The help_center feature is gated by a plan flag in the enterprise submodule (separate repo): PRO/ENTERPRISE True, FREE/BASE False, with the message-limit check extended to count period help_center_queries on the hosted model; self-hosted/OSS installs are unrestricted.
42. FAQ tunables exist with the stated defaults: FAQ_MAX_PAGES_PER_SOURCE=300, FAQ_MAX_BATCH_CHARS=15000, FAQ_IMPORT_MAX_PAGE_CHARS=100000, FAQ_IMPORT_FETCH_TIMEOUT=30s.

backend/app/knowledge/README.md (6 claims):
43. The knowledge module uses multi-strategy content extraction (tag/class/ID selectors, density analysis, fallback) with retries using exponential backoff, boilerplate cleaning, and parallel crawling/embedding via ThreadPoolExecutor.
44. OptimizedPgVector preserves embeddings across database operations, uses batched inserts, and optimizes vector indexing.
45. Default configuration values are as stated: max_depth 5, max_links 25, min_content_length 100, timeout 30s, max_retries 3, max_workers 10, batch_size 20.
46. crawl_scope supports host / path / domain modes (host default), and only http/https links are ever followed.
47. A migration named purge_mangled_crawl_pages_001 exists and deletes mangled non-page URLs; scripts/prune_offsite_pages.py exists and deletes only with --apply.
48. Comprehensive unit tests exist at backend/tests/knowledge/test_enhanced_website_reader.py, test_enhanced_website_kb.py, and test_optimized_pgvector.py.

Per-document counts: README.md 33 · llms.txt 3 · HELP_CENTER_INFRA.md 6 · backend/app/knowledge/README.md 6. Total 48.

One internal tension already visible from the sources themselves (to be tested, not yet a finding): knowledge README's usage example states max_depth default 3 and max_links 10, while its Configuration Options section states defaults 5 and 25 (claims 45).
