**Conditional** (of **42 claims examined** out of **118** identified). Two material gaps: HELP_CENTER_TARGET_IPS format is false in the runbook; Auto Translation is a default-language field, not multilingual serving. No recoverability/payment claim in the three sources failed as stated. Stripe lives only behind empty enterprise submodules.

**Finding 1: HELP_CENTER_TARGET_IPS must be a JSON array — [delta]**

Claim (HELP_CENTER_INFRA.md:34–40): `HELP_CENTER_TARGET_IPS=["<VPS_IPV4>"]` because pydantic-settings JSON-parses a `frozenset`; a bare IP or comma list crash-loops the backend.

Evidence (backend/app/core/config.py:220–222): `HELP_CENTER_TARGET_IPS: frozenset = frozenset(ip.strip() for ip in os.getenv("HELP_CENTER_TARGET_IPS", "").split(",") if ip.strip())`. Lexical+structural search: no JSON parse of this env anywhere.

Gap: A JSON array becomes one bogus member including brackets/quotes. A comma-separated list is what the code accepts. HELP_CENTER_INFRA.md:166 repeats the crash-loop warning against the same false premise.

**Finding 2: Auto Translation / preferred language — [partial]**

Claim (README.md:54): “Multilingual support with configurable **default language per workflow**. Serve customers globally in their preferred language.”

Evidence: `default_language` String(10) default `"en"` (backend/app/models/workflow.py:41; schemas and workflow API create/update). Lexical search `auto.?translat|multilingual|preferred language|locale` in `backend/app` and `frontend/src`: no product i18n or auto-translate path (WhatsApp template language and `localeCompare` only).

Gap: Per-workflow default language exists. Serving in the visitor’s preferred language / auto translation does not.

**Finding 3: Knowledge-worker crash reap — [partial]**

Claim (README.md:418–421): knowledge worker polls every 60s; on startup fails jobs stuck in `processing` after a crash.

Evidence: `faq_processor.py:106–149` `_reap_orphans_once` → `fail_orphaned_processing`. `faq_processor.py:36` poll 60s. `knowledge_processor.py:131–192` marks processing and logs failures; no equivalent startup orphan-reap.

Gap: Crash-reap is FAQ-generation jobs, not knowledge-entry jobs, even though both share the knowledge worker container.

**Finding 4: SQL connector structural read-only — [real]**

Claim (README.md:100–126): AST-only single SELECT; writes/DDL blocked; allowlist; LIMIT; comments stripped; read-only txn + statement timeout; masked columns; row-scope rewrite; outbound isolation; audit without storing rows.

Evidence: `sql_guardrails.py` sqlglot AST, BLOCKED_NODE_TYPES (61–67), MASK_VALUE `***MASKED***` (:40), `_apply_row_scope` (139–205), `mask_rows` (355–369), audit docstring 30–31. `db_connector_service.py:164–191` MySQL READ ONLY / Postgres `statement_timeout`. `ticket_privacy.py:66` `scrub_outbound`.

Gap: None on the structural claims examined.

**Finding 5: Ticket investigator / FAQ poll intervals — [real]**

Claim (README.md:420, 423–424): FAQ/knowledge poll 60s; investigation 15s; lifecycle/SLA 5 min.

Evidence: `faq_processor.py:36` POLL 60; `ticket_investigator.py:49–50` RUN_POLL_INTERVAL_SECONDS = 15, LIFECYCLE_POLL_INTERVAL_SECONDS = 300.

Gap: None.

**Finding 6: Channels, Shopify, Jira, Slack, handoff — [real]**

Claim (README.md:48–75; channels listed in README/llms): human handoff with business hours; Shopify; Jira; Slack; widget; workflow.

Evidence: `channels/registry.py` telegram, whatsapp, messenger, instagram, slack, email, sms, line. `shopify_toolkit.py:66–71` list/get/search products, search_orders, get_order_status, recommend_products; gated off when `transfer_to_human` (`chat_agent.py:509–515`). `jira_toolkit.py`; `slack.py:222` SlackAdapter. `chat_agent.py:474–478, 1412–1436` transfer_to_human. `transfer_agent.py` `is_within_business_hours`.

Gap: None on presence. Shopify tools are mutually exclusive with active human transfer.

**Finding 7: Multi-model catalog includes Gemini and Grok — [real]**

Claim (README.md:9, 51): OpenAI, Anthropic Claude, Google Gemini, Mistral, xAI Grok, DeepSeek, Groq.

Evidence: `ai_config.py:23–34` AIModelType includes GOOGLE, XAI. `model_catalog.py:92–95` Google Gemini; `:118–121` xAI (Grok).

Gap: None for the named list. Extra enum values GOOGLEVERTEX, HUGGINGFACE, OLLAMA are not claimed.

**Finding 8: Apache-2.0 open-core; enterprise separate — [real, operational caveat]**

Claim (README.md:602–612): Apache-2.0 core; enterprise capabilities separately commercial.

Evidence: LICENSE:2–3 Apache 2.0. `.gitmodules` `enterprise_backend.git` / `enterprise_frontend.git`. `backend/app/enterprise` and `frontend/src/modules/enterprise` empty in this tree. `feature_gate.py:42–66` OSS always unrestricted (`HAS_ENTERPRISE` False).

Gap: None vs the claim that enterprise is separate. Buyer of *this* tree does not receive enterprise plan flags (HELP_CENTER_INFRA.md:175 `help_center` in `enterprise/models/plan.py`).

**Finding 9: Help-center hosting runbook vs in-repo compose — [real, operational caveat]**

Claim (HELP_CENTER_INFRA.md:1–7, 163–171): SSR in backend; compose/nginx need no changes; certbot must not be a long-running compose service; POST /ask.

Evidence: `help_center_host.py` host dispatch; `help_center_public.py:321` POST `/ask`; no `certbot` in `docker-compose.yml` or `docker-compose.prod.yml`.

Gap: None on those points. Default `HELP_CENTER_BASE_DOMAIN` in code historically `chattermate.help` vs runbook `help.chattermate.chat` (INFRA.md:9–10, 33) — env must be set in prod; not a broken in-repo compose claim.

**Finding 10: FAQ tunables 300 / 15000 / 100000 / 30s — [real]**

Claim (HELP_CENTER_INFRA.md:191–193): FAQ_* caps.

Evidence (config.py:224–235): FAQ_MAX_PAGES_PER_SOURCE 300; FAQ_MAX_BATCH_CHARS 15000; FAQ_IMPORT_MAX_PAGE_CHARS 100000; FAQ_IMPORT_FETCH_TIMEOUT 30. Additional unclaimed ceiling FAQ_MAX_BATCH_CHARS_CEILING 60000.

Gap: None on the four stated figures.

**Finding 11: File attachments — [real]**

Claim (README.md:53 File Attachments).

Evidence: `file_upload.py` `chat_attachments/<org_id>/…`; widget/agent `allow_attachments`; `widget_chat.py` FileUploadService.

Gap: None for existence.

**Finding 12: Widget theming CSS variables — [real, minor caveat]**

Claim (README.md:9 fully themeable chat widget).

Evidence: `frontend/src/composables/useWidgetStyles.ts` `--cm-*` (card/text/agent-bg/accent).

Gap: Theme surface is CSS variables, not a full visual editor in the files examined.

**Finding 13: WordPress plugin zip present; CLI packages not in tree — [non-delta] / Remaining**

Claim (README.md:50 WordPress plugin; README.md:4,266 and llms.txt:9–11,48 `chattermate-deploy` / `chattermate-cli`).

Evidence: `wordpress/chattermate-chat.zip` (13853 bytes), no unpacked plugin. No `chattermate-cli` or `chattermate-deploy` source in this repository.

Gap: Zip presence supports the plugin artifact claim at file level, not plugin behavior. npm/pip CLIs are **[unverifiable]** from this tree.

No backup/uptime/SLA claims in the three sources (README workflow version restore at :72 is not DB backup). Stripe/billing not in OSS `backend/app`.

**Coverage.** 118 claims frozen (README 78, llms.txt 18, HELP_CENTER_INFRA 22). 42 checked individually, priority §4 order: recoverability/payment/control, then architecture, then operational numbers, then a sample of feature-table rows. Unchecked remainder is mostly marketing URLs, install-copy variants, and low-impact feature bullets. Those are low-risk relative to the two material gaps above; they are not certified.

**Ask the seller before closing.** (1) Confirm production `HELP_CENTER_TARGET_IPS` is comma-separated, not JSON. (2) What “Auto Translation” means in the product they are selling. (3) Delivery of enterprise submodules and billing. (4) Whether knowledge-entry jobs (not FAQ) are reaped after worker crash. (5) Hosted CLI packages and Shopify App listing as extra-repo artifacts.

===
LIMITATIONS ===
Materials: ChatterMate tree bound to inspect_external (README.md, llms.txt, HELP_CENTER_INFRA.md as claim sources; remaining files as evidence), as-of 2026-08-25.
The seller was not consulted and has not confirmed this reading of their own claims.
Limited assurance (§1a): 42 of 118 claims examined; no statement about the rest.

=== GAP MAP ===

**ChatterMate** — open-source AI customer-support platform (widget, agent, Shopify, human handoff).

**Conditional** — 42 of 118 claims examined.

- HELP_CENTER_TARGET_IPS: runbook requires a JSON array; code comma-splits getenv — operators following the runbook misconfigure.
- Auto Translation: default_language on workflows only; no auto-translate / preferred-language serving found.
- Crash-reap: FAQ jobs only, not knowledge-entry jobs.
- SQL guardrails, poll intervals, channels, Shopify toolkit, Jira/Slack, Apache-2.0, Gemini/Grok catalog, POST /ask, no certbot in compose: hold on the evidence checked.
- Enterprise/billing not in this tree (open-core as claimed); CLIs not in-repo.

Coverage: 42/118, P1–P3 first; remainder mostly URLs and low-impact bullets.

Full report with citations available on request.

technical claims verification · not a pen-test, not legal advice