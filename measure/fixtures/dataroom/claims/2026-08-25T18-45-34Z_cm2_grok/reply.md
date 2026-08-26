Claim sources treated as named: README.md, llms.txt, HELP_CENTER_INFRA.md. No other documents were used as claim sources. Nothing was verified against implementation.

=== CLAIM SURFACE ===
244 claims

Per-document: README.md 180; llms.txt 16; HELP_CENTER_INFRA.md 48.

README.md:1 — ChatterMate - Open Source AI Customer Support Chatbot Platform
README.md:9 — Open-source AI customer support platform with human handoff.
README.md:9 — A no-code AI chatbot for 24/7 customer service automation — multi-model AI (OpenAI, Anthropic Claude, Google Gemini, Mistral, xAI Grok, DeepSeek, Groq), intelligent AI-to-human handover, Shopify & e-commerce support, Slack and Jira integrations, visual workflow builder, and a fully themeable chat widget.
README.md:9 — Use the free hosted service or self-host it as an open-source alternative to Intercom, Zendesk, and Chatbase.
README.md:34 — ChatterMate is a no-code AI customer support platform that enables businesses to provide 24/7 customer service through intelligent AI agents.
README.md:34 — Your AI chatbot handles common queries, escalates complex issues to human agents when needed, and continuously learns from your knowledge base.
README.md:34 — Integrate the chat widget on any website — or any Shopify store — with a single line of code.
README.md:36 — Perfect for: e-commerce and Shopify stores, SaaS companies, help desks, customer success teams, and any business looking to automate customer support while maintaining a human touch.
README.md:48 — Intelligent AI-to-human transfer with business hours awareness, real-time availability detection, and context-aware escalation messages.
README.md:49 — Native Shopify integration (App Store listing) — answer order, shipping, and product questions from store data.
README.md:49 — Works for any online store.
README.md:50 — Official WordPress plugin — add the chat widget to any WordPress site by entering your Widget ID.
README.md:50 — No theme edits required.
README.md:51 — Choose your AI provider — OpenAI, Anthropic (Claude), Google Gemini, Mistral, xAI (Grok), DeepSeek, and Groq — with your own API key, or enter a custom model ID.
README.md:51 — Switch providers anytime without code changes.
README.md:52 — Let visitors start conversations instantly — no signup or email required.
README.md:53 — Customers can share images, PDFs, Word docs, spreadsheets, and more directly in chat.
README.md:53 — Secure uploads with S3 storage and magic byte validation.
README.md:54 — Multilingual support with configurable default language per workflow.
README.md:54 — Serve customers globally in their preferred language.
README.md:55 — Native tickets raised straight from chat, then auto-triaged and investigated by AI — it forms hypotheses, gathers evidence from your observability tools and databases, and writes a root-cause analysis you can read and audit.
README.md:56 — Create and manage Jira tickets directly from chat conversations.
README.md:56 — OAuth 2.0 secure integration with automatic ticket tracking.
README.md:56 — Native tickets can also escalate to Jira one-way.
README.md:57 — Token-based security for embedded widgets.
README.md:57 — Support both public Q&A and private authenticated conversations.
README.md:58 — Connect your Slack workspace for internal product support.
README.md:58 — Teams get AI-powered assistance directly in Slack channels.
README.md:59 — Design conversation flows with a drag-and-drop interface.
README.md:59 — Branching logic, conditional responses, and multi-step workflows without coding.
README.md:63 — Build sophisticated conversation flows visually with our intuitive workflow builder.
README.md:68 — Drag-and-drop nodes - AI responses, human handoff, conditions, and more
README.md:69 — Branching logic - Create different paths based on user input
README.md:70 — Node types - Start, AI Response, Human Transfer, Condition, End nodes
README.md:71 — Real-time preview - Test workflows before deploying
README.md:72 — Version control - Save and restore workflow versions
README.md:76 — Tickets are raised from a conversation (by the AI or an agent), or from an alert webhook.
README.md:77 — Each one is triaged, deduplicated against open tickets, and then investigated: the AI proposes hypotheses, tests each against your connected tools, and records every query it ran as evidence you can inspect.
README.md:79 — The result is a versioned root-cause analysis with citations back to that evidence.
README.md:82 — Connect read-only investigation sources under Settings → Ticketing: observability platforms via MCP (Grafana, Elasticsearch, Sentry, CloudWatch, or any MCP server), and optionally a guardrailed SQL connector (Postgres/MySQL, direct or over an SSH tunnel).
README.md:87 — Autonomy is staged, and you choose the level.
README.md:91 — L1: Investigate and document only
README.md:92 — L2: Also propose a resolution — a human approves or rejects it
README.md:93 — L3: Also message the customer and close the ticket, behind confidence guards
README.md:97 — The investigation agent reads untrusted customer text and holds live tool access, so the limits are enforced in code, not by prompting.
README.md:100 — The SQL connector is read-only, structurally.
README.md:100 — Queries are parsed to an AST and rejected unless they are a single plain SELECT.
README.md:101 — Writes, DDL, multiple statements, and a denylist of dangerous functions cannot pass.
README.md:102 — Table access is restricted to an allowlist you pick, enforced through CTEs, joins and subqueries.
README.md:104 — A LIMIT is forced.
README.md:104 — Comments are stripped before execution, since MySQL executes /*! */.
README.md:105 — The connection itself is a read-only transaction with a statement timeout — a second, independent barrier.
README.md:107 — Columns you mask are never readable.
README.md:107 — Masked columns are blocked from being referenced anywhere, including in a WHERE clause; whole-row tricks are blocked too, and results are masked again on the way back.
README.md:111 — Nothing is written to your database, ever.
README.md:111 — Approving an AI proposal records the decision — any change to your systems is made by your team.
README.md:113 — Row-level scoping keeps one customer's data out of another's ticket.
README.md:115 — Queries against it are rewritten to read only the ticket customer's own rows — the AI cannot widen that.
README.md:118 — A ticket with no known customer cannot query a scoped table at all, rather than falling back to reading everything.
README.md:119 — Tables you leave unscoped stay fully readable.
README.md:121 — Cross-customer isolation on outbound messages.
README.md:121 — Identifiers belonging to anyone other than the recipient are stripped from every message sent to a customer, and other customers' tickets are redacted before they reach the model.
README.md:125 — Every query is audited.
README.md:125 — Each attempt is logged with the SQL and outcome; returned rows are deliberately never stored.
README.md:128 — When you connect a database: set a row-scope column for every table that holds per-customer rows — it's the control that stops the agent reading across customers, and it's off until you set it.
README.md:138 — Context-aware AI with conversation memory across sessions
README.md:139 — Real-time monitoring, conversation insights, and performance metrics
README.md:140 — Train your AI with domain-specific knowledge and FAQs
README.md:141 — Fully customizable chat widget to match your brand
README.md:142 — Granular permissions for team members
README.md:143 — Full control over your data with self-hosting option
README.md:147 — All of these are free and included — every adapter ships in the open-source codebase, and self-hosting unlocks the lot.
README.md:150 — What some of them need is your own credentials, because the integration talks to your account on someone else's platform.
README.md:153 — You create the app — a developer app registered once per ChatterMate install, with its keys in the backend .env.
README.md:155 — You paste a token — no app to create; you generate a token or API key on the provider's side and enter it in the ChatterMate UI when connecting.
README.md:160 — Messaging channels — one shared inbox for all of them, with the same AI answers, human handoff, and workflows as the web widget.
README.md:165 — WhatsApp Business Cloud API — templates, media, and 24-hour session handling
README.md:166 — Meta Messenger Platform, connected per Page
README.md:167 — Instagram Direct Messages via the Meta Graph API
README.md:168 — Your Slack workspace as an internal support channel
README.md:169 — Telegram: Bot token from BotFather — no app registration
README.md:170 — LINE Messaging API channel credentials
README.md:171 — SMS: Your Twilio, Vonage, Plivo, or MessageBird account keys
README.md:172 — Email: inbound email becomes a conversation and replies go back over email
README.md:173 — The embeddable widget, plus Shopify and WordPress surfaces
README.md:175 — CRM — push captured leads and contacts out to your sales stack.
README.md:179 — HubSpot: Contacts and leads pushed from chat, OAuth connected
README.md:180 — Pipedrive: Persons and leads pushed from chat, OAuth connected
README.md:182 — Synced by the crm_sync worker.
README.md:188 — Create and track Jira issues from chat (OAuth 2.0); native tickets can escalate to Jira
README.md:189 — Shopify: Order, shipping, and product answers from live store data
README.md:190 — Any MCP server as an agent tool — Grafana, Elasticsearch, Sentry, CloudWatch, and others
README.md:191 — Read-only, guardrailed PostgreSQL and MySQL access for AI ticket investigation
README.md:192 — Outbound ticket webhooks to drive your own automations
README.md:194 — Adding a channel means implementing one adapter in backend/app/channels/ against the shared ChannelAdapter base — the inbox, AI, and handoff come for free.
README.md:196 — CRM adapters follow the same shape in backend/app/crm/.
README.md:202 — ChatterMate is a free, open-source alternative to Intercom, Zendesk AI, and Chatbase — with AI answers and human handoff in one inbox.
README.md:206 — Open source (Apache-2.0)
README.md:207 — Self-hosting / data ownership
README.md:208 — AI answers from your knowledge base
README.md:209 — Built-in human handoff + shared inbox
README.md:210 — Bring your own AI model (OpenAI, Claude, Gemini, Grok, +more)
README.md:211 — Visual no-code workflow builder
README.md:212 — Free tier
README.md:213 — Per-AI-resolution fees: none
README.md:235 — There are two ways to run ChatterMate.
README.md:235 — Most people want the hosted service — sign up and manage everything from the dashboard, the CLI, or an AI agent.
README.md:236 — Self-host with Docker only if you need to run ChatterMate on your own infrastructure.
README.md:241 — Sign up: app.chattermate.chat — free, no card required.
README.md:242 — Manage from your terminal, automate, or drive it with an AI agent using the ChatterMate CLI.
README.md:245 — The ChatterMate CLI — sign up, mint tokens, and manage agents, workflows & knowledge.
README.md:246 — pip install chattermate-cli installs the chattermate command.
README.md:262 — Run the full ChatterMate stack on your own infrastructure with the self-host CLI (Docker-based).
README.md:268 — Scaffold a project, then start the full stack (Postgres, Redis, backend, frontend, worker).
README.md:271 — chattermate-deploy start then open http://localhost/
README.md:287 — Two different chattermate commands — don't mix them up.
README.md:287 — The hosted CLI (pip install chattermate-cli) signs you up and manages agents/knowledge against the ChatterMate API.
README.md:289 — The self-host CLI (npm install -g chattermate-deploy) scaffolds and runs the Docker stack.
README.md:290 — They are separate tools that happen to share the chattermate name.
README.md:298 — The methods below are for self-hosting.
README.md:303 — For Self-Host CLI (Recommended): Node.js 16+
README.md:304 — For Self-Host CLI (Recommended): Docker & Docker Compose
README.md:305 — For Self-Host CLI (Recommended): npm or yarn
README.md:308 — For Manual Installation: Python 3.12+
README.md:309 — For Manual Installation: PostgreSQL 14+ (with Vector extension)
README.md:310 — For Manual Installation: Firebase Project (for push notifications)
README.md:311 — For Manual Installation: Redis (optional, for rate limiting and multi-server socket deployment)
README.md:315 — The self-host CLI (chattermate-deploy, installed via npm) scaffolds and runs the Docker stack.
README.md:316 — This is not the account CLI.
README.md:372 — For Web Push notification, generate a firebase config and place it in backend/app/config/firebase-config.json
README.md:391 — The API server only enqueues long-running jobs — a separate worker process executes them.
README.md:392 — If no worker is running, jobs sit at pending forever and the UI shows a progress bar that never advances.
README.md:397 — Knowledge + FAQ worker handles knowledge ingestion (crawl, PDF, sitemap) and FAQ jobs.
README.md:398 — Ticket investigator handles AI ticket investigation + ticket lifecycle.
README.md:399 — CRM sync handles lead push to HubSpot / Pipedrive.
README.md:401 — Run each from the backend/ directory with the virtualenv active.
README.md:403 — FAQ generation and import jobs are drained by the knowledge worker — there is no separate FAQ container.
README.md:405 — Both knowledge entrypoints pick up the FAQ queue, so either one is enough.
README.md:409 — Recommended for local dev — knowledge and FAQ in one loop.
README.md:412 — What Docker runs: knowledge and FAQ as independent asyncio tasks, so a long FAQ generation never blocks knowledge ingestion.
README.md:416 — FAQ queue only — useful when debugging generation/import in isolation.
README.md:420 — Polls every 60s.
README.md:420 — On startup it fails any job left stuck in processing by a previous crash, so a killed worker doesn't strand a job forever.
README.md:423 — Runs two loops in one process — investigation runs (polled every 15s) and ticket lifecycle/SLA sweeps (every 5 min).
README.md:430 — Needs Redis (REDIS_URL) to push live ticket_update frames to dashboard clients; without it the frontend falls back to polling.
README.md:431 — Orphaned runs are reaped on startup.
README.md:434 — Under Docker, these already run as services — docker compose up starts them alongside the API.
README.md:475 — Pre-built Docker images are available.
README.md:540 — ChatterMate is an open-source AI customer support platform.
README.md:540 — It combines an AI chatbot (trained on your knowledge base) with human agents in a shared inbox, so AI handles routine questions 24/7 and hands off to your team when it matters.
README.md:543 — The core platform is Apache-2.0 licensed — free for personal and commercial use, including self-hosted production deployments.
README.md:543 — There's also a free hosted plan at app.chattermate.chat.
README.md:546 — Human handoff is a core feature, not an add-on.
README.md:546 — ChatterMate detects frustration or explicit requests for a human and transfers the conversation with full context, respecting business hours and agent availability.
README.md:549 — Install it from the Shopify App Store to answer order-status, shipping, and product questions directly from your store data.
README.md:549 — The widget also embeds on any other e-commerce or website platform with one line of code.
README.md:552 — Download the WordPress plugin, install it via Plugins → Add New → Upload Plugin, then enter your Widget ID under Settings → ChatterMate Chat.
README.md:552 — The chat launcher appears on your site with no theme edits.
README.md:552 — A WordPress.org directory listing is in progress.
README.md:555 — Run the full stack (Postgres, Redis, backend, frontend) on your own infrastructure with npm install -g chattermate-deploy.
README.md:555 — Self-hosting gives you complete data ownership.
README.md:558 — OpenAI, Anthropic (Claude), Google Gemini, Mistral, xAI (Grok), DeepSeek, and Groq — bring your own API key, or enter a custom model ID for any model a provider supports.
README.md:558 — You can switch providers at any time without code changes.
README.md:566 — Coming soon: Auto Follow-up System — Automated follow-ups for idle customers
README.md:567 — Coming soon: Customer Contact Management — CRM-like contact organization
README.md:568 — Coming soon: Human Agent AI Suggestions — AI-powered response suggestions for agents
README.md:569 — Coming soon: AI Voice Chat — Voice-enabled AI conversations
README.md:570 — Coming soon: More Integrations — Zendesk, Intercom, and more
README.md:594 — Documentation: docs.chattermate.chat
README.md:595 — Issues: GitHub Issues
README.md:596 — Email: support@chattermate.chat
README.md:602 — ChatterMate follows an open-core model.
README.md:605 — Free for personal and commercial use — including proprietary and SaaS deployments
README.md:606 — No source-disclosure or copyleft obligations
README.md:607 — Includes an explicit patent grant
README.md:611 — Advanced/enterprise capabilities are provided separately under a commercial arrangement
README.md:612 — Priority support, warranties, and indemnification available
README.md:613 — Contact: contact@chattermate.chat
README.md:615 — Contributions are accepted under Apache-2.0 with a Developer Certificate of Origin sign-off.
llms.txt:3 — ChatterMate is an open-source AI customer-support chatbot platform.
llms.txt:4 — You can sign up, create AI agents, add knowledge sources, and embed a chat widget — all from a browser, the terminal (CLI), or programmatically via an AI agent over MCP.
llms.txt:9 — Use the ChatterMate CLI (the chattermate command, installed with pip install chattermate-cli) to sign up and manage everything from the terminal.
llms.txt:10 — It is the account/config CLI — distinct from the self-host Docker tool (npm install -g chattermate-deploy).
llms.txt:31 — AI Agent & Automation Quickstart: copy-paste path from signup to a live widget, built for AI agents and CI.
llms.txt:32 — CLI reference: the account CLI — signup, login, tokens, agents, workflows, knowledge.
llms.txt:33 — MCP Server: let an AI agent configure ChatterMate over the Model Context Protocol (chattermate-mcp).
llms.txt:34 — Quickstart: hosted vs self-host, side by side.
llms.txt:35 — Widget integration: embed the chat widget on any site.
llms.txt:36 — Widget Apps & API keys: authenticated widgets and programmatic tokens.
llms.txt:40 — Sign up / dashboard: https://app.chattermate.chat
llms.txt:41 — Hosted API base: https://api.chattermate.chat (routes under /api/v1)
llms.txt:42 — Widget script: https://app.chattermate.chat/webclient/chattermate.min.js
llms.txt:43 — Documentation: https://docs.chattermate.chat
llms.txt:44 — GitHub: https://github.com/chattermate/chattermate.chat
llms.txt:48 — Self-host with Docker: run the full stack on your own infrastructure with the self-host CLI (npm install -g chattermate-deploy, command chattermate-deploy init/start/stop).
HELP_CENTER_INFRA.md:3 — The public help center is served by the backend itself (host-dispatched SSR — see backend/app/core/help_center_host.py).
HELP_CENTER_INFRA.md:4 — This runbook covers the deploy-time pieces that live outside the repo: DNS, TLS and the host nginx on the production VPS.
HELP_CENTER_INFRA.md:6 — frontend/nginx.conf and the docker-compose files need no changes.
HELP_CENTER_INFRA.md:9 — As deployed on the ChatterMate cloud VPS (2026-07-14): the base domain is help.chattermate.chat (orgs serve at {slug}.help.chattermate.chat), not chattermate.help — that domain was never registered, and the org only owns chattermate.chat.
HELP_CENTER_INFRA.md:12 — DNS is Route53, so the wildcard cert is issued and auto-renewed with the dns-route53 plugin.
HELP_CENTER_INFRA.md:14 — Placeholders stand in for the live values; the real ones live on the VPS and in ops notes, not in this public repo.
HELP_CENTER_INFRA.md:20 — On the cloud box this is the chattermate.chat hosted zone.
HELP_CENTER_INFRA.md:25 — One wildcard record covers every org — no per-org DNS.
HELP_CENTER_INFRA.md:36 — HELP_CENTER_TARGET_IPS is a frozenset setting, so pydantic-settings JSON-parses the env value.
HELP_CENTER_INFRA.md:37 — A bare IP or comma list fails validation and crash-loops the backend at startup — it must be a JSON array.
HELP_CENTER_INFRA.md:39 — Powers the per-org custom-domain A-record verification in §2; unused by the wildcard site.
HELP_CENTER_INFRA.md:43 — HTTP-01 cannot issue wildcards.
HELP_CENTER_INFRA.md:43 — A dedicated IAM user with the standard certbot Route53 policy has its creds at ~/.aws/credentials (mode 600) on the box.
HELP_CENTER_INFRA.md:57 — Auto-renewal: a daily cron runs the compose certbot service as a one-shot, then reloads nginx only if a cert actually changed.
HELP_CENTER_INFRA.md:65 — It uses the certbot/dns-route53 image and mounts /home/chattermate/.aws:/root/.aws:ro, so the wildcard renews unattended.
HELP_CENTER_INFRA.md:67 — Existing HTTP-01 webroot certs still renew — the dns-route53 image is a superset of certbot/certbot.
HELP_CENTER_INFRA.md:69 — renew covers every cert in the shared certbot/conf, not just the wildcard.
HELP_CENTER_INFRA.md:71 — The certbot service must stay one-shot — never give it a long-running loop entrypoint.
HELP_CENTER_INFRA.md:72 — It previously used a long-running loop entrypoint which broke renewal in two non-obvious ways.
HELP_CENTER_INFRA.md:76 — compose run inherits the entrypoint.
HELP_CENTER_INFRA.md:76 — The one-off cron run never exited, so --rm never fired (a container leaked every night) and the cron's chained nginx reload never ran at all.
HELP_CENTER_INFRA.md:79 — sh -c '<script>' <args> silently discards the args.
HELP_CENTER_INFRA.md:80 — So it renewed without the --deploy-hook, every 12h — beating the daily cron to the renewal.
HELP_CENTER_INFRA.md:82 — The cron's own renew then found nothing due, so the hook never fired.
HELP_CENTER_INFRA.md:84 — Net effect: certs renewed on disk while nginx kept serving the old ones, masked only by deploys happening to restart nginx.
HELP_CENTER_INFRA.md:85 — Left alone that expires a live cert.
HELP_CENTER_INFRA.md:86 — The service is therefore declared with no entrypoint and profiles: ["tools"], so compose up -d never starts it while compose run still works.
HELP_CENTER_INFRA.md:98 — Lives at ~/chattermate/nginx/conf.d/help-center.conf (mounted read-only into the chattermate-nginx-1 container).
HELP_CENTER_INFRA.md:99 — proxy_set_header Host $host is what makes backend host-dispatch resolve the org.
HELP_CENTER_INFRA.md:100 — Upstream is the compose service name backend:8000 (nginx shares the chattermate-network), not a published host port.
HELP_CENTER_INFRA.md:132 — Customers add two records (shown in their FAQ admin UI).
HELP_CENTER_INFRA.md:132 — On the cloud box the CNAME target cname.chattermate.chat is not provisioned, so verification falls back to an A record pointing at HELP_CENTER_TARGET_IPS — either form works.
HELP_CENTER_INFRA.md:139 — After they click Verify domain (DNS checks pass, ssl_status becomes pending), issue the certificate on the VPS — HTTP-01 works because the CNAME already routes to us.
HELP_CENTER_INFRA.md:149 — The backend's SSL probe (https://<domain>/healthz) flips ssl_status to active on the customer's next verify/status check.
HELP_CENTER_INFRA.md:152 — Cloudflare-proxied (orange cloud) CNAMEs hide the record from DNS lookups and terminate TLS at Cloudflare — customers must grey-cloud the record for verification, or the CNAME check will fail.
HELP_CENTER_INFRA.md:157 — When manual certbot becomes a burden: a host cron that polls a token-guarded admin endpoint for verified-but-unprovisioned domains — or move edge TLS to Caddy with on-demand TLS. Deferred.
HELP_CENTER_INFRA.md:160 — Deferred because Caddy would have to own port 443 for the whole VPS.
HELP_CENTER_INFRA.md:164 — https://<slug>.help.chattermate.chat/ → 200 (and http:// → 301).
HELP_CENTER_INFRA.md:165 — Published FAQs render, widget loads; an article at /a/<faq-slug> returns 200.
HELP_CENTER_INFRA.md:166 — HELP_CENTER_BASE_DOMAIN → help.chattermate.chat (and the backend is healthy, not crash-looping on HELP_CENTER_TARGET_IPS).
HELP_CENTER_INFRA.md:167 — API unaffected: app.chattermate.chat / api.chattermate.chat/api/v1/... still route normally.
HELP_CENTER_INFRA.md:168 — POST /ask from the page returns an answer and appears in help_center_queries.
HELP_CENTER_INFRA.md:169 — docker ps -a --filter name=certbot → empty.
HELP_CENTER_INFRA.md:169 — compose up -d must not start a certbot container; one that sticks around means the long-running loop entrypoint is back and nginx will stop reloading onto renewed certs.
HELP_CENTER_INFRA.md:175 — The feature is gated by the help_center plan flag, which lives in the enterprise submodule (separate repo/PR — required before cloud rollout).
HELP_CENTER_INFRA.md:178 — add "help_center": True to PRO and ENTERPRISE in enterprise/models/plan.py::get_default_features (FREE/BASE: False).
HELP_CENTER_INFRA.md:185 — Until that PR deploys, cloud orgs see the upgrade lock (plan_allowed: false); self-hosted/OSS installs are unrestricted immediately.
HELP_CENTER_INFRA.md:190 — HELP_CENTER_RESERVED_SLUGS, FAQ_MAX_PAGES_PER_SOURCE (300), FAQ_MAX_BATCH_CHARS (15000), FAQ_IMPORT_MAX_PAGE_CHARS (100000), FAQ_IMPORT_FETCH_TIMEOUT (30s) — the FAQ_* caps bound LLM spend per generation run.

Surface is frozen. Stop.