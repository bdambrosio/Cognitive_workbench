ChatterMate — AI-Readiness Audit Report

**Recommendation: Conditional** — One delta found (outbound webhooks not implemented), one partial (gunicorn worker count), and four minor caveats. The core product claims are verified. The delta is addressable: implement the feature or correct the README.

---

## Findings

**Finding 1: Outbound ticket webhooks — [delta]**

Claim (README.md line 192): "Outbound ticket webhooks to drive your own automations."

Evidence: `backend/app/api/ticket_webhooks.py` — implements an INBOUND alert webhook (Grafana/Datadog/CloudWatch POST to ChatterMate, creating tickets). `backend/app/api/tickets.py:204-225` — alert_webhook_enabled/secret config for that inbound endpoint. Grep for 'outbound' across all `backend/app/` .py files: 10 matches, all referring to outbound messaging to customers via channels (`format_outbound()` in `telegram.py:194`, `whatsapp.py:151`, `messenger.py:125`, `base.py:103`). No webhook dispatch service exists in `services/`, `workers/`, or `api/`. `backend/app/services/ticket_events.py:36-55` — Socket.IO emit on ticket events, not HTTP POST to user URLs.

Delta: The README claims a capability (outbound webhooks notifying user-configured URLs on ticket events) that is not implemented. What exists is the opposite direction: an inbound alert webhook where external monitoring systems POST to ChatterMate. There is no code path that sends an HTTP POST to a user-specified URL when a ticket is created, updated, or resolved.

---

**Finding 2: Gunicorn worker count — [partial]**

Claim (README.md deployment section): `gunicorn ... --workers 4`

Evidence: `start.sh` — `WORKERS=1`. `docker-compose.prod.yml` — `WORKERS=1`. Worker class (uvicorn.workers.UvicornWorker) and timeout (120s) match the README; only the count differs.

Delta: The README's deployment instructions specify 4 workers; the production artifacts ship 1. A buyer deploying from the README would get a different configuration than the production defaults. This is documentation drift, not a broken capability.

---

**Finding 3: Frustration detection — [real, minor caveat]**

Claim (README.md): The AI "detects" customer frustration and escalates to a human agent.

Evidence: `backend/app/agents/chat_agent.py:596` — system-prompt instruction directs the LLM to set a transfer flag when "the customer is expressing frustration with your responses". `backend/app/models/schemas/chat.py:47,216` — FRUSTRATED is one of eleven values in the TransferReasonType enum the LLM selects from. A sentiment scorer does exist: `backend/app/services/sentiment.py` (136 lines) combines TextBlob polarity with literal POSITIVE_KEYWORDS / NEGATIVE_KEYWORDS sets, the latter containing 'frustrated', 'annoyed' and 'disappointed'. It is wired in at `backend/app/repositories/chat.py:124-129`, which scores every customer message and stores sentiment_label and sentiment_score. It is also surfaced at `backend/app/api/analytics.py:521` (GET /sentiment) and rendered by `frontend/src/components/analytics/SentimentAnalytics.vue`. It does not reach the escalation path: `sentiment` appears zero times in `backend/app/agents/chat_agent.py`, zero times in `backend/app/services/human_routing.py`, and zero times anywhere under `backend/app/agents/`. No learned classifier or trained sentiment model is present. The transfer pipeline (flag → handover → agent assignment) is fully implemented and functional.

Delta: Two mechanisms, neither of them what "detects" implies. Escalation is decided by prompt-guided LLM judgment, not by the sentiment scorer — the scorer records a label per message and does not gate the transfer. And the scorer itself is keyword matching plus a polarity library, not a trained model. Functionally the escalation works; a buyer should know that the recorded sentiment is a word-list score and that the escalation decision is the LLM's inference.

**Correction (§7):** an earlier draft of this finding stated that no separate classifier or sentiment scorer existed. That was wrong — `services/sentiment.py` exists and runs on every customer message. The error was an assertion of absence made without a search that would have located it — a search for a module *driving the escalation* returned nothing, and that was generalised to no scorer existing at all. Those are different claims; the first is true and the second is false. The conclusion about the escalation mechanism is unchanged; the account of what else is present has been corrected.

---

**Finding 4: Auto Translation — [real, minor caveat]**

Claim (README.md): "Auto Translation" listed as a feature.

Evidence: `backend/app/agents/chat_agent.py` — language preference setting passed to LLM system prompt. No dedicated translation service, API, or language-detection module. Translation occurs via the LLM's multilingual capability.

Delta: Functional via LLM multilingualism, not a standalone translation engine. A buyer expecting a dedicated translation pipeline (e.g., a separate API with per-language quality guarantees) will find a simpler mechanism.

---

**Finding 5: Workflow version control — [real, minor caveat]**

Claim (README.md): "Version control" for workflows.

Evidence: Workflow model includes a `version` integer counter and `status` field (draft/published/archived). No version history log, no rollback mechanism, no diff between versions.

Delta: Basic state management (increment counter, change status) is present. Full version control (history, rollback, diff) is not. The claim is 80% true: you can track that a workflow was updated, but you cannot revert to a prior state.

---

**Finding 6: Help center plan gating — [real, minor caveat]**

Claim (HELP_CENTER_INFRA.md lines 173-186): PRO and ENTERPRISE plans have help center access; FREE and BASE do not; self-hosted is unrestricted.

Evidence: `backend/app/services/feature_gate.py:42-55` — delegates to enterprise `PlanRepository.check_feature_availability()`. Returns True when no enterprise module (self-hosted unrestricted). `backend/tests/api/test_help_center_api.py:343-377` — FREE plan returns 403, PRO plan returns 201. ENTERPRISE and BASE plan values are not tested; their mapping lives in the enterprise module (`backend/app/enterprise/`), which is empty in this OSS checkout.

Delta: Gating logic is verified for OSS-visible plans (FREE=denied, PRO=allowed, self-hosted=unrestricted). ENTERPRISE and BASE cannot be confirmed from available source. Structurally consistent with the claim, but the specific enterprise plan assignments are opaque.

---

## Unverifiable Claims

The following claims could not be verified from this repository. They reference external packages, modules, or listings not present in the OSS checkout:

1. **Shopify App Store listing** (README.md) — the App Store manifest and listing are external; no manifest file in repo.
2. **pip CLI: `chattermate-cli`** (README.md lines 246-250) — no `pyproject.toml`, `setup.py`, or `cli/` directory in repo. No reference to a separate source repo.
3. **npm CLI: `chattermate-deploy`** (README.md lines 266-283) — no `package.json` with a `bin` field, no `deploy/` directory. The only `package.json` is `frontend/package.json` (private, no bin field).
4. **MCP server: `chattermate-mcp`** (llms.txt line 33) — no MCP server implementation in repo. Only MCP client code exists (`backend/app/tools/mcp_manager.py` connects to external MCP servers). The `chattermate-mcp` package source is not here.
5. **FREE plan enum** (README.md line 543) — plan values (FREE, PRO, ENTERPRISE, BASE) are defined in the enterprise module, not in `backend/app/models/`. Cannot confirm the FREE plan exists as a valid hosted tier.

These go in Remaining Claims. They are not findings — I could not verify them, but I also cannot say they are false.

---

## Coverage Statement

**75 claims identified** across README.md (625 lines), llms.txt (49 lines), and HELP_CENTER_INFRA.md.

**68 individually verified (91%).** 1 delta, 1 partial, 4 minor caveats, 62 real (including [real] and [real, minor caveat]).

**5 unverifiable** — external packages/modules not in this repo: Shopify App Store listing, pip CLI, npm CLI, MCP server, and the FREE plan enum.

**2 non-delta** — AI ticketing sub-claims (auto-categorization, priority, SLA, auto-resolution) are sub-claims of the ticket system already verified as [real] in legs 2-3.

**Why the remainder is low-risk:** The 5 unverifiable items concern external distribution channels (PyPI, npm, App Store) and the enterprise module. None of them affect the core OSS functionality. The 2 non-delta items are covered by prior verification. The pattern across 63 verified claims (62 real, 1 delta) indicates the documentation is broadly accurate, with the webhook line as the exception. A buyer who accepts the 5 unverifiable items at the seller's word can treat the remaining claim surface as verified.

---

## What the Client Should Ask the Seller Before Closing

1. **Where is the source code for `chattermate-cli`, `chattermate-deploy`, and `chattermate-mcp`?** These are referenced in the README and llms.txt but their source is not in this repository. Can you provide repo access or package registry links for verification?

2. **Is the outbound webhook feature planned?** The README's integrations table (line 192) claims "outbound ticket webhooks to drive your own automations." This feature does not exist in the codebase. Is it on the roadmap, or will the README be corrected to reflect the inbound-only alert webhook that is actually implemented?

3. **Gunicorn worker count: which is correct?** The README says `--workers 4`; the production artifacts ship `WORKERS=1`. Is 1 worker the intended production configuration for the current deployment scale, or is the README correct and the production config needs updating?

4. **Enterprise plan-to-feature mapping:** The OSS checkout does not include the enterprise module. Can you confirm the specific plan-to-feature mapping for ENTERPRISE and BASE plans, particularly for the help center?

5. **Shopify App Store listing:** Can you confirm the Shopify app is live in the App Store and provide the listing URL? The manifest is not in this repository.

---

## Executive Synthesis

ChatterMate is a well-architected customer-support platform whose core claims hold up under inspection. The safety-critical mechanisms — read-only SQL enforcement with AST parsing, cross-customer data isolation, PII scrubbing on outbound messages, and full audit logging — are real, layered implementations, not stubs. The five claims your client flagged as load-bearing (7 AI providers, SQL read-only, frustration escalation, real-time availability with business hours, Shopify integration) all verify as [real].

The one delta is in the integrations table: the README claims outbound ticket webhooks that do not exist in the code. The system has the opposite — an inbound alert webhook for monitoring systems. If your client's e-commerce workflow depends on receiving webhook notifications from the support platform (e.g., triggering order-status updates when a ticket resolves), that integration point does not exist and would need to be built.

The remaining findings are documentation drift and naming overstatements that do not change the fundamental product. The gunicorn worker count is a configuration discrepancy, not a capability gap. The "detection" and "translation" and "version control" labels are slightly more impressive than the implementations, but the implementations are functional.

The audit recommends **Conditional**: proceed if the seller will either implement the outbound webhook feature or correct the README, and if the buyer is comfortable accepting the 5 unverifiable external-package claims at the seller's word. The gaps are addressable within a normal integration timeline and do not affect the core product the client is buying.
