**ChatterMate** — Open-source AI customer-support platform (chat widget, AI agent with human handover, Shopify + multi-channel integrations). Python/Vue, ~1,100 tracked files.

**Conditional**

---

**Key items:**

⚠️ **Outbound ticket webhooks — [delta].** README line 192 claims outbound webhooks to user-configured URLs on ticket events. Not implemented. The codebase has the opposite: an inbound alert webhook (Grafana/Datadog/CloudWatch → ChatterMate). If your workflow needs webhook notifications from the platform, this must be built.

⚠️ **Gunicorn workers — [partial].** README says `--workers 4`; production ships `WORKERS=1`. Documentation drift, not a capability gap. A buyer deploying from the README gets a different config than the production defaults.

✅ **SQL connectors — [real].** Read-only enforcement is a real, layered implementation: AST parsing, single-SELECT restriction, table allowlist, forced LIMIT, comment stripping, read-only transaction. Masked columns, row scoping, and cross-customer isolation all verified.

✅ **7 AI providers, switchable without code changes — [real].** All 7 providers (OpenAI, Anthropic, Google, xAI, Groq, Ollama, Mistral) are implemented as interchangeable adapters. Provider selection is a per-agent config field; no code changes needed to switch.

✅ **Shopify integration — [real].** Native integration with order, shipping, and product data access via Shopify Admin API. OAuth flow, token refresh, and per-store configuration all implemented.

---

**Coverage:** 68/75 claims individually verified (91%). 5 unverifiable (external packages: PyPI CLI, npm CLI, MCP server, Shopify App Store, enterprise plan enum). 2 non-delta (sub-claims of verified systems). No claims were skipped within the available source.

*Full report with citations available on request.*

*technical claims verification · not a pen-test, not legal advice*
