# Financial-analysis tooling for Jill — design notes & resume point

Status: **implemented 2026-06-19** — `src/tools/get-financial-statements/`
ships Decision 1 (Alpha Vantage fundamentals on the existing key, stock-price
tool shape, free-tier throttle handling). The PDF-inspection half was
redirected to the existing `src/chat/subagents/code_subagent.py` rather than built as a
separate tool, per Decision 2. Retained as the design rationale.
(Originally: design only, captured 2026-06-17.)

## Origin

Jill's "notice deficiency" / capability-gap concern (self-extension Phase 2a)
fired during a conversation about doing IB-style financial analysis of public
companies. Jill identified two friction points and proposed tools in words:

1. **Data cleaning** — getting standardized financial statements without
   scraping messy PDFs. Jill proposed wrapping **FMP (Financial Modeling Prep)**.
2. **Deep document parsing** — pulling specific sections out of 200-page 10-Ks
   without loading the whole thing into context. Jill proposed **`pdf-grep`**.

Both proposals were reviewed and **redirected** (see decisions below). Jill's
draft code is in the transcript but should NOT be used as-is (PyPDF2 deprecated
and bad at tables; `str(data[:5])` emits Python repr not JSON; hardcoded API
key; no env-var reuse).

## Decision 1 — Financial statements via existing Alpha Vantage account, NOT FMP

`src/tools/stock-price/tool.py` already calls Alpha Vantage (`GLOBAL_QUOTE`)
using `ALPHA_VANTAGE_API_KEY`. The **same account/key** covers the fundamentals
endpoints — `INCOME_STATEMENT`, `BALANCE_SHEET`, `CASH_FLOW`, `EARNINGS`,
`OVERVIEW` — returning standardized annual+quarterly JSON. That is the exact
"removes the data-cleaning bottleneck" win Jill wanted from FMP, with **no new
account, key, or dependency**.

**Plan:** build a second tool `get_financial_statements` mirroring the existing
`react_invoke → tool → _fetch → _create_note` shape in `stock-price/tool.py`;
swap the `function=` param and the normalize step. Reuse `ALPHA_VANTAGE_API_KEY`
and `BASE_URL`.

**CONFIRMED live 2026-06-17** against the actual key (`ALPHA_VANTAGE_API_KEY`,
present in env): `INCOME_STATEMENT`/`BALANCE_SHEET`/`CASH_FLOW` all return real
data on the free key — NOT premium-gated. Free-tier limits verified:
**25 requests/day, ~5/min, and a ~1 req/sec burst cap.** Proven that the
per-second burst (not the daily 25) is what throttles: 3 back-to-back curls
threw the throttle on calls 2–3, but the same call in isolation returned full
data.

Tradeoffs / constraints:
- AV fundamentals are less granular than FMP — adequate for the 3 core
  statements; add FMP later only if a specific analysis hits a wall.
- **Daily cap is the real ceiling.** One company ≈ 5 calls (3 statements +
  OVERVIEW + EARNINGS); a peer set of 3–5 companies ≈ 15–25 calls → roughly
  **one full peer-set analysis per day** on the free tier. Jill should sequence
  calls deliberately, not fan out.
- **Tool must self-throttle:** space calls ≥1s apart, stay <5/min.
- **Throttle detection needs a new key.** The throttle arrives as
  `{"Information": "..."}`. `stock-price/tool.py` `_fetch_quote` only checks
  `data.get("Error Message") or data.get("Note")` — it would MISS `Information`
  and parse it as data. The new tool must treat an `Information` key as a soft
  failure (back off, retry once after ~1s).

Do NOT factor out shared AV plumbing yet — only two call sites; premature
abstraction.

## Decision 2 — Document inspection is a *subagent*, NOT `pdf-grep`

We already have a file-set subagent: `src/chat/subagents/code_subagent.py`, exposed as
`inspect` (own `src/`) and `inspect_external` (a bound project repo). Primitives:
**list / read / grep**, geofenced root with path-traversal+symlink rejection,
read caps, per-call traces, ReAct loop (`_inspect_loop`). (`remember` is the
memory-files subagent; neither targets user documents.)

It **cannot handle PDFs/spreadsheets as-is**, by construction:
- `grep` shells to **ripgrep, which skips binary files** → PDF content invisible.
- `read` does `open(..., 'r', encoding='utf-8')` → PDF returns mojibake.
- git-aware (`git ls-files`, refuses gitignored) → assumes a checkout, not a
  folder of filings.

`pdf-grep` is the **wrong altitude**: single-shot (one pattern → snippets),
whereas real statement analysis is iterative (find "Consolidated Balance
Sheets" → read window → grep "Total liabilities" → cross-check cash flow). The
subagent loop already gives iteration AND context isolation (the 200-page doc
stays in the subagent; only the synthesized answer returns to Jill's window) —
that's the full win Jill asked for; pdf-grep delivers half. It also generalizes
to a *directory* of mixed files (the "small directory of pdf + text +
spreadsheets" Bruce described) and reuses a hardened architecture (project
reuse rule).

**Plan:** build a sibling **document-inspection subagent** reusing the
`_inspect_loop` architecture (geofence, `_safe_resolve`, caps, trace-writing,
parser, primitives). New work:

1. **Extraction layer** under read/grep: `pdfplumber` for PDFs (handles tables
   far better than PyPDF2 — tables are the whole point for financials);
   `openpyxl`/`pandas` for xlsx/csv → text or markdown tables. New deps:
   pdfplumber, openpyxl.
2. **grep-over-binary** — **RESOLVED 2026-06-19: pre-extract, not on-the-fly.**
   The deciding fact (clearer from reading `code_subagent.py` than from the
   plan): `read` needs extraction too, not just grep — `_tool_read` does
   `open(..., 'r', utf-8)` (mojibake on a PDF) and `_tool_grep` shells to `rg`
   (skips binaries). Both primitives consume the SAME text artifact. So the
   chosen shape is: **materialize a parallel text "view" of the bound document
   directory** (pdfplumber→markdown tables, openpyxl/pandas→TSV), then point
   `_inspect_loop` at that view's root. The primitives, `_safe_resolve`
   geofence, caps, trace-writing, and the `rg`/utf-8-open paths then run
   VERBATIM — only new code is the extraction fn + a binding entry point.
   Reasons over on-the-fly: (a) loop is iterative (`_MAX_ITERS`=12, plan's own
   grep→read→grep example) so on-the-fly re-runs pdfplumber per iteration over a
   200-page doc; (b) grep's `path:lineno` and read's line ranges agree for free
   when both read one artifact; (c) maximal architecture reuse.
   **Persistence:** view is **ephemeral** — temp dir, re-extracted per binding,
   NO content-hash cache / invalidation schema (filings immutable, bindings
   rare; persistent cache is a later optimization only if extraction cost bites;
   per `feedback_no_invented_persistence`).
3. **Geofence root for documents** — copy the `external_repo` sticky-binding
   pattern to bind a "documents directory" the subagent is fenced to.

**Decision 3 — build vs. defer: DEFER the full subagent (chosen 2026-06-19).**
Recurrence test not met: demand is a single anticipated case (Jill's
gap-detector firing once in one IB-analysis chat), not an observed recurring
need. Near-term plan:
- Ship the AV `get_financial_statements` tool (the validated concrete win).
- For documents, build ONLY the **narrow pdfplumber→markdown extract helper**
  feeding the existing fetch/read flow — covers "grab one section from a
  web-fetched 10-K." Note this helper IS the extraction layer (item 1 above),
  so it's path-independent: not throwaway if the subagent is later built.
- Build the full geofenced `inspect_documents` subagent only when a SECOND,
  real "small directory of mixed docs" use case appears.
Resolution A (extract-to-text) holds either way — single-file pdfplumber→
markdown is exactly the deferred helper.

## Meta-note on the deficiency mechanism

It proposed `pdf-grep` from scratch without noticing the `code_subagent`
architecture it should have built on — reasoning from Jill's tool *surface*
(`fetch-text`, `calculate`), not the *codebase*, despite `inspect` existing for
exactly that. Consider requiring a capability-gap proposal to run an `inspect`
pass first, so proposals start from "what can I extend?" not "what would I write
from scratch?"

## Resume checklist (tomorrow)

- [x] Confirm AV free-tier rate limits + fundamentals on free tier for this
      account. DONE 2026-06-17: works; 25/day, 5/min, 1/sec burst; handle
      `Information` throttle key + self-throttle (see Decision 1).
- [x] Decide grep-over-binary strategy. DONE 2026-06-19: pre-extract to an
      ephemeral text view, reuse `_inspect_loop` verbatim (see item 2 above).
- [x] Decide whether to build the doc subagent now or defer. DONE 2026-06-19:
      DEFER full subagent; ship narrow pdfplumber→markdown extract helper only
      (Decision 3 above).
- [x] Build `get_financial_statements` AV tool. DONE 2026-06-19:
      `src/tools/get-financial-statements/{tool.py,Skill.md}`. Multi-statement
      (income/balance/cash_flow/earnings/overview, default 3 core), handles the
      `Information` throttle key (soft-fail + 1 retry), self-throttles ≥1.2s
      between AV requests, trims to recent 4 annual + 4 quarterly. Live-verified
      against IBM income (1 quota call). New dep: none (reuses `requests` + key).
- [x] Build narrow PDF→markdown extract helper. DONE 2026-06-19:
      `src/utils/doc_extract.py` — `pdf_to_markdown(path|bytes)` (tables → md,
      all-empty spacer columns dropped) + `extract_to_markdown(path)` suffix
      dispatcher (PDF only in v1, raises on others). This IS the extraction
      layer the deferred doc subagent will reuse.
      **Library: pymupdf, NOT pdfplumber** — reversed the plan's choice. pymupdf
      was already a project dep (fetch-text uses it); validating both on a real
      Berkshire 10-K balance sheet showed pymupdf's `find_tables()` PRESERVES
      word spacing in line-item labels ("Total liabilities") where pdfplumber
      jammed them ("Totalliabilities" — un-greppable), while recovering
      identical numbers. So no new dep + better output. (The plan reasoned
      pdfplumber-vs-PyPDF2 and never saw pymupdf — the same "reason from the
      tool surface not the codebase" miss the plan's own meta-note flags.)
- [ ] (Optional) prototype: gate capability-gap proposals behind an `inspect`
      pass.
