---
name: get_financial_statements
description: Fetch standardized financial statements (income, balance sheet, cash flow, earnings, company overview) for a ticker from Alpha Vantage. Returns combined annual+quarterly JSON for analysis.
args:
  ticker: required string — ticker symbol, e.g. "AAPL", "IBM", "MSFT"
  statements: optional list — any of "income", "balance", "cash_flow", "earnings", "overview". Defaults to ["income", "balance", "cash_flow"].
---

# get_financial_statements

Pull standardized financial statements from Alpha Vantage's fundamentals
endpoints — no PDF scraping, no data cleaning. Returns one JSON note keyed by
statement type, each trimmed to the most recent ~4 annual + 4 quarterly periods.

Statement keys → Alpha Vantage endpoint:

- `income` → INCOME_STATEMENT
- `balance` → BALANCE_SHEET
- `cash_flow` → CASH_FLOW
- `earnings` → EARNINGS (annual + quarterly EPS, surprises)
- `overview` → OVERVIEW (description, sector, market cap, key ratios — flat)

## Required environment

- `ALPHA_VANTAGE_API_KEY` — the same key `stock-price` uses.

## Examples

```json
{"thought": "get the 3 core statements for IBM", "tool": "get_financial_statements", "ticker": "IBM"}
```

```json
{"thought": "I only need the balance sheet and overview for Apple", "tool": "get_financial_statements", "ticker": "AAPL", "statements": ["balance", "overview"]}
```

## Notes & rate limits

- **Free tier is tight: 25 requests/day, ~5/min, ~1 request/sec burst.** Each
  statement type is one request — the default 3-statement call costs 3. A full
  peer-set comparison (3–5 companies) can exhaust the daily budget, so sequence
  calls deliberately; don't fan out across many tickers at once.
- The tool self-throttles (spaces successive requests ≥1s apart) and retries
  once on a throttle response, but it cannot raise the daily ceiling.
- AV fundamentals are less granular than dedicated providers — adequate for the
  core statements and trend analysis, not line-item-exhaustive.
