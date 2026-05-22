---
name: stock-price
description: Look up the current stock price (price, change, volume) for a ticker symbol. End-of-day data on the free tier.
args:
  ticker: required string — ticker symbol, e.g. "AAPL", "MSFT", "TSLA"
---

# stock-price

Fetch a current quote from Alpha Vantage's GLOBAL_QUOTE endpoint. Returns text with price, change, change percent, volume, and related fields.

## Required environment

- `ALPHA_VANTAGE_API_KEY` — free tier allows 25 requests per day.

## Examples

```json
{"thought": "check Apple's price", "tool": "stock-price", "ticker": "AAPL"}
```

```json
{"thought": "compare prices of AAPL and MSFT", "tool": "stock-price", "ticker": "AAPL"}
```
(Second ticker requires a second tool call — one quote per invocation.)

## Notes

- Data is end-of-day by default. Real-time / delayed quotes require an Alpha Vantage premium key.
- Free-tier rate limits are restrictive (25/day) — don't poll.
