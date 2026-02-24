---
name: stock-price
type: python
description: "Get current stock quote (price, change, volume) from Alpha Vantage. Returns a Note with JSON quote data."
schema_hint: {"symbol": "string (ticker, e.g. AAPL, IBM)"}
---

# stock-price

Fetch current stock quote from Alpha Vantage GLOBAL_QUOTE API. Returns a Note containing JSON with price, change, volume, and related fields.

## Input

- `symbol`: Ticker symbol (e.g., AAPL, IBM, MSFT)

## Output

Success (`status: "success"`):
- `resource_id`: Note ID containing JSON quote data
- Note content: `{"symbol": "AAPL", "price": "151.50", "change": "1.50", "change_percent": "1.00%", "volume": "12345678", ...}`

Failure (`status: "failed"`):
- `reason`: Error (e.g., "ALPHA_VANTAGE_API_KEY environment variable required", "No quote data for XYZ")

## Behavior

- Requires `ALPHA_VANTAGE_API_KEY` environment variable
- Free tier: 25 requests/day
- Data is end-of-day by default; realtime/delayed requires premium

## Common Workflows

**Single quote:**
```json
{"type":"stock-price","symbol":"AAPL","out":"$quote"}
{"type":"extract","target":"$quote","instruction":"Extract price and change percent","out":"$summary"}
{"type":"synthesize","target":"$summary","focus":"Format as a brief human-readable report","out":"$report"}
{"type":"say","target":"User","value":"$report"}
```

**Compare symbols:**
```json
{"type":"stock-price","symbol":"AAPL","out":"$aapl"}
{"type":"stock-price","symbol":"MSFT","out":"$msft"}
{"type":"synthesize","target":["$aapl","$msft"],"focus":"Compare current prices and day change","out":"$comparison"}
```
