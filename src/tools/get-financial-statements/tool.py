"""
Standardized financial statements via Alpha Vantage fundamentals APIs.
Returns a Note with combined JSON for the requested statement types
(income / balance / cash_flow / earnings / overview).

Mirrors stock-price/tool.py's react_invoke → tool → _fetch → _create_note
shape, on the SAME Alpha Vantage account/key. Differences from stock-price:
  - fetches one or more fundamentals endpoints per invocation,
  - detects the free-tier throttle (an `Information` key, which
    stock-price's _fetch_quote does NOT check) as a soft failure and
    retries once after a short backoff,
  - self-throttles by spacing successive AV requests ≥1s apart.

Free-tier reality (confirmed 2026-06-17): 25 requests/day, ~5/min, and a
~1 req/sec burst cap. One company at the 3 core statements = 3 requests;
sequence calls deliberately rather than fanning out.

Env: ALPHA_VANTAGE_API_KEY (same key stock-price uses).
"""

import json
import logging
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import requests

# infospace_executor was the planner-side runtime; the chat ReAct loop
# bypasses it. Keep the import optional so this module loads in chat-only
# builds — the InfospaceExecutor annotations degrade to Any there.
try:
    from infospace_executor import InfospaceExecutor
except ImportError:
    InfospaceExecutor = Any  # type: ignore[assignment,misc]

logger = logging.getLogger(__name__)

BASE_URL = "https://www.alphavantage.co/query"

# Closed parameter vocabulary → Alpha Vantage `function` names. Fixed enum,
# not free-form classification.
_STATEMENT_FUNCTIONS = {
    "income": "INCOME_STATEMENT",
    "balance": "BALANCE_SHEET",
    "cash_flow": "CASH_FLOW",
    "earnings": "EARNINGS",
    "overview": "OVERVIEW",
}
_DEFAULT_STATEMENTS = ["income", "balance", "cash_flow"]

# AV fundamentals return most-recent-first arrays (annualReports etc.).
# Keep a bounded window so one note doesn't flood the ReAct context.
_MAX_PERIODS = 4
# Spacing between successive AV requests, and backoff before a throttle
# retry. ≥1s to stay under the ~1 req/sec burst cap.
_THROTTLE_PAUSE = 1.2


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, resource_id: Optional[str], extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)


def _create_note(text_content: str, agent_name: str, resource_manager, source_skill: str = "get-financial-statements",
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=text_content,
        format_type="text",
        source_skill=source_skill,
        source_value=(text_content or "")[:100],
        note_name="",
        extra_props={"tool_metadata": tool_metadata or {}},
    )
    if success:
        return note_id
    logger.error(f"Failed to create Note: {error_msg}")
    return None


def _trim_periods(data: Dict[str, Any], max_periods: int) -> Dict[str, Any]:
    """Bound any list-valued field (annualReports, quarterlyReports,
    annualEarnings, …) to its most-recent `max_periods` entries. AV returns
    these newest-first, so a head slice keeps the recent window. Non-list
    fields (e.g. OVERVIEW's flat scalars) pass through unchanged."""
    out: Dict[str, Any] = {}
    for k, v in data.items():
        out[k] = v[:max_periods] if isinstance(v, list) else v
    return out


def _fetch_statement(function: str, symbol: str, api_key: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """Fetch one fundamentals endpoint. Returns (data, error). On the
    free-tier throttle (an `Information` key), returns (None, "throttled")
    so the caller can back off and retry once."""
    params = {"function": function, "symbol": symbol, "apikey": api_key}
    try:
        r = requests.get(BASE_URL, params=params, timeout=20)
        r.raise_for_status()
        data = r.json()
    except requests.RequestException as e:
        logger.warning(f"Alpha Vantage request failed ({function} {symbol}): {e}")
        return None, str(e)
    except json.JSONDecodeError as e:
        logger.warning(f"Alpha Vantage response not JSON ({function} {symbol}): {e}")
        return None, "response not JSON"

    # Throttle arrives as {"Information": "..."}. stock-price's _fetch_quote
    # misses this key; we must treat it as a soft (retryable) failure.
    if isinstance(data, dict) and data.get("Information"):
        logger.info(f"Alpha Vantage throttle ({function} {symbol}): {data.get('Information')}")
        return None, "throttled"

    err = data.get("Error Message") or data.get("Note") if isinstance(data, dict) else None
    if err:
        logger.warning(f"Alpha Vantage API error ({function} {symbol}): {err}")
        return None, str(err)

    # Invalid symbol / no coverage returns an empty dict on these endpoints.
    if not isinstance(data, dict) or not data:
        return None, "no data"

    return data, None


def _fetch_with_retry(function: str, symbol: str, api_key: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """One throttle retry after a short backoff; other errors pass through."""
    data, err = _fetch_statement(function, symbol, api_key)
    if err == "throttled":
        logger.info(f"backing off {_THROTTLE_PAUSE}s then retrying {function} {symbol}")
        time.sleep(_THROTTLE_PAUSE)
        data, err = _fetch_statement(function, symbol, api_key)
    return data, err


def _normalize_statements(statements: Any) -> Tuple[List[str], Optional[str]]:
    """Resolve the `statements` arg to a validated list of keys. Accepts a
    list, a single string, or None (→ defaults). Returns (keys, error)."""
    if statements is None or statements == "":
        return list(_DEFAULT_STATEMENTS), None
    if isinstance(statements, str):
        requested = [statements]
    elif isinstance(statements, list):
        requested = [str(s) for s in statements]
    else:
        return [], f"`statements` must be a string or list, got {type(statements).__name__}"
    keys: List[str] = []
    for s in requested:
        key = s.strip().lower()
        if key not in _STATEMENT_FUNCTIONS:
            return [], (f"unknown statement {s!r}; valid: "
                        f"{', '.join(_STATEMENT_FUNCTIONS)}")
        if key not in keys:
            keys.append(key)
    return keys, None


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    from utils.chat_tool_stub import build_tool_kwargs, CapturingResourceManager, translate_result
    ticker = args.get("ticker", "")
    if not isinstance(ticker, str) or not ticker.strip():
        return {"status": "error", "text": "get_financial_statements requires non-empty `ticker`"}

    mgr = CapturingResourceManager()
    result = tool(ticker.upper(), **build_tool_kwargs(
        character_name=character_name, backend=backend, manager=mgr,
        symbol=ticker.upper(), statements=args.get("statements"),
    ))
    return translate_result(result, manager=mgr,
                            empty_text=f"no financial data for {ticker}")


def tool(input_value=None, runtime=None, **kwargs):
    """
    Fetch standardized financial statements from Alpha Vantage.

    Args:
        symbol: Ticker symbol (e.g., AAPL, IBM, MSFT)
        statements: list of statement keys (income, balance, cash_flow,
            earnings, overview); defaults to the 3 core statements.
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available", "value": None, "resource_id": None}

    api_key = os.environ.get("ALPHA_VANTAGE_API_KEY", "").strip()
    if not api_key:
        return _fail(executor, "ALPHA_VANTAGE_API_KEY environment variable required")

    symbol = kwargs.get("symbol") or input_value
    if not symbol or not isinstance(symbol, str):
        return _fail(executor, "symbol parameter required (e.g., AAPL, IBM)")
    symbol = symbol.strip().upper()
    if not symbol:
        return _fail(executor, "symbol cannot be empty")

    keys, kerr = _normalize_statements(kwargs.get("statements"))
    if kerr:
        return _fail(executor, kerr, extra={"symbol": symbol})

    collected: Dict[str, Any] = {}
    failures: Dict[str, str] = {}
    for idx, key in enumerate(keys):
        if idx > 0:
            time.sleep(_THROTTLE_PAUSE)  # self-throttle between AV requests
        function = _STATEMENT_FUNCTIONS[key]
        data, err = _fetch_with_retry(function, symbol, api_key)
        if err:
            failures[key] = err
            continue
        collected[key] = _trim_periods(data, _MAX_PERIODS)

    if not collected:
        detail = "; ".join(f"{k}: {v}" for k, v in failures.items()) or "no data"
        return _fail(executor, f"No financial data for {symbol} ({detail})",
                     extra={"symbol": symbol, "failures": failures})

    agent_name = kwargs.get("agent_name", "")
    resource_manager = kwargs.get("resource_manager")
    if not resource_manager:
        return _fail(executor, "resource_manager required in kwargs")

    payload = {"symbol": symbol, "statements": collected}
    if failures:
        payload["unavailable"] = failures
    content = json.dumps(payload, indent=2)
    tool_meta = {"symbol": symbol, "fetched": list(collected.keys()),
                 "unavailable": list(failures.keys())}
    note_id = _create_note(content, agent_name, resource_manager, tool_metadata=tool_meta)
    if not note_id:
        return _fail(executor, "Failed to create Note")

    fetched_str = ", ".join(collected.keys())
    display = f"{symbol}: fetched {fetched_str}"
    if failures:
        display += f" (unavailable: {', '.join(failures.keys())})"
    logger.info(f"get-financial-statements: {display} → {note_id}")
    return _success(executor, content, note_id, extra=tool_meta)
