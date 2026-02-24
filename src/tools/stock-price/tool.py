"""
Stock price quote via Alpha Vantage GLOBAL_QUOTE API.
Returns a Note with JSON quote data (price, change, volume, etc.).

Env: ALPHA_VANTAGE_API_KEY
"""

import json
import logging
import os
from typing import Any, Dict, Optional

import requests

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

BASE_URL = "https://www.alphavantage.co/query"


def _fail(executor: InfospaceExecutor, reason: str, value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("failed", value=value or reason, reason=reason, extra=extra)


def _success(executor: InfospaceExecutor, value: str, resource_id: Optional[str], extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return("success", value=value, resource_id=resource_id, extra=extra)


def _create_note(text_content: str, agent_name: str, resource_manager, source_skill: str = "stock-price",
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


def _fetch_quote(symbol: str, api_key: str) -> Optional[Dict[str, Any]]:
    """Fetch GLOBAL_QUOTE from Alpha Vantage. Returns normalized quote dict or None."""
    params = {"function": "GLOBAL_QUOTE", "symbol": symbol.upper(), "apikey": api_key}
    try:
        r = requests.get(BASE_URL, params=params, timeout=15)
        r.raise_for_status()
        data = r.json()
    except requests.RequestException as e:
        logger.error(f"Alpha Vantage request failed: {e}")
        return None
    except json.JSONDecodeError as e:
        logger.error(f"Alpha Vantage response not JSON: {e}")
        return None

    err = data.get("Error Message") or data.get("Note")
    if err:
        logger.warning(f"Alpha Vantage API: {err}")
        return None

    raw = data.get("Global Quote")
    if not raw or not isinstance(raw, dict):
        return None

    # Normalize keys (Alpha Vantage uses "01. symbol", "05. price", etc.)
    quote = {}
    for k, v in raw.items():
        if isinstance(k, str) and ". " in k:
            clean_key = k.split(". ", 1)[1].replace(" ", "_").lower()
        else:
            clean_key = str(k).replace(" ", "_").lower()
        quote[clean_key] = v

    return quote


def tool(input_value=None, runtime=None, **kwargs):
    """
    Get current stock quote from Alpha Vantage.

    Args:
        symbol: Ticker symbol (e.g., AAPL, IBM, MSFT)
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

    quote = _fetch_quote(symbol, api_key)
    if not quote:
        return _fail(executor, f"No quote data for {symbol}", extra={"symbol": symbol})

    agent_name = kwargs.get("agent_name", "")
    resource_manager = kwargs.get("resource_manager")
    if not resource_manager:
        return _fail(executor, "resource_manager required in kwargs")

    content = json.dumps(quote, indent=2)
    tool_meta = {"symbol": symbol, "price": quote.get("price"), "change_percent": quote.get("change_percent")}
    note_id = _create_note(content, agent_name, resource_manager, tool_metadata=tool_meta)
    if not note_id:
        return _fail(executor, "Failed to create Note")

    display = f"{symbol}: ${quote.get('price', 'N/A')} ({quote.get('change_percent', 'N/A')})"
    logger.info(f"stock-price: {display} → {note_id}")
    return _success(executor, display, note_id, extra=tool_meta)
