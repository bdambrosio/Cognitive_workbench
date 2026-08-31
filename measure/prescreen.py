"""Pre-screen a candidate model's endpoints with the payload we actually send.

WHY THIS IS NOT `_ChatBackend`. The real backend refuses a model with no
configured temperature, and that is deliberate — see src/chat/model_params.py.
But the pre-screen is what decides whether a model is worth asking Bruce for a
temperature at all, so it cannot go through the gate it exists to feed. It
therefore rebuilds the request body. The schema is imported rather than copied;
the rest of the body shape must be re-checked against `_ChatBackend.chat()`
whenever that changes.

TEMPERATURE IS OMITTED BY DEFAULT, so results are at each provider's default
sampling and every row in the table is comparable to every other. This screens
capability and cost, never quality. Nothing here is a benchmark result.

`--temperature` sends an explicit one. It exists for a single question — does
temperature move tokens per call at all — and a row probed with it is NOT
comparable to the rest of the table. It is opt-in for that reason: there is no
default to inherit, which is the same stance src/chat/model_params.py takes.

THE THREE GATES, in the order that eliminates cheapest-first:

  1. Structured output. Does the endpoint accept response_format=json_schema
     with the real schema, and return a schema-valid action? Published
     throughput rankings are measured WITHOUT it, and endpoints that refuse it
     are not candidates at any speed.
  2. Tokens per call for a one-line action. This architecture makes 35-187
     calls per run, so tokens per call sets wall clock. It is what excluded
     Qwen3.8-2.4T at 2,902 tokens for a one-line answer.
  3. One fixture run graded on admissibility only. Not in this file — it is a
     real run. Admissibility is the only review signal that holds still; the
     supported ratio moved 19/23, 21/23, 23/23 on identical text.

TRAPS THIS FILE EXISTS TO AVOID, both paid for already:

  - `max_tokens` must be what a real call uses. At 512 the reasoning models
     here burned the whole budget thinking and returned EMPTY content with
     finish_reason=length, which reads exactly like a model that cannot follow
     the format. `finish` is reported so that is never misread again.
  - The endpoints API is for shortlisting only. It lists `response_format` for
     endpoints that then 404, and lists endpoints that are not servable at all.
     Probe with the real payload.

PIN THE PROVIDER, FALLBACKS OFF. One model id can serve three different
artifacts — different quantization, different snapshot — and a mid-run failover
changes which, with the model id unchanged.

Usage:
    export OPENROUTER_API_KEY=...
    python3 measure/prescreen.py moonshotai/kimi-k2.7-code alibaba/fp8,together

Listing a model's endpoints first:
    curl -s -H "Authorization: Bearer $OPENROUTER_API_KEY" \
      https://openrouter.ai/api/v1/models/<model>/endpoints
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import requests

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from chat.react import REACT_ACTION_SCHEMA                    # noqa: E402

URL = "https://openrouter.ai/api/v1/chat/completions"

# A one-line action against a realistic auditor prompt. Deliberately small:
# gate 1 and gate 2 do not need real context, and a 58k-token probe costs
# minutes per endpoint. The seconds reported here are therefore NOT comparable
# to in-run seconds-per-call, which carry the full context. Tokens per call and
# tokens/sec are.
SYSTEM = (
    "You are an auditor working a technical due-diligence engagement. Each "
    "turn you emit exactly ONE action as a JSON object with a `thought` and a "
    "`tool` field, plus whatever arguments that tool takes. Available tools: "
    "inspect_external(query), read_file(path), respond(text), yield(next).\n\n"
    "Emit the action and nothing else. No prose outside the JSON.")

USER = (
    "The data room has nine documents. You have read doc4_infrastructure_"
    "config.md and found that backups have been failing for 21 days with no "
    "alerting configured. The seller's claim sheet says backups run nightly "
    "'with 30-day retention'.\n\nEmit next action:")


def probe(model: str, tag: str, effort: str | None = "low",
          max_tokens: int = 16384, timeout: int = 300,
          url: str = URL, key_env: str | None = "OPENROUTER_API_KEY",
          temperature: float | None = None,
          system: str = SYSTEM, user: str = USER,
          schema: dict | None = REACT_ACTION_SCHEMA) -> dict:
    """One endpoint, one call. Returns a row; never raises.

    `tag` pins the OpenRouter provider. For a direct route (a provider's own
    API rather than the gateway) pass `url` and `key_env` and set tag to the
    provider name — there is nothing to pin, so the `provider` block is
    dropped. `key_env=None` is a local server that wants no bearer token.

    TWO CALLERS SINCE 2026-08-31. The gates above send a react action under
    `REACT_ACTION_SCHEMA`; `measure/method_probe` sends a question about
    METHOD and wants prose, so it passes `schema=None`. The defaults are the
    gate probe unchanged, and every row in docs/model-prescreen.md was taken
    with them — a caller that overrides `system`, `user` or `schema` is NOT
    producing a row comparable to that table.
    """
    direct = "openrouter.ai" not in url
    body = {
        "model": model,
        "messages": [{"role": "system", "content": system},
                     {"role": "user", "content": user}],
        "max_tokens": max_tokens,
        "top_p": 0.95,
    }
    if schema is not None:
        body["response_format"] = {"type": "json_schema",
                                   "json_schema": {"name": "react_action",
                                                   "schema": schema}}
    if not direct:
        body["provider"] = {"order": [tag], "allow_fallbacks": False}
    if effort:
        body["reasoning_effort"] = effort
    # Sent only when asked for. Absent means the provider default, which is
    # what the comparable rows in docs/model-prescreen.md were probed at.
    if temperature is not None:
        body["temperature"] = temperature
    key = os.environ.get(key_env) if key_env else None
    if key_env and not key:
        return {"model": model, "tag": tag, "status": "NOKEY",
                "err": f"{key_env} not set. ~/.bashrc returns early for "
                       f"non-interactive shells; eval the export line."}
    t0 = time.time()
    try:
        headers = {"Content-Type": "application/json"}
        if key:
            headers["Authorization"] = f"Bearer {key}"
        r = requests.post(url, json=body, timeout=timeout, headers=headers)
    except Exception as e:                                     # noqa: BLE001
        return {"model": model, "tag": tag, "status": "EXC",
                "err": f"{type(e).__name__}: {e}"}
    dt = time.time() - t0
    if not r.ok:
        # 404 here means "listed but not servable", not "no such model".
        return {"model": model, "tag": tag, "status": r.status_code,
                "err": r.text[:200].replace("\n", " ")}
    d = r.json()
    if "choices" not in d:
        return {"model": model, "tag": tag, "status": "NOCHOICES",
                "err": str(d)[:200]}
    choice = d["choices"][0]
    msg = choice.get("message") or {}
    content = (msg.get("content") or "").strip()
    reasoning = msg.get("reasoning") or msg.get("reasoning_content") or ""
    usage = d.get("usage") or {}
    ctok = usage.get("completion_tokens")
    if schema is None:
        valid, tool = None, None
    else:
        try:
            parsed = json.loads(content)
            valid = (isinstance(parsed, dict)
                     and "thought" in parsed and "tool" in parsed)
            tool = parsed.get("tool") if isinstance(parsed, dict) else None
        except Exception as e:                                 # noqa: BLE001
            valid, tool = False, None
            if content:
                print(f"  ({tag}: content did not parse as JSON: {e})",
                      file=sys.stderr)
    return {"model": model, "tag": tag, "status": "ok",
            "temperature": temperature,
            "finish": choice.get("finish_reason"),
            "secs": round(dt, 1),
            "prompt_tok": usage.get("prompt_tokens"),
            "completion_tok": ctok,
            "reasoning_chars": len(reasoning),
            # Needed to settle whether `completion_tokens` bills the reasoning
            # channel or only the visible answer: with both lengths and the
            # token count, the chars-per-token arithmetic decides it.
            # backend.py's clamp skips this arithmetic on cloud routes on the
            # assumption that it does NOT — an assumption never measured.
            "content_chars": len(content),
            "tok_s": round(ctok / dt, 1) if ctok and dt else None,
            "schema_valid": valid, "tool": tool,
            "provider": d.get("provider"),
            "content": content if schema is None else None,
            "content_head": content[:80].replace("\n", " ")}


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    model, tags = sys.argv[1], sys.argv[2].split(",")
    effort = sys.argv[3] if len(sys.argv) > 3 else "low"
    # Optional direct route: prescreen.py <model> <name> <effort> <url> <keyenv>
    url = sys.argv[4] if len(sys.argv) > 4 else URL
    key_env = sys.argv[5] if len(sys.argv) > 5 else "OPENROUTER_API_KEY"
    for tag in tags:
        print(json.dumps(probe(model, tag, effort=effort or None,
                               url=url, key_env=key_env)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
