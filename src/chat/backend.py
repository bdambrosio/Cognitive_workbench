"""LLM backend client for chat mode — moved verbatim from chat_loop.py
in the 2026-06 mixin refactor. Routing: anthropic native / unified
OpenAI-compat with api_key / legacy cloud / local engines."""

from __future__ import annotations

import json
import logging
import os
import sys
from typing import Any, Dict, List, Optional

import requests

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from cot_profiles import is_reasoning_model, resolve_profile  # noqa: E402

logger = logging.getLogger('chat_loop')

# ─── LLM backend ────────────────────────────────────────────────────────────

class _ChatBackend:
    """Thin OpenAI-compatible chat client with structured-CoT support.

    Routes (checked in order):
      1. server == 'anthropic' → POST {base_url}/v1/messages with
         `x-api-key` + `anthropic-version` headers. system is a top-level
         string field; messages contains only user/assistant. temperature
         and top_p are omitted (Opus 4.7 rejects them; defaults are fine
         for other Claude models). stops → stop_sequences. Requires api_key.
      2. `api_key` set → unified OpenAI-compat path. POST to
         {base_url}/v1/chat/completions with `Authorization: Bearer
         <env[api_key]>`. Grammar / chat_template_kwargs are NOT attached
         (cloud endpoints reject them). The api_key field is the NAME of
         an environment variable, not the key itself.
      3. server in ('openrouter', 'openai') → utils.llm_api.LLM (legacy
         cloud shortcut, kept for back-compat).
      4. anything else (local, vllm, llama.cpp, sglang_api_server, lmstudio,
         unset) → POST {base_url}/v1/chat/completions directly, no auth.
         SGLangAPIServer accepts both `grammar` (GBNF, forwarded as ebnf)
         and `ebnf`; llama-server accepts `grammar`.

    `cot_profile` selects a GBNF skeleton for the <think> block. Attached
    only when the model gates as reasoning (auto-detected by name or
    forced via `is_reasoning=True`) AND we're on path 3 (no api_key).
    """

    def __init__(self, server: str, model: str, base_url: str,
                 is_reasoning: Optional[bool] = None,
                 api_key: Optional[str] = None,
                 reasoning_effort: Optional[str] = None):
        self.server = (server or 'local').lower()
        self.model = model or ''
        self.base_url = (base_url or 'http://127.0.0.1:5000').rstrip('/')
        if self.base_url.endswith('/v1'):
            self.base_url = self.base_url[:-3]
        # OpenAI-style reasoning effort dial. gpt-oss harmony serving
        # accepts "low"|"medium"|"high"; vLLM with the gpt-oss reasoning
        # parser respects the field. Forwarded only on the local OAI path
        # below — Anthropic and the legacy cloud_llm route ignore it.
        eff = (reasoning_effort or '').strip().lower() or None
        if eff is not None and eff not in ('low', 'medium', 'high'):
            raise RuntimeError(
                f"_ChatBackend: reasoning_effort must be low|medium|high, "
                f"got {reasoning_effort!r}")
        self.reasoning_effort = eff
        # Learned per-endpoint parameter adaptations (see _post_adapting).
        self._param_renames: Dict[str, str] = {}
        self._param_drops: set = set()
        # New unified path: api_key field is the NAME of an env var. We
        # resolve it once at init so a missing env var fails loudly here
        # (with the var name) instead of silently 401-ing on first call.
        self._api_key_value: Optional[str] = None
        self._api_key_var: Optional[str] = None
        if api_key:
            api_key = str(api_key).strip()
            if api_key:
                self._api_key_var = api_key
                resolved = os.environ.get(api_key)
                if not resolved:
                    raise RuntimeError(
                        f"_ChatBackend: api_key env var {api_key!r} is not set. "
                        f"Either export {api_key} or remove the api_key field.")
                self._api_key_value = resolved
        # Anthropic native route requires api_key. Validate up front so
        # the failure is loud at scenario load instead of on first turn.
        if self.server == 'anthropic' and self._api_key_value is None:
            raise RuntimeError(
                "_ChatBackend: server='anthropic' requires api_key "
                "(name of env var holding the Anthropic API key).")

        # Legacy cloud shortcut — only used when api_key is NOT set, so
        # the new unified path takes precedence for any config that
        # specifies api_key (even if server happens to be openrouter/openai).
        self._cloud_llm = None
        if self._api_key_value is None and self.server in ('openrouter', 'openai'):
            from utils.llm_api import LLM
            self._cloud_llm = LLM(server_name=self.server, model_name=self.model)
        # Cloud routing flag, exposed for callers that need to vary
        # behavior by backend kind (e.g. ChatLoop's per-profile token
        # floors, which must be looser on local engines that lump
        # reasoning + visible output into one budget).
        self.is_cloud = (self._api_key_value is not None
                         or self._cloud_llm is not None
                         or self.server == 'anthropic')
        if is_reasoning is None:
            self.is_reasoning = is_reasoning_model(self.model)
        else:
            self.is_reasoning = bool(is_reasoning)
        # Stop/finish reason of the most recent chat() call. Surfaced so the
        # ReAct loop can tell a token-limit truncation (finish='length' /
        # stop_reason='max_tokens') apart from genuinely malformed output
        # when an emission fails to parse. Set on every chat() return path.
        self.last_finish_reason: Optional[str] = None
        # Server context window, fetched lazily from /v1/models on the
        # first local call. Sentinel None = not yet asked; 0 = asked and
        # the server did not say, so stop asking.
        self._server_max_model_len: Optional[int] = None

    @property
    def supports_image_input(self) -> bool:
        """True when the active route can carry image_url content parts.
        Route 4 (unified OpenAI-compat: vLLM, llama-server, xAI, OpenAI
        with api_key) handles them natively. Anthropic uses a different
        content-block shape; the legacy cloud_llm path may not handle
        list content. Both rejected for v1."""
        if self.server == 'anthropic':
            return False
        if self._cloud_llm is not None:
            return False
        return True

    # Providers differ on which chat-completions parameters they accept, and
    # the differences move: OpenAI's reasoning models reject `max_tokens` in
    # favour of `max_completion_tokens`, xAI's grok-4.3 rejects `stop`. Rather
    # than carry a table of model names — guesswork that rots — read the
    # provider's own 400. It is structured for exactly this:
    #   {"error": {"code": "unsupported_parameter", "param": "max_tokens",
    #              "message": "... Use 'max_completion_tokens' instead."}}
    # so the offending field is named in a machine-readable slot. Rename it if
    # we know an equivalent, otherwise drop it, retry, and remember for the
    # rest of the session — one wasted request per endpoint, once.
    _PARAM_EQUIVALENTS = {'max_tokens': 'max_completion_tokens'}

    # Bound the loop: each pass must name a parameter still in the body, and
    # each pass removes or renames one, so it terminates well inside the cap.
    _PARAM_ADAPT_MAX = 4

    # Rough chars-per-token for the prompt estimate. Deliberately low so
    # the estimate over-counts tokens and the clamp errs toward reserving
    # less output — an answer that is shorter than asked beats a 400.
    _CHARS_PER_TOKEN = 3.0
    # Slack for the chat template's own scaffolding (role headers, the
    # generation prompt, a thinking-channel opener) which the character
    # count above does not see.
    _CLAMP_MARGIN_TOKENS = 512

    def _server_window(self) -> int:
        """Context window of the served model, or 0 if unknown.

        Asked once, from /v1/models. The window belongs to whichever model
        the server currently holds — not to any scenario — which is why
        this is discovered here rather than declared in YAML. A scenario
        cannot know it, and on a rig where the backend is swapped several
        times a day, a hand-copied value is wrong the moment it changes.
        """
        if self._server_max_model_len is not None:
            return self._server_max_model_len
        self._server_max_model_len = 0
        try:
            resp = requests.get(f'{self.base_url}/v1/models', timeout=10)
            if resp.ok:
                for m in (resp.json() or {}).get('data') or []:
                    win = m.get('max_model_len')
                    if isinstance(win, int) and win > 0:
                        self._server_max_model_len = win
                        logger.info(
                            "_ChatBackend: server context window is %d tokens",
                            win)
                        break
        except Exception as e:
            logger.warning(f"_ChatBackend: could not read /v1/models: {e}")
        return self._server_max_model_len

    def _clamp_max_tokens(self, messages: List[Dict[str, Any]],
                          max_tokens: int) -> int:
        """Shrink the output reservation so prompt + output fits the window.

        vLLM rejects at admission when prompt_tokens + max_tokens exceeds
        max_model_len — no tokens generated, whole turn lost to fallback
        synthesis. Observed 2026-08-15 at 57,145 + 8,192 against 65,336:
        over by one token.

        Clamping beats retrying: nothing is spent discovering the limit.
        It cannot rescue a prompt that alone exceeds the window — that
        needs history trimming, which is _cap_turn's job — but it removes
        the failure mode where a reservation the caller chose blindly is
        what tips it over.
        """
        window = self._server_window()
        if not window or self.is_cloud:
            # Cloud providers meter reasoning separately and max_tokens
            # caps visible output only, so this arithmetic does not apply.
            return max_tokens
        chars = sum(len(str(m.get('content', '') or '')) for m in messages)
        est_prompt = int(chars / self._CHARS_PER_TOKEN) + self._CLAMP_MARGIN_TOKENS
        room = window - est_prompt
        if room >= max_tokens:
            return max_tokens
        if room < 256:
            # Nothing useful left to reserve. Send the floor and let the
            # server decide — a 400 here is honest, and the caller's
            # fallback path handles it. Silently emitting a 20-token reply
            # would look like the model had nothing to say.
            logger.warning(
                "_ChatBackend: estimated prompt %d tokens leaves %d of a "
                "%d-token window; output reservation cannot be met",
                est_prompt, room, window)
            return max(room, 256)
        logger.info(
            "_ChatBackend: clamping max_tokens %d -> %d (est prompt %d, "
            "window %d)", max_tokens, room, est_prompt, window)
        return room

    def _post_adapting(self, url: str, headers: Dict[str, str],
                       body: Dict[str, Any]):
        resp = requests.post(url, headers=headers, json=body, timeout=120)
        for _ in range(self._PARAM_ADAPT_MAX):
            if resp.ok or resp.status_code != 400:
                return resp
            try:
                err = (resp.json() or {}).get('error') or {}
            except ValueError:
                return resp
            # unsupported_parameter: the field itself is rejected — rename it
            # if we know an equivalent, else drop it.
            # unsupported_value: the field is fine but our value is not (e.g.
            # reasoning models pin temperature to 1). Dropping it lets the
            # provider's default apply, which is what those models want.
            code = err.get('code')
            if code not in ('unsupported_parameter', 'unsupported_value'):
                return resp
            param = err.get('param')
            if not param or param not in body:
                return resp
            new = (self._PARAM_EQUIVALENTS.get(param)
                   if code == 'unsupported_parameter' else None)
            if new:
                body[new] = body.pop(param)
                self._param_renames[param] = new
                logger.info(
                    "_ChatBackend: %s rejects %r; using %r for this session",
                    self.model or self.base_url, param, new)
            else:
                body.pop(param, None)
                self._param_drops.add(param)
                logger.info(
                    "_ChatBackend: %s rejects %r; dropping it for this session",
                    self.model or self.base_url, param)
            resp = requests.post(url, headers=headers, json=body, timeout=120)
        return resp

    def chat(self, messages: List[Dict[str, Any]],
             max_tokens: int = 600,
             temperature: float = 0.7,
             top_p: float = 1.0,
             stops: Optional[List[str]] = None,
             is_json: bool = False,
             cot_profile: Optional[str] = None,
             enable_thinking: Optional[bool] = None,
             reasoning_effort: Optional[str] = None,
             response_schema: Optional[Dict[str, Any]] = None) -> str:
        # Per-call reasoning_effort override. Only takes effect when the
        # scenario already declared a baseline (self.reasoning_effort is
        # not None) — the field is a reasoning-model concept; we don't
        # send it to non-reasoning backends. Validation matches __init__.
        if reasoning_effort is not None:
            eff = str(reasoning_effort).strip().lower()
            if eff not in ('low', 'medium', 'high'):
                raise RuntimeError(
                    f"_ChatBackend.chat: reasoning_effort must be "
                    f"low|medium|high, got {reasoning_effort!r}")
            reasoning_effort = eff
        stops = stops or []
        if self._cloud_llm is not None:
            from Messages import SystemMessage, UserMessage, AssistantMessage
            cls_by_role = {'system': SystemMessage, 'user': UserMessage, 'assistant': AssistantMessage}
            prompt_msgs = [cls_by_role.get(m['role'], UserMessage)(content=m['content']) for m in messages]
            response = self._cloud_llm.ask(
                input={}, prompt_msgs=prompt_msgs,
                temp=temperature, top_p=top_p, max_tokens=max_tokens,
                stops=stops if stops else None, is_json=is_json, log=False, trace=False,
            )
            if response is None:
                raise RuntimeError(f'cloud LLM ({self.server}) returned None')
            self.last_finish_reason = None  # legacy cloud route doesn't expose it
            return response if isinstance(response, str) else json.dumps(response)

        # Anthropic native Messages API. Different endpoint, headers, and
        # body shape than OpenAI chat completions. temperature/top_p are
        # omitted: Opus 4.7 rejects temperature outright, and other Claude
        # models work fine on defaults. cot_profile / enable_thinking /
        # is_json are local-engine concepts and don't apply here.
        if self.server == 'anthropic':
            sys_parts: List[str] = []
            convo: List[Dict[str, str]] = []
            for m in messages:
                role = m.get('role')
                content = m.get('content', '')
                if role == 'system':
                    if content:
                        sys_parts.append(content)
                else:
                    convo.append({'role': role, 'content': content})
            body = {
                'model': self.model,
                'max_tokens': max_tokens,
                'messages': convo,
            }
            if sys_parts:
                # Mark the system block as cacheable. Cuts per-call input
                # cost ~90% on cache hits (5-min ephemeral TTL); pays a 25%
                # write premium on the first call. Wins are largest for
                # the judge (12 sequential calls share an identical rubric)
                # and the ReAct inner loop (same system across 2-3 iters
                # within a turn). Below ~1024 tokens the marker is ignored
                # and we just pay base price, so no downside on short calls.
                body['system'] = [{
                    'type': 'text',
                    'text': "\n\n".join(sys_parts),
                    'cache_control': {'type': 'ephemeral'},
                }]
            if stops:
                body['stop_sequences'] = stops
            headers = {
                'Content-Type': 'application/json',
                'x-api-key': self._api_key_value,
                'anthropic-version': '2023-06-01',
            }
            resp = requests.post(
                f'{self.base_url}/v1/messages',
                headers=headers, json=body, timeout=120,
            )
            resp.raise_for_status()
            data = resp.json()
            self.last_finish_reason = data.get('stop_reason')
            blocks = data.get('content') or []
            text = ''.join(
                b.get('text', '') for b in blocks
                if isinstance(b, dict) and b.get('type') == 'text'
            )
            try:
                usage = data.get('usage') or {}
                logger.info(
                    "<llm-raw> route=anthropic stop=%s "
                    "in=%s cache_write=%s cache_read=%s out=%s text=%s",
                    data.get('stop_reason'),
                    usage.get('input_tokens'),
                    usage.get('cache_creation_input_tokens'),
                    usage.get('cache_read_input_tokens'),
                    usage.get('output_tokens'),
                    json.dumps(text, ensure_ascii=False),
                )
            except Exception:
                pass
            return text

        url = f'{self.base_url}/v1/chat/completions'
        max_tokens = self._clamp_max_tokens(messages, max_tokens)
        body: Dict[str, Any] = {
            'messages': messages,
            'temperature': temperature,
            'top_p': top_p,
            'max_tokens': max_tokens,
        }
        if self.model:
            body['model'] = self.model
        # Wire-level `stop` field. xAI grok-4.3 hard-rejects this
        # parameter (400 "Model grok-4.3 does not support parameter
        # stop"); the rest of the OpenAI-compat ecosystem accepts it.
        # Skip on cloud paths and rely on (a) prompt-level discipline —
        # the reflection templates explicitly require an "</end>" marker
        # and forbid postscripts — plus (b) the client-side stop-marker
        # truncation just before return below, which replicates the
        # server-side cutoff for any model whose compliance slips.
        # Local engines (llama-server, SGLang) keep the safety belt.
        if stops and not self.is_cloud:
            body['stop'] = stops
        # Reasoning effort: scenario-declared baseline, optionally
        # overridden per call. Sent whenever either is set, so a call site
        # can opt into reasoning without the scenario declaring a baseline
        # — vLLM maps this into chat_template_kwargs and derives
        # enable_thinking from it, which lets a server launched with
        # enable_thinking=false stay off everywhere except these calls.
        effective = (reasoning_effort if reasoning_effort is not None
                     else self.reasoning_effort)
        if effective is not None:
            body['reasoning_effort'] = effective

        # Skip grammar / chat_template_kwargs when going to a cloud endpoint
        # (signaled by api_key being set). Cloud providers reject those
        # fields; locally-served engines (llama-server, SGLangAPIServer)
        # accept them.
        if not self.is_cloud:
            grammar = resolve_profile(cot_profile) if self.is_reasoning else None
            if grammar:
                # llama-server accepts `grammar`; SGLangAPIServer (our wrapper)
                # accepts both `grammar` and `ebnf`. Send `grammar` for both.
                body['grammar'] = grammar

            # Structured-output constraint passed by the caller (the ReAct
            # loop sends REACT_ACTION_SCHEMA). vLLM 0.21 / SGLang honor the
            # OpenAI-style response_format=json_schema; it forces complete,
            # schema-valid JSON and enforces the thought's maxLength, so a
            # runaway thought can't burn the token budget before the tool
            # field is emitted. The bench's vLLM confirmed it works
            # alongside MTP speculative decoding. Cloud gets a different
            # shape — see the is_cloud branch below.
            if response_schema is not None:
                body['response_format'] = {
                    'type': 'json_schema',
                    'json_schema': {'name': 'react_action', 'schema': response_schema},
                }

            # Per-request thinking suppression for Qwen3.x and similar jinja
            # templates that auto-prefix <think>. Setting enable_thinking=False
            # tells the chat template NOT to open the thinking block, so the
            # model goes straight to the answer. Soft `/think` `/nothink`
            # directives don't work on Qwen3.6 — only this template kwarg does.
            if enable_thinking is False:
                body['chat_template_kwargs'] = {'enable_thinking': False}

        elif response_schema is not None:
            # Cloud gets the same schema, minus strict mode.
            #
            # Grammar and chat_template_kwargs are genuinely local-only —
            # cloud rejects them. response_format is not, and bundling all
            # three behind one guard left cloud as the only route with no
            # structured-output constraint at all. What that produced on
            # gpt-5.6-luna, measured 2026-08-16 across two runs: 160
            # `ERROR: unknown tool None` against 57 OK observations — the
            # model emitting its ANSWER as JSON (`{"text": ...}`,
            # `{"creation_occurrences": ...}`) rather than an action.
            # required:[thought,tool] is exactly the constraint against
            # that, and local, which has always had it, produced zero such
            # emissions in 51 calls.
            #
            # strict=true is not available: it demands
            # additionalProperties=false, and this schema allows extras
            # precisely so each tool's own args (text, query, x, radius)
            # pass through. Non-strict is best-effort rather than
            # guaranteed — 3/3 compliant when probed against the real
            # subagent prompt, where json_object dropped `tool` 1/3.
            body['response_format'] = {
                'type': 'json_schema',
                'json_schema': {'name': 'react_action',
                                'schema': response_schema},
            }

        headers: Dict[str, str] = {'Content-Type': 'application/json'}
        if self._api_key_value:
            headers['Authorization'] = f'Bearer {self._api_key_value}'
        # Apply whatever this endpoint taught us on an earlier call.
        for old, new in self._param_renames.items():
            if old in body:
                body[new] = body.pop(old)
        for dead in self._param_drops:
            body.pop(dead, None)

        resp = self._post_adapting(url, headers, body)
        if not resp.ok:
            # Surface the provider's actual error reason (xAI/OpenAI return
            # JSON like {"error":{"message":"...","type":"..."}}); requests'
            # default HTTPError only carries the status line.
            raise RuntimeError(
                f'{resp.status_code} {resp.reason} for {url}: '
                f'{resp.text[:1000]}')
        data = resp.json()
        choices = data.get('choices') or []
        if not choices:
            raise RuntimeError(f'no choices in chat response: {data}')

        # Raw-response diagnostic. Captures content + reasoning_content
        # (the latter only present when llama-server has
        # --reasoning-format set, or SGLang has reasoning_parser set).
        # Lets us see what the engine actually emitted before any
        # client-side <think> stripping or grammar-related contortions.
        choice = choices[0]
        self.last_finish_reason = choice.get('finish_reason')
        try:
            logger.info(
                "<llm-raw> profile=%s grammar=%s finish=%s message=%s",
                cot_profile or '(none)',
                'on' if grammar else 'off',
                choice.get('finish_reason'),
                json.dumps(choice.get('message') or {}, ensure_ascii=False),
            )
        except Exception:
            pass

        msg = choice.get('message') or {}
        text = msg.get('content', '')
        if isinstance(text, list):
            text = ''.join(p.get('text', '') for p in text if isinstance(p, dict))
        # Strip leftover reasoning blocks if the server didn't already
        # (SGLangAPIServer with reasoning_parser does; llama-server does not).
        if isinstance(text, str):
            if '</think>' in text:
                text = text.split('</think>', 1)[1].lstrip()
            elif text.lstrip().startswith('<think>'):
                text = ''
        # Client-side stop emulation. On cloud paths we suppressed
        # body['stop'] above (xAI grok-4.3 rejects it); on local paths
        # the server already truncated and the markers won't appear
        # here — so this loop is idempotent. Replicates the server's
        # truncate-at-first-stop behavior so the prompt's "</end>"
        # anchor still drops trailing prose regardless of whether the
        # provider honored the wire field.
        if stops and isinstance(text, str):
            earliest = -1
            for s in stops:
                if not s:
                    continue
                idx = text.find(s)
                if idx != -1 and (earliest == -1 or idx < earliest):
                    earliest = idx
            if earliest != -1:
                text = text[:earliest]
        return text or ''
