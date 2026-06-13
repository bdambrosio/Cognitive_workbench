"""ReAct loop, action parsing, and built-in tool runners — ReactMixin
for ChatLoop, moved verbatim from chat_loop.py in the 2026-06 mixin
refactor."""

from __future__ import annotations

import json
import logging
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import requests

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from utils.json_utils import repair_json_string  # noqa: E402

logger = logging.getLogger('chat_loop')

# ReAct loop: maximum iterations per turn before forcing a fallback
# synthesis. Empirically (263 chat turns audited 2026-05-03) p95 is 2
# iters and the cap had never been hit — raised 8→12 to add headroom
# for hypothetical multi-tool sequences and more ambitious autonomous
# concerns without introducing room for runaway loops to wander. 12
# also matches inspect's inner cap, removing the asymmetry where the
# outer loop had fewer iters than each subagent it could invoke.
REACT_MAX_ITERS = 12

# Per-iteration budget for re-emitting a malformed/truncated action. A
# parse failure (commonly a `thought` that ran long and got cut off by the
# token limit before the JSON closed) is retried WITHOUT consuming an
# action iteration — a format stumble shouldn't bleed the REACT_MAX_ITERS
# budget. After this many retries the loop bails to fallback synthesis.
REACT_MAX_FORMAT_RETRIES = 2

# Structured-output schema for a ReAct action emission. Sent as
# response_format=json_schema on local engines (vLLM 0.21 / SGLang honor
# it; vLLM ignores the legacy top-level guided_* fields). Forces complete,
# schema-valid JSON every emission and bounds `thought` so a runaway
# reasoning dump can't eat the token budget before the `tool` field lands.
# maxLength is in CHARACTERS (no token-based cap exists in JSON schema):
# 4000 chars ≈ a generous ~1024-token thought — room to reason on hard
# questions, but a hard ceiling well under the 8192-token call budget.
# additionalProperties=True so each tool's own args (query, source,
# content, prompt, …) pass through unconstrained.
REACT_ACTION_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {
        "thought": {"type": "string", "maxLength": 4000},
        "tool": {"type": "string"},
    },
    "required": ["thought", "tool"],
    "additionalProperties": True,
}

# Tools the model can emit. Validated structurally in _parse_react_action;
# the dispatcher in _run_react_loop knows how to run each.
_REACT_TOOLS = ('process_text', 'recall', 'inspect', 'inspect_external', 'security', 'display', 'respond')

# Per-iteration auto-binding: $step1, $step2, ... names the result of each
# action so subsequent actions can reference it. Scoped to the current turn
# only — does not leak into conversation history or the next turn's loop.
_REACT_BINDING_RE = re.compile(r'^\$step\d+$')


class ReactMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # ReAct loop — single-action-per-iteration tool use for chat.
    #
    # Tools: process_text (LLM transformation), discovered tools from
    # src/tools/ (search-web, fetch-text, calculate, …), respond (exit).
    # Each emit is a single JSON object; results auto-bind to $step1, $step2…
    # Variable scope is per-turn; nothing leaks into conversation history.
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_react_action(raw: str) -> Optional[Dict[str, Any]]:
        """Extract a single JSON action object from LLM output via the
        shared tolerant parser (fences, missing braces, bareword keys,
        etc). Returns None if no dict can be recovered."""
        if not raw or not isinstance(raw, str):
            return None
        obj = repair_json_string(raw)
        return obj if isinstance(obj, dict) else None

    @staticmethod
    def _resolve_react_value(val: Any, log: List[Tuple[str, str]]) -> str:
        """Literal string passes through; `$stepN` looks up the log.

        Strips the OK:/EMPTY:/ERROR: observation prefix from bound values
        on resolve. Rationale: the prefix is informational signal for the
        LLM reading the working log (success vs failure discrimination),
        but it should not appear in downstream substitutions — when the
        agent does `respond.text = "$step2"`, the user shouldn't see
        "OK: <content>" leaking through. The LLM has already discriminated
        in the log; substitution gives clean content."""
        if not isinstance(val, str):
            return str(val) if val is not None else ''
        if _REACT_BINDING_RE.match(val):
            content = ''
            for label, c in log:
                if label == val:
                    content = c
                    break
            for tag in ('OK: ', 'EMPTY: ', 'ERROR: '):
                if content.startswith(tag):
                    return content[len(tag):]
            return content
        return val

    def _diagnose_process_text_args(self, raw_src: Any, resolved_src: str,
                                    instruction: Any,
                                    log: List[Tuple[str, str]]
                                    ) -> Optional[str]:
        """Return a diagnostic reason string when process_text args look
        malformed — section-name placeholder as `source`, unresolved
        `$stepN` binding, or empty fields. Returns None when args are
        usable. Caller (the ReAct dispatch) wraps the returned reason
        with the ERROR: prefix, so emit naked sentences here. Each
        diagnostic names the failure AND points at the recovery path —
        the model has been observed to recover cleanly when given a
        legible error."""
        if not isinstance(instruction, str) or not instruction.strip():
            return ("requires a non-empty `instruction` field; "
                    "no instruction was supplied.")
        # Unresolved $stepN: model passed a binding that doesn't exist.
        if isinstance(raw_src, str) and _REACT_BINDING_RE.match(raw_src) and not resolved_src:
            bound = sorted({lab for lab, _ in log
                            if _REACT_BINDING_RE.match(lab)})
            avail = ", ".join(bound) if bound else "(none yet this turn)"
            return (f"`source` is `{raw_src}`, an unresolved binding. "
                    f"Available `$stepN` bindings this turn: {avail}. "
                    f"Pass an existing binding or literal inline text instead.")
        if not resolved_src:
            return ("`source` is empty. Pass either literal text content "
                    "(the actual material to process) or a `$stepN` binding "
                    "from a prior tool call this turn.")
        # Heading-shaped placeholder: starts with `##` and is short.
        # Real content the model wants processed is typically much
        # longer; a section heading is rarely a reasonable source.
        stripped = resolved_src.lstrip()
        if stripped.startswith('##') and len(resolved_src) < 500:
            return ("`source` appears to be a section heading like "
                    "`## Conversation history` rather than content. "
                    "Section bodies are already in your system prompt — "
                    "read them directly in `respond` rather than passing "
                    "the heading as `source`. Section names do not "
                    "resolve to their bodies; `source` accepts only "
                    "literal inline text or a `$stepN` binding.")
        return None

    def _run_process_text(self, source_text: str, instruction: str) -> str:
        """Apply a focused LLM pass to source_text using the given instruction.
        Used by the ReAct process_text tool.

        The persona block is passed as system context so output respects
        Jill's voice (fair witness, no sycophancy, grounded specifics). The
        citation rule is added explicitly because the persona doesn't
        codify it: when the source contains a 'Sources:' block (the format
        web-search results use), the output must cite by domain or
        publication name rather than 'the source' / 'the writeup'.
        """
        has_sources = isinstance(source_text, str) and 'Sources:' in source_text
        sys_parts = [
            f"You are {self.character_name}. The user has asked you to apply an "
            "instruction to a source text. Output ONLY the transformed result — "
            "no preamble, no commentary, no headers, no markdown fences."
        ]
        if self.persona:
            sys_parts.append("## Persona (governs voice and stance)\n" + self.persona)
        if has_sources:
            sys_parts.append(
                "## Citation requirement\n"
                "The source contains a 'Sources:' block. Cite by real "
                "publication / domain when available (e.g. \"per weather.com\", "
                "\"according to NOAA\", \"per ir.tesla.com\"). Never use "
                "generic phrases like \"the source\" or \"the writeup\". If "
                "a source line lacks a domain or carries '(no public URL)', "
                "do NOT invent a domain and do NOT forward the raw URL — say "
                "something honest like \"per the weather service consulted\" "
                "or attribute by the source title. Never paste an opaque "
                "internal reference (e.g. weather://turn0forecast0) into "
                "your output. If the source did not address the instruction, "
                "say so plainly rather than improvise."
            )
        sys_msg = "\n\n".join(sys_parts)
        user_msg = f"INSTRUCTION:\n{instruction}\n\nSOURCE:\n{source_text}"
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.4, cot_profile='none',
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] process_text failed: {e}")
            return f"ERROR: process_text raised: {e}"
        text = (result or '').strip()
        if not text:
            return 'EMPTY: process_text produced no output'
        return 'OK: ' + text

    def _emit_status(self, msg: str) -> None:
        """Overwrite the current terminal line with a transient status.
        No-op when stdout isn't a TTY (logs / piped output) — printed
        per-iteration log lines via logger.info already cover that case."""
        if not sys.stdout.isatty():
            return
        try:
            line = f"\r{self._STATUS_DIM}[{self.character_name}] {msg}{self._STATUS_RESET}"
            # Pad with spaces so a shorter status overwrites leftover chars
            # from a longer previous one. The \r places the cursor back at
            # column 0; the next write (or _clear_status) replaces this.
            sys.stdout.write(line.ljust(self._STATUS_PAD))
            sys.stdout.flush()
        except Exception:
            pass  # status is non-essential; never raise

    def _clear_status(self) -> None:
        """Clear the status line and return cursor to column 0."""
        if not sys.stdout.isatty():
            return
        try:
            sys.stdout.write('\r' + ' ' * self._STATUS_PAD + '\r')
            sys.stdout.flush()
        except Exception:
            pass

    def _run_react_loop(self, source: str, user_text: str, orientation: str,
                        recall: Optional[List[Tuple[str, str, str]]] = None,
                        agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        image_url: Optional[str] = None
                        ) -> Tuple[str, List[Tuple[str, str]], List[Dict[str, Any]], str]:
        """Run the ReAct loop. Returns (reply, log, iters, exit_reason).
        The trace is NOT written here — caller writes it after post-turn
        reflection so the trace can include memories written."""
        log: List[Tuple[str, str]] = []
        # Per-iteration record:
        #   system   — full system prompt sent (constant across iters,
        #              captured here for trace fidelity)
        #   user     — full user message sent (grows with the working log)
        #   raw      — model's full raw emission (thought + tool call)
        #   appended — log entries this iter's parsed action contributed to
        #              the working log for the NEXT iter to see. May be 0
        #              entries (respond / loop-exit) or 2 entries (action
        #              + observation) or 1 entry (NOTE on parse error).
        # The model's raw emission and `appended` differ — the difference
        # is exactly the reasoning that got "compressed away" before the
        # next iter saw the context.
        iters: List[Dict[str, Any]] = []
        # Store-and-append architecture for KV-cache stability:
        #   • system_prompt_str and user_prefix_str are built ONCE at loop
        #     start and reused literally on every iteration — never rebuilt.
        #   • log_appendage_str grows by literal string append in lockstep
        #     with the log list. We never reformat past entries.
        #   • Each iter's user message = user_prefix_str + log_appendage_str
        #     + trailer. All three are stored strings; the only operation
        #     between iterations is concatenation of stored pieces.
        # This guarantees a byte-stable prefix across iterations so the
        # backend's KV cache hits on every iter past the first.
        now_str = datetime.now().astimezone().strftime("%A, %B %d %Y, %I:%M %p %Z")
        system_prompt_str = self._build_react_system_prompt(
            source, orientation, now_str, recall=recall,
            agent_concerns=agent_concerns, user_concerns=user_concerns)
        user_prefix_str = self._build_react_user_prefix(source, user_text)
        log_appendage_str = ""
        trailer = "\nEmit next action:"

        def _append_log(label: str, content: str) -> None:
            """Lockstep append: list and string grow together. The string
            format must match what _build_react_user_prefix would have
            rendered for these entries (LABEL:\\nCONTENT\\n)."""
            nonlocal log_appendage_str
            log.append((label, content))
            log_appendage_str += f"{label}:\n{content}\n"

        # Track whether the agent explicitly pushed something to the canvas
        # this turn. If she did, leave it alone — `display` content is richer
        # than the prose reply. If she didn't, auto-mirror the respond text
        # to canvas as markdown so the surface always reflects the latest
        # reply rather than going stale.
        did_display = False

        self._affect.enter_loop()
        for i in range(REACT_MAX_ITERS):
            self._affect.set_react_iter(i + 1)
            # Emit one parseable action. A malformed/truncated emission is
            # retried here WITHOUT consuming this action iteration (bounded by
            # REACT_MAX_FORMAT_RETRIES) — a format stumble shouldn't cost a
            # tool turn. Each attempt rebuilds usr_msg so the corrective NOTE
            # appended by the previous attempt is in view.
            action = None
            for fmt_attempt in range(REACT_MAX_FORMAT_RETRIES + 1):
                pre_log_len = len(log)
                usr_msg = user_prefix_str + log_appendage_str + trailer
                # When the turn carries an image, every iter sends a
                # multimodal content array (text first per vLLM's expected
                # ordering, then image_url). The image must persist across
                # iters: ReAct's parse-failure retry path drops back into
                # iter 2+, and if the image were missing there, the model
                # confabulates against its own prior text. Byte-stable image
                # tokens across iters still let vLLM prefix-cache the text.
                # No-image turns keep the plain-string content shape exactly
                # as before — wire format byte-identical to pre-multimodal.
                if image_url:
                    user_content: Any = [
                        {'type': 'text', 'text': usr_msg},
                        {'type': 'image_url', 'image_url': {'url': image_url}},
                    ]
                else:
                    user_content = usr_msg
                prompt = [
                    {'role': 'system', 'content': system_prompt_str},
                    {'role': 'user', 'content': user_content},
                ]
                self._emit_status('thinking…')
                self._affect.set_waiting_for_llm(True)
                try:
                    raw = self.backend.chat(prompt, max_tokens=self.react_max_tokens,
                                            temperature=0.7, cot_profile='none',
                                            response_schema=REACT_ACTION_SCHEMA)
                except Exception as e:
                    logger.error(f"[{self.character_name}] ReAct iter {i+1} LLM failed: {e}")
                    iters.append({'system': system_prompt_str, 'user': usr_msg,
                                  'raw': f'(LLM call failed: {e})', 'appended': []})
                    reply = self._react_fallback_synthesis(log, str(e))
                    self._clear_status()
                    self._affect.exit_loop()
                    return reply, log, iters, 'llm_error'
                finally:
                    self._affect.set_waiting_for_llm(False)

                iters.append({'system': system_prompt_str, 'user': usr_msg, 'raw': raw or '',
                              'appended': []})

                action = self._parse_react_action(raw)
                if action is not None:
                    break

                # Parse failed. Distinguish a token-limit truncation (the
                # common case: a `thought` that ran long and got cut off
                # before the JSON closed) from genuinely malformed output,
                # and steer the retry accordingly.
                truncated = self.backend.last_finish_reason in ('length', 'max_tokens')
                logger.warning(
                    f"[{self.character_name}] ReAct iter {i+1} fmt-retry "
                    f"{fmt_attempt + 1}/{REACT_MAX_FORMAT_RETRIES}: unparseable "
                    f"(finish={self.backend.last_finish_reason}): {raw[:160]!r}")
                self._emit_status('parse failed, retrying…')
                self._affect.incr_llm_retry()
                if truncated:
                    note = ("Your previous emission was cut off by the token limit before the "
                            "JSON action closed — it was too long. Emit ONE COMPLETE, COMPACT "
                            "JSON action now: a SHORT thought (a single clause), the `tool`, and "
                            "concise args. Keep EVERY field brief — do not dump analysis or "
                            "step-by-step work into `thought` or into any argument value. Work "
                            "the problem in small steps across iterations instead.")
                else:
                    note = ("Previous output was not valid JSON. Do NOT apologize — emit ONE JSON "
                            "action now: a single object with a short `thought` and a `tool`.")
                _append_log('NOTE', note)
                iters[-1]['appended'] = log[pre_log_len:]

            if action is None:
                # Out of format-retry budget without a parseable action —
                # don't burn the rest of the turn; synthesize from the log.
                logger.warning(
                    f"[{self.character_name}] ReAct iter {i+1}: no parseable action after "
                    f"{REACT_MAX_FORMAT_RETRIES + 1} attempts; bailing to fallback synthesis")
                break

            tool = action.get('tool')
            if tool == 'respond':
                text = self._resolve_react_value(action.get('text', ''), log)
                logger.info(f"[{self.character_name}] ReAct iter {i+1}: respond ({len(text)} chars)")
                # respond appends nothing to the log (loop exits); appended stays []
                self._clear_status()
                # Auto-mirror: when the agent didn't push richer content
                # via `display` this turn, render the respond text on the
                # canvas as markdown. Keeps the surface in sync with the
                # latest reply for free.
                if not did_display and text and text.strip():
                    try:
                        self._canvas.set_content(text, fmt='markdown')
                    except Exception as e:
                        logger.warning(
                            f"[{self.character_name}] canvas auto-mirror failed: {e}")
                self._affect.mark_completion()
                self._affect.exit_loop()
                return text, log, iters, 'respond'

            self._emit_status(f'using {tool}…')
            # Tool-call signature for repeat detection. Truncated to avoid
            # giant payloads in the publisher state. Sort keys so equivalent
            # actions hash the same regardless of dict ordering.
            _tool_args_sig = json.dumps(
                {k: v for k, v in action.items() if k != 'tool'},
                sort_keys=True, default=str)[:200]
            self._affect.set_tool(tool, args_sig=_tool_args_sig)
            if tool == 'recall':
                self._affect.set_memory_op('recall')
            binding = f'$step{i+1}'
            _append_log(f'ACTION {i+1}', json.dumps(action))

            # ----------------------------------------------------------
            # Tool-observation convention (KISS, prose-only, no runtime
            # layering). Every observation string emitted by a tool starts
            # with one of three prefixes so the ReAct LLM can discriminate
            # success from failure without keyword-spotting:
            #   OK: <content>     — tool succeeded, content follows
            #   EMPTY: <reason>   — tool ran without error but produced no
            #                       usable result (no search hits; remember
            #                       found nothing; empty argument)
            #   ERROR: <reason>   — tool failed (unavailable, raised,
            #                       returned malformed). Treat as "this
            #                       tool is currently broken" — do not
            #                       retry the same call.
            # New tools must follow the same convention. See src/chat/
            # AGENTS.md for the author-facing rule.
            # ----------------------------------------------------------
            if tool == 'process_text':
                raw_src = action.get('source', '')
                ins = action.get('instruction', '')
                src = self._resolve_react_value(raw_src, log)
                diag = self._diagnose_process_text_args(raw_src, src, ins, log)
                if diag is not None:
                    logger.warning(f"[{self.character_name}] process_text "
                                   f"dispatch rejected (iter {i+1}): {diag}")
                    # Diagnose returns a free-form reason — wrap as ERROR
                    # so the agent treats it the same as any other tool
                    # failure rather than mistaking it for content.
                    obs = f'ERROR: process_text rejected: {diag}'
                else:
                    obs = self._run_process_text(src, ins)
            elif tool == 'recall':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_remember(q)
            elif tool == 'inspect':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_inspect(q)
            elif tool == 'inspect_external':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_inspect_external(q)
            elif tool == 'security':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_security(q)
            elif tool == 'display':
                content = self._resolve_react_value(action.get('content', ''), log)
                fmt = (action.get('format') or 'markdown').strip().lower()
                obs = self._run_display(content, fmt)
                # Suppress auto-mirror on the final respond — the agent
                # has already chosen what belongs on the canvas this turn.
                if isinstance(obs, str) and obs.startswith('OK'):
                    did_display = True
            elif tool in self._discovered_tools:
                obs = self._dispatch_discovered_tool(tool, action, log)
            else:
                # Tool list shown to the model on bad emission. Built-ins
                # are stable; the discovered set comes from the registry.
                builtin = ["process_text", "recall", "inspect", "security",
                           "display", "respond"]
                if self._get_external_repo() is not None:
                    builtin.insert(builtin.index("inspect") + 1, "inspect_external")
                avail = ", ".join(builtin + sorted(self._discovered_tools.keys()))
                obs = f"ERROR: unknown tool {tool!r}; available: {avail}"

            _append_log(binding, obs)
            iters[-1]['appended'] = log[pre_log_len:]
            # Tool observations follow the documented OK:/EMPTY:/ERROR: prefix
            # protocol (see 4170–4184). ERROR: is the only failure signal and
            # is a literal prefix, not a keyword classification.
            if isinstance(obs, str) and obs.startswith('ERROR'):
                self._affect.incr_tool_error()
            self._affect.set_memory_op('none')
            self._affect.set_tool(None)
            logger.info(f"[{self.character_name}] ReAct iter {i+1}: {tool} → {binding} ({len(obs)} chars)")

        logger.warning(f"[{self.character_name}] ReAct hit max iters ({REACT_MAX_ITERS})")
        reply = self._react_fallback_synthesis(log)
        self._clear_status()
        self._affect.exit_loop()
        return reply, log, iters, 'max_iters'

    def _subagent_traces_dir(self) -> 'Path':
        """Where the active-recall subagent writes its per-call traces.
        Peer of memory/, NOT under it — keeps subsequent recall calls from
        reading their own prior traces as substrate. Layout:
        scenarios/<world>/<agent>/subagent_traces/."""
        return self._memory_dir().parent / 'subagent_traces'

    def _persist_image_url(self, image_url: str) -> Optional[str]:
        """Write a data-URI-encoded image to <memory>/images/<sha>.<ext>
        once (deduped by sha256) and return the relative path string.
        Returns None for http(s) URLs (the URL itself is the trace
        identifier; we don't fetch). Raises on a malformed data URI."""
        if image_url.startswith(('http://', 'https://')):
            return None
        m = self._DATA_URI_RE.match(image_url)
        if not m:
            raise ValueError("image_url is neither http(s) URL nor a base64 data URI")
        mime, b64 = m.group(1), m.group(2)
        import base64, hashlib
        try:
            raw = base64.b64decode(b64, validate=False)
        except Exception as e:
            raise ValueError(f"data URI base64 decode failed: {e}")
        sha = hashlib.sha256(raw).hexdigest()
        ext = {
            'image/png': 'png', 'image/jpeg': 'jpg', 'image/jpg': 'jpg',
            'image/gif': 'gif', 'image/webp': 'webp',
        }.get(mime.lower(), 'bin')
        img_dir = self._memory_dir() / 'images'
        img_dir.mkdir(parents=True, exist_ok=True)
        out = img_dir / f'{sha}.{ext}'
        if not out.exists():
            out.write_bytes(raw)
        return str(out)

    def _run_remember(self, query: str) -> str:
        """Backend for the ReAct `recall` tool. Delegates to the
        active-recall subagent (chat.remember.remember), which runs its
        own thin ReAct loop over the memory dir. Returns an OK:/EMPTY:/
        ERROR: prefixed observation per the ReAct tool-observation
        convention. Subagent's full per-call trace lands under
        subagent_traces/ for debugging.

        Stage 4b: passes the active-threads inventory to the subagent
        as a system-prompt extension. The subagent gets activity-level
        structure for free without needing to grep files for it."""
        if not query or not str(query).strip():
            return "EMPTY: remember query was empty"
        try:
            from chat.remember import remember as _remember
            threads = self._get_threads(statuses=('active',))
            thread_context = self._render_threads_for_subagent(threads)
            answer = _remember(
                query=str(query),
                memory_dir=self._memory_dir(),
                llm_backend=self.backend,
                trace_dir=self._subagent_traces_dir(),
                thread_context=thread_context,
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] remember subagent raised: {e}")
            return f"ERROR: remember subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: remember subagent produced no answer'
        return 'OK: ' + text

    def _preflight_proxy_imgs(self, content: str) -> Optional[str]:
        """If `content` references the local image proxy, GET each URL
        ahead of canvas publish. The proxy's Content-Type guard returns
        415 for non-image upstreams (most commonly a webpage URL the
        model mistakenly piped into <img>); preflight relays that to the
        ReAct loop as an ERROR observation so the next iteration can
        re-strategize instead of leaving a broken-icon on the canvas.

        Returns None if all proxy URLs validated, or a short reason
        string if any failed. Caller wraps with `ERROR:` prefix.
        """
        for m in self._PROXY_IMG_RE.finditer(content):
            url = m.group(1)
            try:
                r = requests.get(url, timeout=5)
            except requests.RequestException as e:
                return f"proxy unreachable for {url}: {e}"
            if r.status_code != 200:
                # The proxy emits an actionable hint in the body
                # (e.g. "upstream returned 'text/html', not an image…").
                # Relay it verbatim so Jill sees what to do next.
                hint = (r.text or '').strip().replace('\n', ' ')[:400]
                return (
                    f"proxy rejected {url} (HTTP {r.status_code})"
                    + (f" — {hint}" if hint else '')
                )
        return None

    def _run_display(self, content: str, fmt: str) -> str:
        """Backend for the ReAct `display` tool. Publishes the content to
        the canvas Zenoh key; the display window (launched separately or
        via launcher --canvas) renders markdown or HTML. Non-terminal:
        the ReAct loop continues after this — Jill should follow up with
        a `respond` so the conversation gets a textual reply too."""
        if content is None:
            return "ERROR: display content was None"
        body = str(content)
        if not body.strip():
            return "EMPTY: display content was empty"
        if fmt not in ('markdown', 'html'):
            fmt = 'markdown'
        # Validate any proxy URLs before publish so a page-URL-as-image
        # bug surfaces here, not as a broken-icon on the user's screen.
        if fmt == 'html':
            err = self._preflight_proxy_imgs(body)
            if err is not None:
                return f"ERROR: display preflight: {err}"
        try:
            n_bytes = self._canvas.set_content(body, fmt=fmt)
        except Exception as e:
            logger.warning(f"[{self.character_name}] canvas publish failed: {e}")
            return f"ERROR: canvas publish failed: {e}"
        return f"OK: rendered to canvas ({n_bytes} bytes, format={fmt})"

    def _inspect_traces_dir(self) -> 'Path':
        """Where the inspect (codebase-query) subagent writes its per-call
        traces. Sibling of subagent_traces/ — kept separate so per-tool
        debugging stays clean. Layout:
        scenarios/<world>/<agent>/inspect_traces/."""
        return self._memory_dir().parent / 'inspect_traces'

    def _run_inspect(self, query: str) -> str:
        """Backend for the ReAct `inspect` tool. Delegates to the
        codebase-inspection subagent (chat.code_subagent.inspect), which
        runs its own thin ReAct loop over src/ with read-only primitives
        (list, read, grep via ripgrep). Returns an OK:/EMPTY:/ERROR:
        prefixed observation per the ReAct tool-observation convention.

        Uses Jill's main backend (self.backend) — the same model the
        outer ReAct loop runs on. Per-scenario YAML decides the model;
        no per-subagent backend overrides. Per-call traces under
        inspect_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: inspect query was empty"
        try:
            from chat.code_subagent import inspect as _inspect
            answer = _inspect(
                query=str(query),
                repo_root=Path(_SRC_DIR),
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] inspect subagent raised: {e}")
            return f"ERROR: inspect subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: inspect subagent produced no answer'
        return 'OK: ' + text

    def _run_inspect_external(self, query: str) -> str:
        """Backend for the ReAct `inspect_external` tool. Delegates to the
        same subagent loop as `inspect`, geofenced to the bound external
        repo (see _set_external_repo / _get_external_repo). Returns ERROR
        if no repo is currently bound — the system prompt only advertises
        this tool when a binding exists, so a call here without binding
        means the model fabricated the tool."""
        if not query or not str(query).strip():
            return "EMPTY: inspect_external query was empty"
        repo = self._get_external_repo()
        if repo is None:
            return ("ERROR: no external repo is bound for this session "
                    "(use /set-external-repo <path> first)")
        try:
            from chat.code_subagent import inspect_external as _inspect_external
            answer = _inspect_external(
                query=str(query),
                repo_root=repo,
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] inspect_external subagent raised: {e}")
            return f"ERROR: inspect_external subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: inspect_external subagent produced no answer'
        return 'OK: ' + text

    def _security_traces_dir(self) -> 'Path':
        """Where the security (network-investigation) subagent writes its
        per-call traces. Sibling of inspect_traces/ — kept separate so
        per-tool debugging stays clean. Layout:
        scenarios/<world>/<agent>/security_traces/."""
        return self._memory_dir().parent / 'security_traces'

    def _run_security(self, query: str) -> str:
        """Backend for the ReAct `security` tool. Delegates to the
        network-security investigation subagent (chat.security.security),
        which runs its own thin ReAct loop with read-only typed primitives
        (discover, scan_services, system_state via nmap + iproute2).
        Returns an OK:/EMPTY:/ERROR: prefixed observation per the ReAct
        tool-observation convention.

        Uses Jill's main backend (self.backend) — see _run_inspect for
        rationale. Per-call traces under security_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: security query was empty"
        try:
            from chat.security import security as _security
            answer = _security(
                query=str(query),
                llm_backend=self.backend,
                trace_dir=self._security_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] security subagent raised: {e}")
            return f"ERROR: security subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: security subagent produced no answer'
        return 'OK: ' + text

    def _react_fallback_synthesis(self, log: List[Tuple[str, str]], fail_reason: str = '') -> str:
        """Force a Jill-voice reply when respond never fired."""
        if not log:
            return f"(I couldn't formulate a response.{(' — ' + fail_reason) if fail_reason else ''})"
        summary = "\n".join(f"{label}: {content[:400]}" for label, content in log)
        sys_msg = (f"You are {self.character_name}, speaking in first person. The ReAct "
                   "loop did not exit cleanly. Synthesize the log into a reply, in your voice. "
                   "If incomplete, say so honestly rather than fabricate.")
        user_msg = (f"Log:\n{summary}"
                    + (f"\n\n(Failure: {fail_reason})" if fail_reason else "")
                    + "\n\nWrite the response now. No preamble.")
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg}, {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.7, cot_profile='none')
            return (result or '').strip() or "(could not synthesize)"
        except Exception as e:
            return f"(trouble formulating a response: {e})"
