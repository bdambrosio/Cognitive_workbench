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
# Geofence root for the self-inspect subagent. The repo root, not _SRC_DIR:
# docs/, tests/, and bench/ are part of the substrate she is asked about,
# and scoping to src/ left her unable to read her own design docs. Safe to
# widen because the subagent is git-aware inside a checkout — list uses
# `git ls-files`, read refuses gitignored paths, and grep is ripgrep with
# gitignore honored. scenarios/*/ is gitignored, so live world state
# (resources.json, autonomy.jsonl, disposition_state.jsonl) stays out of
# reach; only the 16 tracked scenario YAMLs are visible.
_REPO_ROOT = os.path.dirname(_SRC_DIR)

from utils.json_utils import (repair_json_string,  # noqa: E402
                              unparseable_action_note)

logger = logging.getLogger('chat_loop')

# ReAct loop: maximum iterations per turn before forcing a fallback
# synthesis. Empirically (263 chat turns audited 2026-05-03) p95 is 2
# iters and the cap had never been hit — raised 8→12 to add headroom
# for hypothetical multi-tool sequences and more ambitious autonomous
# concerns without introducing room for runaway loops to wander.
#
# 12→16 on 2026-08-23. The dataroom fixture is the first task that reaches
# the cap as a matter of course rather than by wandering: an arm read the
# method, listed a 9-document corpus and read the documents one per call,
# and hit 12 at the exact iteration ingestion finished — sound work, no
# repeats, no budget left to do the job it had just prepared for. Arms that
# batch reads (3 docs per call) finish comfortably, so the cap was
# selecting for batching rather than for reasoning.
#
# This DELIBERATELY breaks the old parity with inspect's inner cap (still
# 12, code_subagent._MAX_ITERS). The asymmetry that mattered was the outer
# loop having FEWER iterations than a subagent it invokes; outer > inner is
# the harmless direction.
REACT_MAX_ITERS = 16

# Output ceiling for one process_text pass. Reasoning tokens bill
# against this same budget on a thinking model, so raising it is not
# free and lowering it truncates rewrites — see _run_process_text.
_PROCESS_TEXT_MAX_TOKENS = 4096

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
_REACT_TOOLS = ('process_text', 'recall', 'inspect', 'inspect_external', 'security', 'justify', 'agent-say', 'display', 'respond', 'yield')

# Per-iteration auto-binding: $step1, $step2, ... names the result of each
# action so subsequent actions can reference it. Scoped to the current turn
# only — does not leak into conversation history or the next turn's loop.
_REACT_BINDING_RE = re.compile(r'^\$step\d+$')

# Above this length a `source` is material by weight; below it the
# reference probe decides. Keeps the probe off the common path.
_REFERENCE_PROBE_MAX_CHARS = 200


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
        `$stepN` binding, empty fields, or a `source` that references
        material instead of carrying it. Returns None when args are
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
        # Prose pointer ("Recent reasoning traces #2728 through #2733"):
        # nothing resolves it, so the call transforms the pointer and writes
        # the rest from the instruction alone — a synthesis of material it
        # never saw. The tell is meaning, not shape, hence a probe. A
        # resolved $stepN came from a tool and is material by construction.
        is_binding = isinstance(raw_src, str) and _REACT_BINDING_RE.match(raw_src.strip())
        if not is_binding and len(resolved_src) <= _REFERENCE_PROBE_MAX_CHARS:
            if self._source_is_reference(resolved_src, instruction):
                return ("`source` names where material lives rather than "
                        "carrying it, so there is nothing to transform — "
                        "turn numbers, note ids and descriptions of prior "
                        "conversation do not resolve to their contents. "
                        "Fetch it first (recall, inspect, or read it in an "
                        "earlier step this turn) and pass that step's "
                        "`$stepN`, or paste the text inline.")
        return None

    def _source_is_reference(self, source_text: str, instruction: str) -> bool:
        """True when `source` points at material instead of being material.
        Fails open: a probe that errors or answers off-format must not make
        process_text unavailable."""
        sys_msg = ("You classify one field of a tool call. Answer with "
                   "exactly one word and nothing else.")
        user_msg = (
            "A tool transforms a SOURCE text according to an INSTRUCTION. "
            "The SOURCE is supposed to BE the material. It is a mistake for "
            "it to be a reference — a phrase naming where material can be "
            "found (a range of turns, a note or step id, a section heading, "
            "an earlier conversation, a file) without containing it.\n\n"
            "Short material is fine and common: a sentence to translate, a "
            "list to reformat, a figure to explain. Length does not decide "
            "this — only whether the text IS the thing or POINTS AT it.\n\n"
            f"INSTRUCTION:\n{instruction}\n\nSOURCE:\n{source_text}\n\n"
            "Answer with one word: material or reference.")
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=8, temperature=0.0, cot_profile='none',
                # Opt out of any scenario baseline. Eight tokens is a
                # verdict, not a deliberation; hand this a thinking channel
                # and it returns empty, which fails open and silently
                # un-guards every process_text call.
                reasoning_effort='none',
            )
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] process_text source probe failed: "
                f"{e}; allowing the call through")
            return False
        verdict = (result or '').strip().strip('.`"\'').lower()
        if verdict.startswith('reference'):
            logger.info(
                f"[{self.character_name}] process_text source probe: "
                f"reference — {source_text[:120]!r}")
            return True
        if not verdict.startswith('material'):
            logger.warning(
                f"[{self.character_name}] process_text source probe returned "
                f"{verdict[:60]!r}, neither verdict; allowing the call through")
        return False

    def _run_process_text(self, source_text: str, instruction: str,
                          cited_sources: Optional[List[Dict[str, Any]]] = None
                          ) -> str:
        """Apply a focused LLM pass to source_text using the given instruction.
        Used by the ReAct process_text tool.

        The persona block is passed as system context so output respects
        Jill's voice (fair witness, no sycophancy, grounded specifics). The
        citation rule is added explicitly because the persona doesn't
        codify it: `cited_sources` is the structured source list attached
        to the $stepN binding this call is processing (set by the ReAct
        dispatch from the step's tool_meta) — when present, the output
        must cite by domain or publication name rather than 'the source'
        / 'the writeup'.
        """
        has_sources = bool(cited_sources)
        sys_parts = [
            f"You are {self.character_name}. The user has asked you to apply an "
            "instruction to a source text. Output ONLY the transformed result — "
            "no preamble, no commentary, no headers, no markdown fences."
        ]
        if self.persona:
            sys_parts.append("## Persona (governs voice and stance)\n" + self.persona)
        # THE METHOD CROSSES THE TOOL BOUNDARY. Without this, a sub-call's
        # entire context is the framing line, the persona and the citation
        # rule — so an instruction reading "use the §5 finding format, the §6
        # verdict vocabulary and the §9 taxonomy" is four dangling pointers,
        # and the sub-model invents plausible formats instead. Observed
        # 2026-08-24: one arm wrote exactly that instruction; another
        # compensated by inlining ~3,000 characters of contract into every
        # call, which is the correct workaround and an expensive one.
        #
        # workflow_text, not the file: load_workflow has already stripped the
        # practice-only sections, so what crosses is what an executing agent
        # is meant to read.
        #
        # PLACED BEFORE THE CONDITIONAL BLOCK ON PURPOSE. Everything above
        # this point is byte-stable across calls, so the provider can cache
        # the prefix; the citation block below varies with the binding.
        wf = getattr(self, 'workflow_text', '')
        if wf:
            sys_parts.append(
                "## Method (the procedure this work follows)\n"
                "The instruction may cite this by section. Follow it as "
                "written; where it fixes a format or a vocabulary, that "
                "format is mandatory and not a suggestion.\n\n" + wf)
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

        # A REWRITE CANNOT ROUND-TRIP A SOURCE LARGER THAN ITS OWN CEILING.
        # Arithmetic only — no guess at what the instruction wants. A summary
        # legitimately outputs far less than it reads, so this cannot be an
        # error; a whole-document edit legitimately outputs about what it
        # reads, and that is the case that silently loses its tail.
        est_in = int(len(source_text) / getattr(
            self.backend, '_CHARS_PER_TOKEN', 3.0))
        if est_in > _PROCESS_TEXT_MAX_TOKENS:
            logger.warning(
                "[%s] process_text: source ~%d tokens against a %d-token "
                "ceiling. An edit-shaped instruction cannot return this "
                "document whole.",
                self.character_name, est_in, _PROCESS_TEXT_MAX_TOKENS)
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=_PROCESS_TEXT_MAX_TOKENS, cot_profile='none',
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] process_text failed: {e}")
            return f"ERROR: process_text raised: {e}"

        # TRUNCATION IS THE DANGEROUS OUTCOME, NOT THE EMPTY ONE. A rewrite cut
        # off at the ceiling looks exactly like a successful edit that removed
        # trailing content — the caller cannot tell "I deleted the footer" from
        # "the footer fell off the end". Reported as ERROR so it is never bound
        # and forwarded as a finished document.
        if self.backend.last_finish_reason in ('length', 'max_tokens'):
            return ('ERROR: process_text output hit the token ceiling and is '
                    'TRUNCATED — the tail is missing, not removed. Do not use '
                    'this result. Process the document in sections, or ask for '
                    'a shorter transformation.')

        text = (result or '').strip()
        if not text:
            return 'EMPTY: process_text produced no output'

        # NO-OP DETECTION. Observed 2026-08-24: an arm issued the same removal
        # instruction ten times against a document process_text kept handing
        # back byte-identical, burning ten of sixteen iterations and finishing
        # one short of the cap. Every call answered `OK:` followed by the whole
        # document, so the only way to learn nothing had changed was to diff
        # 8,000 characters by eye, per iteration. Saying so costs one
        # comparison and ends the loop on the first repeat.
        if text == source_text.strip():
            return ('EMPTY: process_text returned the source UNCHANGED — the '
                    'instruction produced no edit. Reissuing it verbatim will '
                    'return this same text; change the instruction, or work '
                    'with the document as it stands.\n' + text)
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
                        recall: Optional[List[Tuple[str, str, str, Optional[str], Optional[str]]]] = None,
                        agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        image_url: Optional[str] = None
                        ) -> Tuple[str, List[Tuple[str, str]], List[Dict[str, Any]], str]:
        """Run the ReAct loop. Returns (reply, log, iters, exit_reason).
        The trace is NOT written here — caller writes it after post-turn
        reflection so the trace can include memories written."""
        # Episode boundary for canvas scrollback: publishes during this
        # episode (user turn or auto-fire) coalesce to one keyframe.
        self._canvas.new_turn()
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
        # THE WORKING LOG IS NOW EXPLICITLY CLOSED. The prefix opens it with
        # "## Working log" and nothing closed it, so the last observation's
        # content ran straight into this trailer separated by one newline.
        # Harmless for a short observation; not harmless for a long one.
        #
        # Observed 2026-08-24: an arm processing an 8,000-character report
        # read "Emit next action:" as the document's final line and spent ten
        # of sixteen iterations instructing process_text to delete it. The
        # string appears zero times in the delivered report and ten times in
        # the trace — every one inside an instruction asking for its removal.
        # process_text correctly returned the text unchanged each time, which
        # the arm read as the edit failing.
        #
        # A closing header, mirroring the opening one, is the whole fix. It
        # preserves the store-and-append discipline above (appended once, at
        # the end, so the cached prefix is untouched).
        trailer = "\n## End of working log\n\nEmit next action:"

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

        # Most-recent image produced by a tool this turn (e.g. camera-capture),
        # injected into the multimodal content array below so the model can SEE
        # it. Single slot (most-recent-wins) and reset each turn: a fresh frame
        # replaces the old one, and stale frames don't leak across turns.
        self._pending_tool_image = None

        # Structured provenance from the most recent discovered-tool call
        # (source URLs, query, paths). Consumed right after dispatch into
        # the iter record; reset here so stale meta can't leak across turns.
        self._pending_tool_meta = None

        # $stepN binding → structured source list ({url,domain,title,...})
        # for steps whose tool_meta carried sources (search-web). This is
        # the structural signal for process_text's citation discipline —
        # replaces the old `'Sources:' in text` substring test. Per-turn.
        self._step_sources: Dict[str, List[Dict[str, Any]]] = {}

        # Intentional-yield handoff: when an autonomous run exits via the
        # `yield` action, the follow-up instruction lands here for the
        # caller (_process_user_turn) to spawn the successor concern from.
        # Reset each loop so a stale handoff can't leak across turns.
        self._react_yield_next = None

        self._affect.enter_loop()
        for i in range(REACT_MAX_ITERS):
            self._affect.set_react_iter(i + 1)
            # Budget nudge: yield-adherence is decided at the moment the
            # budget runs short, not when the catalog was read ~2k tokens
            # ago. Surface it in the working log while there is still
            # room to hand off cleanly (observed: cut-off max_iters turns
            # with zero yields ever emitted, 2026-07-15).
            if i == REACT_MAX_ITERS - 3:
                _append_log(
                    'NOTE',
                    f"iteration {i + 1} of {REACT_MAX_ITERS} — the action "
                    "budget is nearly exhausted. If the remaining work will "
                    "not fit, do NOT start anything new: emit `yield` NOW "
                    "with a self-contained `next` (what is done, what "
                    "remains, every specific already found) so a follow-up "
                    "run continues the work. If the work is done, `respond`.")
            # Emit one parseable action. A malformed/truncated emission is
            # retried here WITHOUT consuming this action iteration (bounded by
            # REACT_MAX_FORMAT_RETRIES) — a format stumble shouldn't cost a
            # tool turn. Each attempt rebuilds usr_msg so the corrective NOTE
            # appended by the previous attempt is in view.
            action = None
            for fmt_attempt in range(REACT_MAX_FORMAT_RETRIES + 1):
                pre_log_len = len(log)
                usr_msg = user_prefix_str + log_appendage_str + trailer
                # When the turn carries any image, every iter sends a multimodal
                # content array (text first per vLLM's expected ordering, then
                # image_url blocks). Two image sources: the user-supplied image
                # for this turn (image_url) and the most-recent tool-produced
                # image (self._pending_tool_image, e.g. a camera-capture frame).
                # Images must persist across iters: ReAct's parse-failure retry
                # path drops back into iter 2+, and if an image were missing
                # there, the model confabulates against its own prior text.
                # Byte-stable image tokens across iters still let vLLM
                # prefix-cache the text. No-image turns keep the plain-string
                # content shape exactly as before — byte-identical wire format.
                image_blocks: List[Any] = []
                if image_url:
                    image_blocks.append(
                        {'type': 'image_url', 'image_url': {'url': image_url}})
                if self._pending_tool_image is not None:
                    image_blocks.append(
                        {'type': 'image_url',
                         'image_url': {'url': self._pending_tool_image['data_uri']}})
                if image_blocks:
                    user_content: Any = [{'type': 'text', 'text': usr_msg},
                                         *image_blocks]
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
                                            # None -> the model's configured
                                            # temperature. NOT a literal: this
                                            # is the main action-emission call
                                            # and the one that decided every
                                            # discarded run.
                                            temperature=getattr(
                                                self, 'react_temperature', None),
                                            cot_profile='none',
                                            reasoning_effort=self._reasoning_effort,
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
                _append_log('NOTE', unparseable_action_note(truncated))
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

            # Intentional multi-loop continuation: the run ends at a clean
            # boundary, handing the remainder verbatim to a successor
            # concern (autonomous fires) or a fresh agent_concern (user
            # turns) — no synthesizer pass, unlike the reactive max_iters
            # route. Validation failures fall through to the
            # dispatch chain below, which emits the ERROR observation and
            # lets the loop continue.
            if tool == 'yield':
                next_slice = self._resolve_react_value(
                    action.get('next', ''), log).strip()
                if next_slice:
                    text = self._resolve_react_value(action.get('text', ''), log)
                    if not text.strip() and source != self.character_name:
                        # A user is waiting on this turn — never hand them
                        # "(no reply)". State the handoff plainly.
                        text = f"(continuing in the background: {next_slice})"
                    self._react_yield_next = next_slice
                    logger.info(
                        f"[{self.character_name}] ReAct iter {i+1}: yield "
                        f"(next={next_slice[:120]!r}, text={len(text)} chars)")
                    self._clear_status()
                    self._affect.mark_completion()
                    self._affect.exit_loop()
                    return text, log, iters, 'yield'

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
            # CLAUDE.md for the author-facing rule.
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
                    # Citation discipline fires on a structural signal: the
                    # source is a $stepN binding whose step carried
                    # structured sources — not on substring inspection of
                    # the text.
                    cited = None
                    if isinstance(raw_src, str):
                        cited = self._step_sources.get(raw_src.strip())
                    obs = self._run_process_text(src, ins,
                                                 cited_sources=cited)
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
            elif tool == 'justify':
                raw_seq = action.get('turn_seq')
                try:
                    want_seq = (int(raw_seq) if raw_seq not in (None, '')
                                else None)
                except (TypeError, ValueError):
                    want_seq = None
                    logger.warning(
                        f"[{self.character_name}] justify: unusable turn_seq "
                        f"{raw_seq!r}; auditing the most recent reply instead")
                obs = self._run_justify(source, want_seq)
            elif tool == 'agent-say':
                to = str(action.get('to') or '')
                msg_text = self._resolve_react_value(action.get('text', ''), log)
                obs = self._run_agent_say(to, msg_text)
            elif tool == 'display':
                content = self._resolve_react_value(action.get('content', ''), log)
                fmt = (action.get('format') or 'markdown').strip().lower()
                obs = self._run_display(content, fmt)
                # Suppress auto-mirror on the final respond — the agent
                # has already chosen what belongs on the canvas this turn.
                if isinstance(obs, str) and obs.startswith('OK'):
                    did_display = True
            elif tool == 'yield':
                # Reached only when the happy-path branch above declined:
                # wrong context or missing args. Emit the ERROR observation
                # and keep the loop alive.
                if source != self.character_name:
                    obs = ("ERROR: `yield` is only available in autonomous "
                           "runs; finish with `respond` instead.")
                else:
                    obs = ("ERROR: `yield` requires a non-empty `next` field "
                           "— the imperative instruction for the follow-up "
                           "run.")
            elif (canon := self._canonical_tool_name(tool)) is not None:
                obs = self._dispatch_discovered_tool(canon, action, log)
                if self._pending_tool_meta is not None:
                    iters[-1]['tool_meta'] = self._pending_tool_meta
                    # _step_sources drives process_text's citation
                    # requirement, whose premise is that the observation
                    # carries a 'Sources:' bibliography to cite BY. Only a
                    # mediated observation has one: a synthesis over
                    # sources it lists. A direct fetch's "source" is the
                    # document itself — its own URL is not something to
                    # attribute to, and for an internal endpoint
                    # (an InfluxDB query, the factorio bridge) there is no
                    # publication to name. Gating here keeps the trail
                    # record complete for every tool while leaving the
                    # citation path where it belongs.
                    from chat.claims import observation_mediation
                    srcs = [s for m in (self._pending_tool_meta.get('meta') or [])
                            for s in ((m.get('tool_metadata') or {}).get('sources') or [])
                            if isinstance(s, dict)]
                    if srcs and observation_mediation(
                            self._pending_tool_meta.get('tool')) == 'mediated':
                        self._step_sources[binding] = srcs
                    self._pending_tool_meta = None
            else:
                # Tool list shown to the model on bad emission. Built-ins
                # are stable; the discovered set comes from the registry.
                builtin = ["process_text", "recall", "inspect", "security",
                           "justify", "display", "respond"]
                if self._get_external_repo() is not None:
                    builtin.insert(builtin.index("inspect") + 1, "inspect_external")
                if getattr(self, '_peers', None):
                    builtin.insert(builtin.index("display"), "agent-say")
                # Omitted tools must not appear here either — this branch is
                # exactly where an omitted tool lands, and listing it back
                # would invite the retry it was dropped to prevent.
                _omitted = set(getattr(self, '_omitted_tools', None) or [])
                discovered = sorted(set(self._discovered_tools) - _omitted)
                avail = ", ".join(builtin + discovered)
                obs = f"ERROR: unknown tool {tool!r}; available: {avail}"

            # Name the step's tool even when it carried no structured
            # metadata. `justify` derives mediation from tool_meta, so a
            # step absent from it got no mediation verdict at all: a claim
            # quoted against a `process_text` synthesis rendered with no
            # warning, while the same claim quoted against `search-web`
            # got one. That is backwards — process_text is a second model
            # pass over an observation, the likeliest place for drift to
            # enter. Corpus at the time of the fix: 240 claim-refs to
            # steps with metadata, 183 to steps with none (justify 101,
            # process_text 26, recall 17, inspect 6, world-look 9).
            if not iters[-1].get('tool_meta'):
                iters[-1]['tool_meta'] = {'tool': tool, 'meta': []}
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
        active-recall subagent (chat.subagents.recall.recall), which runs its
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
            from chat.subagents.recall import recall as _recall
            threads = self._get_threads(statuses=('active',))
            thread_context = self._render_threads_for_subagent(threads)
            answer = _recall(
                query=str(query),
                memory_dir=self._memory_dir(),
                llm_backend=self.backend,
                trace_dir=self._subagent_traces_dir(),
                thread_context=thread_context,
                reasoning_effort=self._reasoning_effort,
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
        codebase-inspection subagent (chat.subagents.code_subagent.inspect), which
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
            from chat.subagents.code_subagent import inspect as _inspect
            answer = _inspect(
                query=str(query),
                # Scenario may narrow this; see chat_loop's inspect_repo.
                repo_root=Path(getattr(self, '_inspect_root', None)
                               or _REPO_ROOT),
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
                reasoning_effort=self._reasoning_effort,
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
            from chat.subagents.code_subagent import inspect_external as _inspect_external
            answer = _inspect_external(
                query=str(query),
                repo_root=repo,
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
                reasoning_effort=self._reasoning_effort,
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
        network-security investigation subagent (chat.subagents.security.security),
        which runs its own thin ReAct loop with read-only typed primitives
        (discover, scan_services, system_state via nmap + iproute2).
        Returns an OK:/EMPTY:/ERROR: prefixed observation per the ReAct
        tool-observation convention.

        Uses Jill's main backend (self.backend) — see _run_inspect for
        rationale. Per-call traces under security_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: security query was empty"
        try:
            from chat.subagents.security import security as _security, call_deadline
            # One deadline for all security work in this turn. Set on first
            # use (a turn that never asks doesn't start a clock) and reset
            # per turn by _process_user_turn. Without sharing it, the
            # per-call cap bounds nothing — this loop can call the subagent
            # again and again, each time with a fresh budget.
            if getattr(self, '_turn_security_deadline', None) is None:
                self._turn_security_deadline = call_deadline()
            answer = _security(
                query=str(query),
                llm_backend=self.backend,
                trace_dir=self._security_traces_dir(),
                baseline_dir=self._memory_dir() / 'security_baselines',
                deadline=self._turn_security_deadline,
                reasoning_effort=self._reasoning_effort,
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] security subagent raised: {e}")
            return f"ERROR: security subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: security subagent produced no answer'
        return 'OK: ' + text

    def _react_fallback_synthesis(self, log: List[Tuple[str, str]], fail_reason: str = '') -> str:
        """Force a Jill-voice reply when respond never fired.

        When `fail_reason` is set the loop did not merely run out of
        iterations — an LLM call raised (context overflow being the most
        likely cause, since the reasoning-history block stacks prior
        working logs onto every prompt). That path used to return a
        fluent, ordinary-looking reply whose only trace was a log line,
        so a degraded turn was indistinguishable from a good one. The
        marker below makes it visible wherever the reply goes: CLI,
        conversation.txt, and the trace.
        """
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
            text = (result or '').strip() or "(could not synthesize)"
        except Exception as e:
            text = f"(trouble formulating a response: {e})"
        if fail_reason:
            return (f"[degraded reply — a model call failed mid-loop and this "
                    f"was synthesized from a partial working log. "
                    f"{fail_reason[:200]}]\n\n{text}")
        return text
