"""Base class for the tool subagents.

`recall`, `inspect`/`inspect_external`, and `security` each carried a
private copy of the same control loop — emit one JSON action, dispatch a
primitive, bind the observation to $stepN, repeat until `respond` or the
iteration cap. The copies were made by hand from each other (the module
docstrings still say so), which propagated the design forward once but
propagated no improvement backward. This class is the single definition.

Subclasses supply what genuinely differs:

    label            name for logs, answer wording, and trace filename
    max_iters        iteration cap
    system_prompt()  the static system prompt (stable across iterations,
                     so the backend's prefix cache hits)
    primitives()     ordered {tool name: callable(action) -> observation}
    precheck()       optional early guards (empty query, missing root)

Deliberately NOT shared: the primitive implementations. `recall` reads a
gitignored world directory while `inspect` reads a git checkout and uses
`git ls-files` / `git check-ignore` / ripgrep. Pointing recall at the
git-aware versions would make it blind to every memory file it exists to
read — the memory dir matches `scenarios/*/` in .gitignore. Same three
verb names, deliberately different implementations.

Also not shared: the main chat ReAct loop in `react.py`. It carries
$stepN resolution across tools, citation tracking, affect signalling,
image blocks, yield semantics, and an exit_reason contract the concern
layer branches on. Folding it in here is a separate decision that needs
characterization tests on that loop first.
"""

import json
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from chat.react import REACT_ACTION_SCHEMA
from utils.json_utils import repair_json_string, unparseable_action_note
from utils.subagent_trace import write_subagent_trace

logger = logging.getLogger(__name__)


class Subagent:
    """One ReAct-shaped investigation loop over a fixed primitive set."""

    label: str = 'subagent'
    max_iters: int = 12
    max_tokens: int = 8192
    # None -> the model's configured temperature. inspect_external is a
    # subagent, so this sits on the audit path; a literal here is the
    # same silent variable as the one in the ReAct loop.
    temperature: Optional[float] = None

    # Consecutive unparseable emissions tolerated before the loop gives up
    # and reports from the working log. The schema below removes the
    # malformed-shape cause but NOT truncation: a schema-constrained action
    # still gets cut at max_tokens if the model stuffs a huge payload into
    # a field. Observed 2026-08-16 — seven identical retries pasting 300
    # lines of source into `respond.text`, ~6 minutes for zero progress,
    # each one spending an iteration of max_iters.
    max_consecutive_unparseable: int = 3

    # Structured-output constraint on the action emission. All three
    # subagents already prompt for the main loop's shape — {thought, tool,
    # ...args} — so this reuses REACT_ACTION_SCHEMA rather than defining a
    # second one that would drift from it.
    #
    # Added 2026-08-15 after a model swap. The judgement call before that
    # was "not yet": zero unparseable emissions in ~1400 traced iterations
    # since May. That evidence was model-specific. On the first run against
    # Gemma-4-31B the failure mode returned immediately — 9 unparseable
    # emissions in a single 12-iteration call, which burned the whole call.
    # An unbounded denylist of models that happen to emit clean JSON is not
    # a robustness story; the schema makes it structural. _ChatBackend
    # applies it on local engines only and skips it on cloud paths.
    response_schema: Optional[Dict[str, Any]] = REACT_ACTION_SCHEMA

    # Ceiling on the reasoning effort a subagent inherits from its
    # character. A character baseline reaches EVERY emission, and almost
    # none of a subagent's are the thinking: it picks a primitive, picks a
    # path, emits JSON, and only the final `respond` is synthesis. Measured
    # 2026-08-16 on a cloud reasoning model — one turn made 19 inspect
    # calls of ~5 iterations each, so ~100 of ~130 emissions that turn were
    # a subagent deciding which file to read, all at effort=high. Cloud
    # reasoning tokens are billed and not returned, so the whole cost
    # surfaced as latency with nothing in the logs to account for it.
    #
    # A ceiling, not an override: a character already at or below this
    # keeps its own value, and None (nothing sent) stays None. A subclass
    # whose work really is reasoning can raise its own.
    max_reasoning_effort: Optional[str] = 'low'

    _EFFORT_RANK = {'low': 1, 'medium': 2, 'high': 3}

    @classmethod
    def _clamp_effort(cls, effort: Optional[str]) -> Optional[str]:
        ceiling = cls.max_reasoning_effort
        if effort is None or ceiling is None:
            return effort
        rank = cls._EFFORT_RANK
        if rank.get(effort, 0) <= rank.get(ceiling, 0):
            return effort
        return ceiling

    def __init__(self, llm_backend, trace_dir: Path, *,
                 reasoning_effort: Optional[str] = None,
                 deadline: Optional[float] = None):
        self.llm_backend = llm_backend
        self.trace_dir = Path(trace_dir)
        self.reasoning_effort = self._clamp_effort(reasoning_effort)
        if self.reasoning_effort != reasoning_effort:
            logger.info(
                f"{self.label}: reasoning_effort {reasoning_effort!r} -> "
                f"{self.reasoning_effort!r} (subagent ceiling)")
        self.deadline = deadline

    # -- subclass surface ------------------------------------------------

    def system_prompt(self) -> str:
        raise NotImplementedError

    def primitives(self) -> Dict[str, Callable[[Dict[str, Any]], str]]:
        """Ordered map of tool name to handler. Order is user-visible: it
        drives the `available:` list in the unknown-tool observation."""
        raise NotImplementedError

    def precheck(self, query: str) -> Optional[str]:
        """Return an answer string to short-circuit on (bad query, missing
        root), or None to run the loop."""
        return None

    def answer_suffix(self) -> str:
        """Text appended to the answer by the harness, not the model: what a
        subclass's primitives carried verbatim during the run (code_subagent's
        `cite`). Empty by default. Appended on every exit, because material
        the tool copied is evidence whether or not the model got to respond."""
        return ''

    def budget_exhausted_observation(self) -> str:
        """Observation substituted for a primitive once `deadline` passes.
        Only reached by subagents constructed with a deadline."""
        return (f"ERROR: {self.label} has used its whole time budget and no "
                f"further work will run — respond NOW with what the log "
                f"above already shows, and say plainly which part of the "
                f"question you did not get to.")

    # -- the loop --------------------------------------------------------

    def run(self, query: str) -> str:
        early = self.precheck(query)
        if early is not None:
            return early

        sys_prompt = self.system_prompt()
        prims = self.primitives()
        available = ', '.join(list(prims) + ['respond'])
        user_prefix = f"Query: {query.strip()}\n\n## Working log\n"
        log_lines: List[str] = []
        iters: List[Dict[str, Any]] = []

        def _build_user_msg() -> str:
            body = user_prefix + ('\n'.join(log_lines) + '\n' if log_lines
                                  else '')
            return body + '\nEmit next action:\n'

        answer = ''
        exit_reason = 'max_iters'
        consecutive_unparseable = 0
        last_unparseable_truncated = False
        # BUDGET FOR ONE ACTION EMISSION, raised once when the model is cut
        # off mid-emission. Mirrors the main ReAct loop: the corrective note
        # below steers a model that overran a payload field, and cannot steer
        # one whose reasoning consumed the whole budget and left nothing for
        # the action. Retrying that at the same ceiling spends an attempt to
        # hit the same wall — measured 2026-08-28, three consecutive
        # `finish=length` emissions here exhausting the retry budget in ~3
        # minutes for no progress. Reset on any emission that parses.
        budget = self.max_tokens
        for i in range(self.max_iters):
            messages = [
                {'role': 'system', 'content': sys_prompt},
                {'role': 'user', 'content': _build_user_msg()},
            ]
            try:
                raw = self.llm_backend.chat(
                    messages, max_tokens=budget,
                    temperature=self.temperature,
                    reasoning_effort=self.reasoning_effort,
                    response_schema=self.response_schema)
            except Exception as e:
                logger.warning(
                    f"{self.label}: llm call failed at iter {i+1}: {e}")
                answer = f"({self.label}: llm error at iter {i+1}: {e})"
                exit_reason = 'llm_error'
                break

            action = repair_json_string(raw or '')
            action = action if isinstance(action, dict) else None
            iter_rec: Dict[str, Any] = {'raw': raw, 'action': action}
            iters.append(iter_rec)
            if action is None:
                # Steer the retry by WHY the parse failed. "Unparseable"
                # alone reads as bad JSON, so a model cut off mid-payload
                # re-emits the same oversized action verbatim.
                truncated = getattr(self.llm_backend, 'last_finish_reason',
                                    None) in ('length', 'max_tokens')
                consecutive_unparseable += 1
                last_unparseable_truncated = truncated
                if truncated and budget == self.max_tokens:
                    budget *= 2
                    logger.warning(
                        f"{self.label}: cut off at {self.max_tokens} tokens; "
                        f"retrying at {budget}")
                logger.warning(
                    f"{self.label}: unparseable emission at iter {i+1} "
                    f"({consecutive_unparseable}/"
                    f"{self.max_consecutive_unparseable}, "
                    f"finish={getattr(self.llm_backend, 'last_finish_reason', None)})")
                iter_rec['observation'] = '(unparseable)'
                if consecutive_unparseable >= self.max_consecutive_unparseable:
                    exit_reason = 'format_failed'
                    break
                log_lines.append("NOTE: " + unparseable_action_note(truncated))
                continue
            consecutive_unparseable = 0
            budget = self.max_tokens

            tool = action.get('tool')
            if tool is None:
                # Parsed, but not an action. Observed 2026-08-16 on a cloud
                # model with no schema constraint: it emitted its ANSWER as
                # JSON — {"text": ...}, {"creation_occurrences": ...} — and
                # once it decided the tools were broken, {"error": ...}.
                #
                # The old reply was "unknown tool None", which names nothing
                # the model can act on and reads as a tool failure rather
                # than a format slip. That mattered: 160 of 217 observations
                # in one run were this error, and the model concluded from
                # the density that its file reads were returning nothing —
                # they never were, not once. Say what is wrong and how to
                # deliver an answer, so a format slip cannot compound into a
                # belief that the primitives are dead.
                keys = ', '.join(sorted(action)[:6]) or '(none)'
                obs = (f"ERROR: that was valid JSON but not an action — it "
                       f"has no `tool` field (keys: {keys}). Every emission "
                       f"must name one of: {available}. Your primitives are "
                       f"working; this is a formatting problem, not a tool "
                       f"failure. To deliver a finished answer, put it in "
                       f"the `text` field of a respond action: "
                       f'{{"thought": "<terse>", "tool": "respond", '
                       f'"text": "<your answer>"}}')
                iter_rec['observation'] = obs
                log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
                log_lines.append(f"$step{i+1}:")
                log_lines.append(obs)
                log_lines.append('')
                continue

            if tool == 'respond':
                # EMPTY MUST STAY EMPTY. This was `or '(no answer)'`, and
                # that sentinel is a NON-EMPTY string, so the caller's guard
                # (`if not text: return 'EMPTY: ...'`, react.py) could never
                # fire — the parent was handed a cheerful `OK: (no answer)`.
                # Observed 2026-08-23: a subagent read METHOD.md in full
                # across four `read` calls, emitted a respond carrying no
                # `text`, and its parent was told the read had SUCCEEDED and
                # returned nothing. 14 of 29 inspect calls that run; the
                # parent spent two legs re-asking a tool reporting OK, then
                # yielded blocked. A subagent that answers nothing has to say
                # so in the one place that checks, which is emptiness.
                answer = str(action.get('text', '') or '').strip()
                exit_reason = 'respond'
                iter_rec['observation'] = '(respond)'
                break

            binding = f'$step{i+1}'
            if self.deadline is not None and time.monotonic() >= self.deadline:
                # Refuse the primitive, not the answer: the model still gets
                # a turn to report what it has, and the iteration cap ends
                # the loop if it asks for more anyway. Not silent — a
                # truncated survey the caller believes is complete is worse
                # than a short one that says so.
                logger.warning(
                    f"{self.label}: time budget spent at iter {i+1}; "
                    f"refusing further work")
                obs = self.budget_exhausted_observation()
            else:
                handler = prims.get(tool)
                if handler is None:
                    obs = (f"ERROR: unknown tool {tool!r}; "
                           f"available: {available}")
                else:
                    obs = handler(action)

            iter_rec['observation'] = obs
            log_lines.append(f"ACTION {i+1}: {json.dumps(action)}")
            log_lines.append(f"{binding}:")
            log_lines.append(obs)
            log_lines.append('')

    # Per-observation and total caps on salvaged evidence. Sized so grep hits
    # survive whole — they are short, one file:line per row, and they are what
    # a caller most needs — while a long file read is trimmed. The caller can
    # always read a file again; it cannot re-run a search it never saw.
        if exit_reason == 'max_iters' and not answer:
            answer = (f"({self.label}: hit max iterations without responding; "
                      f"consider narrowing the query)"
                      + self._salvage(iters))
        elif exit_reason == 'format_failed' and not answer:
            cause = ("the answer was too large to emit — ask for a summary or "
                     "a narrower line range rather than raw content"
                     if last_unparseable_truncated
                     else "it could not emit valid JSON")
            answer = (f"({self.label}: gave up after "
                      f"{self.max_consecutive_unparseable} consecutive "
                      f"unparseable emissions; {cause}.)"
                      + self._salvage(iters))

        suffix = self.answer_suffix()
        if suffix:
            answer = (answer or '') + suffix
        write_subagent_trace(self.trace_dir, self.label, query, iters, answer,
                             exit_reason)
        return answer

    _SALVAGE_PER_OBS = 600
    _SALVAGE_TOTAL = 5000

    def _salvage(self, iters: List[Dict[str, Any]]) -> str:
        """Return what the loop actually found, for the paths that end without
        an answer.

        WHY THIS EXISTS. A subagent that runs out of iterations used to return
        only a diagnostic and discard its working log. Observed 2026-08-23 on
        the first real engagement: an `inspect_external` call searching for a
        sentiment module hit the grep result

            repositories/chat.py:36: from app.services.sentiment import ...

        at iteration 3 of 12, kept working, exhausted its budget, and reported
        "hit max iterations without responding". The parent — hearing nothing
        from that call and nothing contradicting it from the others — published
        a finding that no sentiment scorer existed. One did.

        The search was not the failure. The failure was that a loop which
        found the answer threw it away on the way out, which is the same shape
        as procedure dying in a capped observation and a report dying in an
        unsaved canvas render.

        Chronological, not most-recent-first: the useful hit was iteration 3 of
        12, so recency is the wrong heuristic for which evidence mattered.
        """
        rows: List[str] = []
        used = 0
        for n, it in enumerate(iters, 1):
            obs = str(it.get('observation') or '').strip()
            if not obs or obs.startswith('ERROR:'):
                continue
            if len(obs) > self._SALVAGE_PER_OBS:
                obs = obs[:self._SALVAGE_PER_OBS].rstrip() + ' …[trimmed]'
            if used + len(obs) > self._SALVAGE_TOTAL:
                rows.append(f"…[{len(iters) - n + 1} further observations "
                            f"omitted for length]")
                break
            rows.append(f"  step {n}: {obs}")
            used += len(obs)
        if not rows:
            return ''
        return ("\n\nPARTIAL — what this call did find before it ran out, "
                "unsummarised:\n" + "\n".join(rows))

