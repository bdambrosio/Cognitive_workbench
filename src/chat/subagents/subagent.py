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
from utils.json_utils import repair_json_string
from utils.subagent_trace import write_subagent_trace

logger = logging.getLogger(__name__)


class Subagent:
    """One ReAct-shaped investigation loop over a fixed primitive set."""

    label: str = 'subagent'
    max_iters: int = 12
    max_tokens: int = 4096
    temperature: float = 0.2

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

    def __init__(self, llm_backend, trace_dir: Path, *,
                 reasoning_effort: Optional[str] = None,
                 deadline: Optional[float] = None):
        self.llm_backend = llm_backend
        self.trace_dir = Path(trace_dir)
        self.reasoning_effort = reasoning_effort
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
        for i in range(self.max_iters):
            messages = [
                {'role': 'system', 'content': sys_prompt},
                {'role': 'user', 'content': _build_user_msg()},
            ]
            try:
                raw = self.llm_backend.chat(
                    messages, max_tokens=self.max_tokens,
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
                log_lines.append("NOTE: previous output was unparseable; "
                                 "emit ONE JSON action now.")
                iter_rec['observation'] = '(unparseable)'
                continue

            tool = action.get('tool')
            if tool == 'respond':
                answer = (str(action.get('text', '') or '').strip()
                          or '(no answer)')
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

        if exit_reason == 'max_iters' and not answer:
            answer = (f"({self.label}: hit max iterations without responding; "
                      f"consider narrowing the query)")

        write_subagent_trace(self.trace_dir, self.label, query, iters, answer,
                             exit_reason)
        return answer
