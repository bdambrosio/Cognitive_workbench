"""Prompt assembly — system prompt, ReAct prompt variants, orientation
summary, reasoning-history rendering. PromptsMixin for ChatLoop, moved
verbatim from chat_loop.py in the 2026-06 mixin refactor."""

from __future__ import annotations

import logging
import os
import sys
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from chat.concerns import _AGENT_CONCERN_FIRE_THRESHOLD  # noqa: E402
from chat.memories import _MEMORY_CATEGORIES  # noqa: E402

logger = logging.getLogger('chat_loop')

# Reasoning history (awareness feed): per-turn ReAct trace persisted as a
# Note in the `reasoning_history` collection. Last N entries surface in the
# user-message prefix between conversation history and current input. Most
# recent _REASONING_HISTORY_FULL render in full; older ones render as a
# compressed action-sequence digest.
_REASONING_HISTORY_COLLECTION_NAME = "reasoning_history"

# NOT APPLIED — deliberately. reasoning_trace.jsonl is append-only and
# pruning it would (a) discard the analytical history the provenance and
# disposition work reads, and (b) break turn_seq, which chat_loop seeds by
# counting lines in this file: after a prune the counter restarts mid-
# history and the sidecar joins (memories.jsonl / claims.jsonl
# source_turn_seq) silently point at the wrong turns. Per-record growth is
# bounded by _REASONING_HISTORY_OBS_CAP instead. Archiving old records is a
# manual operation; if it is ever automated, seed turn_seq from the last
# record's turn_seq rather than the line count first.
_REASONING_HISTORY_RING_SIZE = 50    # on-disk cap (unused; see above)

_REASONING_HISTORY_RECENT = 6        # surfaced in the prompt

# Per-turn ceiling on rendered conversation history.
#
# history_limit bounds the prompt by TURN COUNT, which ignores how big a
# turn is: a session with one large paste reached the context ceiling well
# before its twentieth turn (2026-08-15, a 400 at 57,145 prompt + 8,192
# reserved output against 65,336 — over by one token, whole turn lost).
#
# A global token budget with oldest-first eviction would be worse: a single
# large recent turn would evict everything behind it, leaving one turn of
# context. Capping the CONTRIBUTOR instead makes the worst case
# history_limit * cap, and no turn can crowd out its neighbours.
#
# Measured over 3,784 real turns: median 95 chars, p95 1,456, p99 2,718,
# max 17,630. At 4,000 this touches 14 turns (0.4%) and preserves 94% of
# all text — invisible in the common case, bounded in the pathological one.
# Elided text stays reachable: conversation.txt is on disk and `recall`
# greps it.
_HISTORY_TURN_MAX_CHARS = 4000


def _cap_turn(text: str) -> str:
    """Bound one rendered history turn. Keeps the head — the opening of a
    turn carries its intent; a truncated tail is what the agent can ask
    `recall` for."""
    if len(text) <= _HISTORY_TURN_MAX_CHARS:
        return text
    return (text[:_HISTORY_TURN_MAX_CHARS]
            + f"\n…[turn truncated at {_HISTORY_TURN_MAX_CHARS} chars of "
              f"{len(text)}; use recall to retrieve the rest]")

_REASONING_HISTORY_FULL = 3          # of those, how many in full vs compressed

_REASONING_HISTORY_OBS_CAP = 1000    # per-iter observation cap in stored trace

# When the cap above elides part of an observation, the untruncated text
# is kept under record['observations_full'] for claim attribution only —
# the attributor cannot otherwise distinguish "no evidence for this
# claim" from "the evidence was cut out of my view", and silently grades
# read-from-source facts as model_prior (live miss: turn 2349, where the
# cut fell mid-sentence and split one paper sentence across both
# groundings). Bounded by what the ReAct loop actually hands the model,
# so the trace never stores more than the model saw, and the field is
# omitted entirely on turns where nothing was truncated.
_ATTRIBUTION_OBS_CAP = 8000


class PromptsMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # ------------------------------------------------------------------
    # Orientation pass (character_evaluator)
    # ------------------------------------------------------------------

    def _orientation_summary(self, source: str, text: str) -> str:
        if not self.orientation_enabled:
            return ''
        try:
            from character_evaluator import evaluate, build_orientation_summary
            recent = self._build_dialog(source, limit=6)
            recent_str = '\n'.join(f"{t['source']}: {t['text']}" for t in recent[:-1])
            event = {
                'event_type': 'user_text',
                'source': source,
                'content': text,
                'disposition': 'inform',
                'event_id': uuid.uuid4().hex[:12],
                'timestamp': datetime.now(timezone.utc).isoformat(),
            }
            # Pass current active concerns so the evaluator can do real
            # relevance assessment. Concern text serves double duty as id
            # (truncated) and description — the evaluator emits matches as
            # `<id>:<level>` and the rendered orientation surfaces those
            # ids verbatim, so a meaningful slug lets Jill see *which*
            # concern the input activates.
            char_concerns: List[Dict[str, str]] = []
            for _nid, note, _a in self._iter_active_agent_concerns():
                ctext = str((note.get('properties') or {}).get('content', '') or '').strip()
                if not ctext:
                    continue
                slug = ctext[:60].replace('\n', ' ').strip()
                char_concerns.append({'id': slug, 'description': ctext})
            companion = self._companion_for_turn(source)[1]
            assessment = evaluate(
                event=event,
                character_concerns=char_concerns,
                # Top user_concerns mapped to the evaluator's schema so the
                # attend_to_user_concerns dimension assesses against the
                # real user model (was hard-coded [] — the evaluator saw
                # "(none tracked)" on every event in chat mode).
                user_concerns=self._user_concerns_for_evaluator(),
                goals_compact=[],
                recent_context=recent_str,
                activity_state='chat-only (no autonomous activity)',
                llm_generate=self._make_llm_callable(
                    'triage', reasoning_effort=self._reasoning_effort),
                narrator_persona=self.persona,
                narrator_self_model=self.self_model,
                companion_state=companion,
            )
            return build_orientation_summary(assessment, event_content=text)
        except Exception as e:
            logger.warning(f'[{self.character_name}] orientation eval failed: {e}')
            return ''

    @staticmethod
    def _render_user_concerns_block(
            source: str,
            user_concerns: List[Tuple[str, str, float, Dict[str, Any]]]) -> str:
        """Render the surfaced user_concerns: ranked headline plus the
        concern's context (evidence + what the user appears to want) as an
        indented continuation line. The context is what makes a stale
        headline usable — display-capped so five concerns stay a modest
        prompt block."""
        uc_lines: List[str] = []
        for _nid, text, strength, props in user_concerns:
            uc_lines.append(f"- [{strength:.2f}] {text}")
            ctx = str((props or {}).get('context', '') or '').strip()
            if ctx:
                uc_lines.append(f"    {ctx[:200]}")
        return (
            f"## What {source} has been tracking (user_concerns, ranked by strength)\n"
            "Topics user has surfaced or returned to recently. Strength "
            "decays each turn unless user touches the topic again. The "
            "indented line under each is its context: the evidence and "
            "what the user appears to want regarding it — let that shape "
            "the response. Use to inform responses; do not act on these "
            "autonomously.\n\n"
            + "\n".join(uc_lines)
        )

    @staticmethod
    def _render_pending_fires_block(
            pending: List[Dict[str, Any]]) -> str:
        """Render the fire digest: autonomous acts awaiting the user's
        reaction, surfaced once each (chat_loop marks them) so the user
        gets a reaction opportunity inside the outcome-judgment window
        (docs/fire-outcome-capture.md §4)."""
        pf_lines: List[str] = []
        for rec in pending:
            turns = int(rec.get('user_turns_since', 0) or 0)
            ago = f"{turns} user turn{'s' if turns != 1 else ''} ago"
            concern = str(rec.get('concern_text', '') or '').strip()
            digest = str(rec.get('reply_digest', '') or '').strip()
            pf_lines.append(f"- [{ago}] {concern[:120]}\n    I said/did: {digest}")
        return (
            "## My recent autonomous acts the user may not have seen\n"
            "Work I did on my own concerns since the user last engaged "
            "with it. Where it fits naturally, mention one briefly (a "
            "clause is enough) so the user can react; the user's current "
            "input always takes priority, and if they are already "
            "reacting to one of these, just respond to that naturally — "
            "no need to re-announce it.\n\n"
            + "\n".join(pf_lines)
        )

    @staticmethod
    def _render_wip_review_block(
            inventory: List[Tuple[str, str, float, str]]) -> str:
        """Render the WIP inventory a wip_reviewer fire reviews: one
        entry per active concern carrying work-in-progress, activation
        descending (chat_loop collects it via _collect_concern_wip)."""
        lines: List[str] = []
        for _nid, text, activation, wip in inventory:
            lines.append(f"- [{activation:.2f}] {text[:120]}")
            for wl in wip.splitlines():
                lines.append(f"    {wl}")
        return (
            "## Work-in-progress across my active concerns (for this review)\n"
            "One entry per concern carrying WIP. Much of it is procedural "
            "— notes on how to do a job well; those are not reviewable "
            "items. The reviewable items are half-done arcs: a pending "
            "idea, an unfollowed finding, work stalled or blocked on "
            "something. An entry ending with a 'NEXT: ' line is naming "
            "its own most promising next step — weigh those first.\n\n"
            + "\n".join(lines)
        )

    def _build_system_prompt(self, source: str, orientation: str,
                             recall: Optional[List[Tuple[str, str, str, Optional[str], Optional[str]]]] = None,
                             agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                             user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None) -> str:
        """Build the persona/state portion of the system prompt — shared base
        for ReAct mode. The prose-only directive that used to live here was
        moved out: ReAct supplies its own JSON-emit directive in
        _build_react_system_prompt. There is no non-ReAct chat path now."""
        parts: List[str] = []
        parts.append(f"You are {self.character_name}, speaking in first person.")
        if self.persona:
            parts.append("## Persona (from character config)\n" + self.persona)
        if self.self_model:
            parts.append("## Self-model (from character config; what I am, not who)\n" + self.self_model)
        if self.capabilities:
            parts.append("## Capabilities (from character config; chat-only mode)\n" + self.capabilities)
        # Substrate provenance: computed once at session start
        # (chat_loop._compute_substrate_line). Absent when git is
        # unavailable.
        substrate = getattr(self, '_substrate_line', '') or ''
        if substrate:
            parts.append(
                "## Substrate (harness provenance, session start)\n"
                "What I am actually running — the working tree, not just "
                "pushed history. This covers harness code only; my model "
                "weights are separate (backend named below when known). "
                "If commits landed since my last session, my beliefs "
                "about my own architecture may be stale — verify with "
                "inspect or exec-script before asserting them.\n\n"
                + substrate)
        # Which body she actually has, probed at session start
        # (chat_loop._compute_embodiment_line). Persona text can only say
        # "when X is up…"; this says which is up. Self-model is left alone —
        # an embodiment is environment, not substrate.
        embodiment = getattr(self, '_embodiment_line', '') or ''
        if embodiment:
            parts.append(
                "## Embodiment (probed at session start)\n"
                "Which world I am actually in right now. Measured, not "
                "configured — if the persona describes an embodiment that "
                "is not listed as LIVE below, I do not have that body this "
                "session. A surface can come up or go down mid-session, so "
                "a tool's own error is the authority if they disagree.\n\n"
                + embodiment)
        # Position, unlike the probe above, is re-measured every turn —
        # it is the one embodiment fact that changes while the agent
        # stands still, and the only current state the prompt used to
        # omit. See chat_loop._world_position_line for what that cost.
        position = self._world_position_line()
        if position:
            parts.append(
                "## Where I am in the shared world (measured this turn)\n"
                "Read from the world just now. This supersedes any position "
                "I remember from an earlier turn or an earlier session: the "
                "world does not persist across restarts, so a remembered "
                "position is a record of where I was, never evidence of "
                "where I am.\n\n"
                + position)
        # When an external repo is bound for the session, append a single
        # capabilities line so the persona-level account reflects what the
        # ReAct surface actually exposes. Self-model is intentionally left
        # alone — external code is environment, not substrate.
        external_repo = self._get_external_repo()
        if external_repo is not None:
            parts.append(
                f"## External repo (bound for this session)\n"
                f"She can also navigate the project repo at `{external_repo}` "
                f"via the inspect_external tool — same list/read/grep "
                f"primitives as inspect, different geofence. This is reading "
                f"an external codebase as documentation, not introspection.")
        if self.setting:
            parts.append("## Setting (from character config)\n" + self.setting)
        # Not keyed on `source` directly: an autonomous fire's source is
        # the character's own name, and looking that up returned nothing
        # — see chat_loop._companion_for_turn for what that cost.
        companion_entity, companion = self._companion_for_turn(source)
        if companion:
            parts.append(
                f"## Companion model of {companion_entity} (rolling LLM reflection; fair-witness texture, not a brief to flatter)\n"
                f"{companion}"
            )
        # User concerns sit adjacent to companion — they're a structured
        # part of the user model. Decay each turn; bumped on similarity
        # match with input. Surface what user has been tracking, ranked
        # by current strength.
        if user_concerns:
            parts.append(self._render_user_concerns_block(source, user_concerns))
        # Agent concerns sit before memories: directives to advance, not
        # background context. Each item renders with current activation
        # so Jill can see how close to firing each one is. Seeds are
        # constitutional (sources for derived concerns), distinguished
        # by tag. instruction-bearing concerns fire when activation
        # crosses threshold.
        if agent_concerns:
            ac_lines: List[str] = []
            for _nid, text, activation, props in agent_concerns:
                tags = []
                if props.get('seed'):
                    tags.append('seed')
                if props.get('successor_of'):
                    tags.append(f"successor d{props.get('successor_depth', 1)}")
                tag_str = f", {','.join(tags)}" if tags else ''
                ac_lines.append(f"- [{activation:.2f}{tag_str}] {text}")
                instr = (props.get('instruction') or '').strip()
                rhythm = props.get('rhythm_hours')
                if instr and rhythm:
                    ac_lines.append(
                        f"    fires every ~{rhythm}h when activation crosses "
                        f"{_AGENT_CONCERN_FIRE_THRESHOLD:.2f}: {instr[:120]}")
                elif not instr:
                    ac_lines.append(
                        "    standing concern, no instruction (won't fire)")
            # Whether firing actually happens is a launcher flag she
            # otherwise cannot see (_autonomy_enabled, chat_loop.py).
            # Without this she can describe the mechanism but not answer
            # "will you actually go do that?" about her own concerns.
            if getattr(self, '_autonomy_enabled', False):
                autonomy_line = (
                    "Autonomous firing is ON this session: when one of these "
                    "crosses threshold it runs on its own, with no user turn "
                    "to prompt it.")
            else:
                autonomy_line = (
                    "Autonomous firing is OFF this session (the launcher's "
                    "--autonomy flag is not set). Activation still grows and "
                    "is shown here, but nothing fires by itself — these shape "
                    "what I attend to on turns I am given, and I say so "
                    "plainly if asked whether I will act on one unprompted.")
            parts.append(
                f"## My active concerns (agent_concerns, ranked by activation)\n"
                "Pressure-driven: activation grows over wall-clock time at "
                "each concern's rhythm; firing decrements it. Concerns "
                "without an instruction don't fire — they shape what I "
                "attend to without driving action.\n"
                f"{autonomy_line}\n\n"
                + "\n".join(ac_lines)
            )
        # Fire digest: pending autonomous fires being surfaced this turn
        # (set at user-turn entry in _process_user_turn; empty on
        # autonomous turns). Absent when empty — prompt stability.
        pending_fires = getattr(self, '_pending_fire_digest', None)
        if pending_fires:
            parts.append(self._render_pending_fires_block(pending_fires))
        # WIP inventory: present only on a wip_reviewer concern's own
        # fire (set in _process_user_turn). Absent otherwise.
        wip_inventory = getattr(self, '_wip_review_inventory', None)
        if wip_inventory:
            parts.append(self._render_wip_review_block(wip_inventory))
        # Threads: activity-level anchors. Computed once per turn in
        # _process_user_turn_inner via _compute_thread_activation; the
        # rendered block names the primary active thread plus any
        # secondary threads carrying meaningful weight. Activations
        # themselves are not surfaced as numbers — prominence in the
        # prompt encodes the distribution.
        threads_block = self._render_active_threads_block(
            self._current_thread_activation)
        if threads_block:
            parts.append(threads_block)
        if recall:
            # Episodic specifics retrieved from prior conversations. Distinct
            # from the rolling Companion summary: these are durable items
            # that should not decay with style. Rendered grouped by
            # category so the model can treat each group on its own terms.
            # Negative-polarity items (explicit rejections / dispreferences)
            # are prefixed `[avoid] ` so the model treats them as boundaries
            # rather than positive facts.
            grouped: Dict[str, List[str]] = {c: [] for c in _MEMORY_CATEGORIES}
            for text, cat, pol, note_id, created_at in recall:
                if cat not in grouped:
                    cat = 'fact'
                marker = '[avoid] ' if pol == 'negative' else ''
                # Provenance tag: memory id + write date, so the model can
                # cite a memory ("per what you told me in May") and age-
                # discount stale ones, and so claim attribution can point
                # at the exact note. Untagged when the source note is gone.
                tag = ''
                if note_id:
                    day = str(created_at or '')[:10]
                    tag = f"[{note_id} · {day}] " if day else f"[{note_id}] "
                grouped[cat].append(f"{tag}{marker}{text}")

            body_lines: List[str] = []
            for cat in self._CATEGORY_RENDER_ORDER:
                items = grouped.get(cat) or []
                if not items:
                    continue
                if body_lines:
                    body_lines.append('')  # blank line between groups
                body_lines.append(f"{self._CATEGORY_HEADERS[cat]}:")
                for t in items:
                    body_lines.append(f"- {t}")

            if body_lines:
                parts.append(
                    "## Recalled memories (semantic match against everything "
                    "I have remembered — across all my conversations, not "
                    f"only the one with {source})\n"
                    "Retrieval is by meaning, so a memory here may have been "
                    "formed while talking with someone else; do not assume "
                    f"{source} said it. Preferences shape how to respond; "
                    "commitments are open agreements that may need follow-up; "
                    "facts are background specifics. Items marked `[avoid]` "
                    "are explicit rejections — do not act on them as if "
                    "they were positive preferences. The `[Note_N · date]` "
                    "tag is each memory's id and write date — use the date "
                    "to judge staleness; never echo the raw Note_N id in "
                    "conversation.\n\n"
                    + "\n".join(body_lines)
                )
        # Same counterpart as the companion block above — keyed on
        # `source` this was empty on every autonomous fire.
        disc = self._discourse_state.get(
            self._counterpart_for_turn(source), '').strip()
        if disc:
            parts.append(
                "## Shared premises and standing decisions (from periodic reflection)\n"
                "These are the operating premises and decisions both parties "
                "have accepted across this conversation. Treat as load-bearing — "
                "disagreeing with one reopens prior discussion rather than "
                "introducing a new topic.\n\n"
                + disc)
        if orientation:
            parts.append(orientation)
        return "\n\n".join(parts)

    def _build_react_system_prompt(self, source: str, orientation: str,
                                   now_str: str,
                                   recall: Optional[List[Tuple[str, str, str, Optional[str], Optional[str]]]] = None,
                                   agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                                   user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None) -> str:
        base = self._build_system_prompt(
            source, orientation, recall=recall,
            agent_concerns=agent_concerns, user_concerns=user_concerns)
        search_omitted = 'search' in (self._omitted_tools or [])
        if search_omitted:
            fresh_info_guidance = (
                "You do NOT know: current weather, recent news, current "
                "prices, or anything requiring fresh information. You have "
                "no live-data tool available this session; if asked for "
                "fresh information, say so plainly rather than guessing.\n"
            )
        else:
            fresh_info_guidance = (
                "You do NOT know: current weather, recent news, current prices, "
                "or anything requiring fresh information. For time-sensitive or "
                "fact-specific questions, your first action is `search-web`.\n"
            )
        react = (
            f"\n\n## Now (system clock)\n{now_str}\n"
            "\n"
            "## ReAct Tool Loop — READ FIRST\n"
            "Each emission is ONE JSON action object — nothing else. Prose "
            "around the JSON is discarded and the loop will retry. Output "
            "begins with `{` and ends with `}`.\n"
            "\n"
            "Every emission MUST include a `thought` field: ONE TERSE "
            "SENTENCE supporting your action choice. Not narration of the "
            "user's intent, not a summary of the conversation, not "
            "throwaway filler — the actual one-line reason for picking "
            "THIS action over the alternatives. The thought is preserved "
            "verbatim into your future-turn awareness feed.\n"
            "\n"
            + fresh_info_guidance
            + "\n"
            + self._build_react_tool_catalog()
            + "\n"
            "## Observation format\n"
            "Each tool observation in the working log starts with one of three tags:\n"
            "  `OK: <content>`     — tool succeeded; content follows\n"
            "  `EMPTY: <reason>`   — tool ran cleanly but produced no usable result\n"
            "  `ERROR: <reason>`   — tool failed (unavailable / raised / malformed args)\n"
            "Treat ERROR as a hard signal: the tool is currently broken — do NOT retry the "
            "same call. Treat EMPTY as a soft signal: reformulating the call (different query, "
            "different URL) may help, but do not loop blindly. The tag is informational for your "
            "reading of the log; it is automatically stripped when `$stepN` is substituted into "
            "downstream actions or into `respond.text`, so you do not need to strip it yourself.\n"
            "\n"
            "Three sources of content in this loop, each attributable when asked about provenance: "
            "(a) Substrate output (your own): text you write inline within an action — `thought`, "
            "`process_text` `instruction`, or inline `respond.text` — is produced by you in this "
            "iteration. You can report THAT you produced it, not WHY a particular phrasing or "
            "formulation arose. (b) Bound inputs: each action's result auto-binds to `$step1, "
            "$step2, …` (per-turn scope) — observable outputs of prior tool calls. (c) Prompt-given "
            "inputs: `## Conversation history`, `## Active concerns`, `## Recalled memories`, "
            "`## Recent reasoning` are observable input from external state. Reason over them "
            "directly in `respond` rather than echoing a section name into `process_text`'s "
            "`source`; `source` resolves only literal inline text or a `$stepN` binding, never a "
            "section header.\n"
            "\n"
            + ("" if search_omitted else
               "Worked example. User: 'what's the weather in Berkeley tomorrow?'\n"
               "  Iter 1: `{\"thought\": \"Need fresh weather data — search first.\", \"tool\": \"search\", \"query\": \"Berkeley CA weather forecast tomorrow\"}` → $step1\n"
               "  Iter 2: `{\"thought\": \"Search synthesis is decent; render in my voice with source.\", \"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"answer the user in your voice in 1-2 sentences, citing the source domain\"}` → $step2\n"
               "  Iter 3: `{\"thought\": \"Processed answer is ready — send it.\", \"tool\": \"respond\", \"text\": \"$step2\"}` → loop exits.\n"
               "\n"
               "Worked example with display. User: 'show me the S&P 500 daily closes for the last week.'\n"
               "  Iter 1: `{\"thought\": \"Need fresh price data.\", \"tool\": \"search\", \"query\": \"S&P 500 daily close last 7 trading days\"}` → $step1\n"
               "  Iter 2: `{\"thought\": \"Format extracted prices as a markdown table for the canvas.\", \"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"extract the daily closes as a markdown table with Date and Close columns; nothing else\"}` → $step2\n"
               "  Iter 3: `{\"thought\": \"Push table to canvas — it's tabular, not prose.\", \"tool\": \"display\", \"content\": \"$step2\", \"format\": \"markdown\"}` → $step3 (non-terminal)\n"
               "  Iter 4: `{\"thought\": \"Acknowledge in the chat with a brief note pointing at the canvas.\", \"tool\": \"respond\", \"text\": \"Posted the last week's closes on screen — let me know if you want a longer window or a chart.\"}` → loop exits.\n"
               "\n")
            + "Output ONLY one JSON object. No prose, no apology, no explanation."
        )
        return base + react

    def _firing_concern_entity(self) -> str:
        """Counterpart of the concern currently firing, for autonomous
        turns. 'User' when there is no concern id, the note has gone, or
        it names no entity — the value this replaced was that constant."""
        cid = (getattr(self, '_current_turn', None) or {}).get(
            'autonomous_concern_id')
        if not cid:
            return 'User'
        try:
            note = self.resource_manager.get_resource(cid) or {}
            entity = str((note.get('properties') or {}).get('entity')
                         or '').strip()
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] firing-concern entity lookup "
                f"failed for {cid}: {e}; falling back to 'User'")
            return 'User'
        return entity or 'User'

    def _build_react_user_prefix(self, source: str, user_text: str) -> str:
        """Build the user-message PREFIX for the ReAct loop. Constructed once
        per loop; the working-log entries and the "Emit next action:" trailer
        are appended by the caller (store-and-append: the prefix is byte-stable
        across iterations so KV cache hits, only the log grows). Ends with
        '## Working log\\n' so the first appended entry sits below the header."""
        parts: List[str] = []
        # On autonomous turns `source` is the character itself, so it names
        # no counterpart. Use the firing concern's entity: work raised while
        # talking with Jack should resume against the Jack conversation, not
        # the User one. This line hard-coded 'User' until 2026-08-16, which
        # was right while there was only ever one counterpart.
        history_entity = self._counterpart_for_turn(source)
        history = self.store.get_recent_turns(history_entity, limit=self.history_limit, scope='all')
        if history and history[-1].get('direction') == 'in' and str(history[-1].get('text', '')) == user_text:
            history = history[:-1]
        if history:
            parts.append("## Conversation history (verbatim session turns)")
            for t in history:
                who = history_entity if t.get('direction') == 'in' else self.character_name
                parts.append(f"{who}: {_cap_turn(str(t.get('text', '')))}")
            parts.append("")
        # Awareness feed: prior ReAct traces (Jill's own thinking from
        # recent turns) sit between conversation history and current
        # input. Read once per loop entry — same store-and-append
        # discipline as conversation history: byte-stable across
        # iterations within the loop, fresh on the next loop.
        reasoning_block = self._get_reasoning_history_block(history_entity)
        if reasoning_block:
            parts.append(reasoning_block)
            parts.append("")
        # Name the source. With co-resident agents "user input" was a
        # false label on every peer turn — the one field naming who is
        # actually speaking.
        if source == self.character_name:
            parts.append("## Current input (my own concern firing)")
        else:
            parts.append(f"## Current input (from {source})")
        parts.append(user_text)
        parts.append("")
        parts.append("## Working log (this loop's actions and observations)")
        return "\n".join(parts) + "\n"

    @staticmethod
    def _trace_origin(rec: Dict[str, Any]) -> str:
        """Who this trace's turn came from, for the record's header.

        The record has carried `source` and `autonomous` all along; until
        2026-08-16 neither was rendered, so a Jack turn and a concern fire
        both appeared under `USER INPUT:`. With several agents co-resident
        that is not a cosmetic gap — it is the prompt asserting a single
        relationship where there are three."""
        if rec.get('autonomous'):
            return 'autonomous concern fire'
        return f"turn from {str(rec.get('source') or '?')}"

    def _render_trace_record(self, rec: Dict[str, Any], full: bool) -> str:
        """Render a single per-turn record from reasoning_trace.jsonl
        for inclusion in the user-prompt's ## Recent reasoning block.
        full=True renders all per-turn-unique fields (used for recent
        traces and all traces in benchmark_mode); full=False renders a
        one-line digest (used for older traces, and for traces belonging
        to a different interlocutor — see _get_reasoning_history_block)."""
        seq = int(rec.get('turn_seq', 0))
        origin = self._trace_origin(rec)
        if not full:
            user_short = (rec.get('user_input') or '')[:60].replace('\n', ' ')
            resp_short = (rec.get('raw_response') or '').replace('\n', ' ')[:200]
            return (f"### trace #{seq} ({origin}): "
                    f"'{user_short}' → \"{resp_short}\"")
        parts: List[str] = [f"### trace #{seq} — {origin}"]
        ts = rec.get('ts')
        if ts:
            parts.append(f"TIMESTAMP: {ts}")
        if rec.get('orientation'):
            parts.append(f"ORIENTATION:\n{rec['orientation']}")
        if rec.get('active_concerns'):
            parts.append("ACTIVE CONCERNS:")
            for c in rec['active_concerns']:
                parts.append(f"  - {c}")
        if rec.get('recall_hits'):
            parts.append("RECALL HITS:")
            for r in rec['recall_hits']:
                parts.append(f"  - {r}")
        prefix_refs = rec.get('prefix_trace_refs') or []
        if prefix_refs:
            parts.append(
                f"PRIOR TRACES VISIBLE TO ME AT THIS TURN: {prefix_refs} "
                "(by turn_seq; resolve via reasoning_trace.jsonl line N)")
        if rec.get('autonomous'):
            parts.append(f"CONCERN INSTRUCTION: {rec.get('user_input', '')}")
        else:
            parts.append(f"INPUT FROM {str(rec.get('source') or '?')}: "
                         f"{rec.get('user_input', '')}")
        if rec.get('image_ref'):
            parts.append(f"IMAGE ATTACHED: {rec['image_ref']}")
        if rec.get('working_log'):
            parts.append(f"WORKING LOG:\n{rec['working_log']}")
        if rec.get('raw_response'):
            parts.append(f"RAW RESPONSE: {rec['raw_response']}")
        return "\n".join(parts)

    def _get_reasoning_history_block(self, thread_entity: str = 'User') -> str:
        """Build the ## Recent reasoning block for the user-message prefix.
        Reads structured per-turn records from reasoning_trace.jsonl and
        renders per-turn-unique content (orientation, concerns at the
        time, recall, user input, working log, raw response, plus
        prefix_trace_refs as a faithful awareness-window record).

        Side effect: sets self._last_inject_trace_seqs to the list of
        turn_seqs that were rendered, used by the next
        _write_reasoning_history call to populate prefix_trace_refs.

        In benchmark_mode: surfaces ALL session records in full.
        In normal chat mode: last _REASONING_HISTORY_RECENT records,
        most recent _REASONING_HISTORY_FULL in full, older as one-line
        digest (token budget).

        thread_entity is the interlocutor whose conversation this turn
        belongs to. The trace file is a single stream across ALL of them,
        so with co-resident agents a turn answering Jack used to carry
        User turns at full weight. Records from another interlocutor now
        render as a digest whatever their recency: enough to know that
        exchange happened, not enough to dominate this one. Own
        autonomous fires count as this thread — they are continuation of
        my own work, not a different relationship."""
        self._last_inject_trace_seqs = []
        records = self._load_reasoning_records()
        if not records:
            return ''
        bench = bool((self.config.get('chat') or {}).get('benchmark_mode'))
        selected = records if bench else records[-_REASONING_HISTORY_RECENT:]
        n = len(selected)

        def _same_thread(rec: Dict[str, Any]) -> bool:
            return (bool(rec.get('autonomous'))
                    or str(rec.get('source') or '') == thread_entity)

        # Budget the full renderings over THIS thread's records only. Taking
        # the last _REASONING_HISTORY_FULL of the mixed list instead would
        # let a burst on the other thread push every same-thread record into
        # digest — measured on a real two-agent trace, answering Jack gave
        # zero full traces while three of the six were his.
        same_idxs = [i for i, r in enumerate(selected) if _same_thread(r)]
        full_idxs = (set(range(n)) if bench
                     else set(same_idxs[-_REASONING_HISTORY_FULL:]))
        sections: List[str] = []
        for idx, rec in enumerate(selected):
            seq = int(rec.get('turn_seq', 0))
            if seq:
                self._last_inject_trace_seqs.append(seq)
            sections.append(self._render_trace_record(
                rec, full=(idx in full_idxs)))
        if not sections:
            return ''
        return ("## Recent reasoning (my own ReAct traces from prior turns — "
                "per-turn orientation, concerns, recall, input, working "
                "log, and raw response. Each trace is headed by who its "
                "turn came from: traces from a different interlocutor are "
                "shown as a one-line digest, since they belong to a "
                "separate conversation. The static prefix and current state "
                "above also conditioned each turn; only per-turn-varying "
                "content is shown here.)\n"
                + "\n\n".join(sections))
