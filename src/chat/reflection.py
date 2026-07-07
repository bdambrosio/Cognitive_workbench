"""Post-turn reflection (frame check → memories / user_concerns /
agent_concerns extraction) — ReflectionMixin for ChatLoop, moved verbatim
from chat_loop.py in the 2026-06 mixin refactor."""

from __future__ import annotations

import json
import logging
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from utils.json_utils import repair_json_string  # noqa: E402

from chat.memories import _MEMORY_CATEGORIES  # noqa: E402
from chat.react import REACT_MAX_ITERS  # noqa: E402

logger = logging.getLogger('chat_loop')

# Concern-instruction authoring rubric. Each fired concern dispatches
# one ReAct loop capped at REACT_MAX_ITERS — an instruction that can't
# be completed in that budget hits the max-iters fallback (which spawns
# a successor concern). The fallback exists as a safety net, not a
# crutch: authoring should default to narrow, single-step instructions.
# This canonical wording is referenced from the reflection prompt so
# the rubric and the budget stay in sync.
_CONCERN_INSTRUCTION_NARROWNESS_RULE = (
    f"Each `instruction` must be narrow enough to complete in roughly "
    f"{REACT_MAX_ITERS} ReAct iterations (one tool call per iter, plus a "
    f"final respond). Write single concrete actions (\"look up X and "
    f"summarize the result\"), not omnibus directives (\"investigate the "
    f"entire topic of Y\"). If the natural work is broader, pick the "
    f"most-immediate slice — successor concerns can carry the rest."
)

# Frame values the reflection LLM may return. Only `none` permits writes.
# Anything else (or unparseable / missing) suppresses the turn's memories
# entirely — the asymmetric cost of a poisoned memory is worse than
# missing one we'll re-encounter.
_REFLECT_FRAME_OK = 'none'

# STAGE 6 — fire outcomes (docs/fire-outcome-capture.md). Kept OUT of
# _REFLECT_SYS and appended to the formatted system message only when
# the pending-fire registry is non-empty: on the (common) empty-registry
# turn the reflection prompt must stay byte-identical to pre-capture
# behavior (KV-cache stability; existing benchmarks unaffected). Suffix
# position keeps the shared prefix cacheable when it does appear. No
# format placeholders — appended after .format(), so literal braces in
# the JSON example are safe.
_REFLECT_STAGE6_RULE = (
    "STAGE 6 — fire outcomes. For each act listed under \"## Recent "
    "autonomous acts awaiting outcome\", judge ONLY from evidence "
    "visible in this exchange whether the user's words or behavior show "
    "the act helped, was neutral, hindered, or is being ignored. Absence "
    "of mention is NOT evidence — omit the entry and it stays pending. "
    "Do not infer approval from politeness. valence is for the concern's "
    "domain; user_impact is for how the act landed with the user; they "
    "may disagree.\n"
    "Emit judged acts in `fire_outcomes` (usually this list is empty):\n"
    "  [{\"fire_id\": \"<id exactly as listed>\", "
    "\"outcome\": \"helped|neutral|hindered|ignored\", "
    "\"valence\": <-1.0..1.0>, \"user_impact\": <-1.0..1.0>, "
    "\"evidence\": \"<quote or paraphrase of the user evidence, <=200 chars>\"}, ...]"
)


class ReflectionMixin:
    """Mixin for ChatLoop — moved verbatim from chat_loop.py."""

    # Reflection prompt: extract durable episodic specifics from the latest
    # exchange. Two-stage decision:
    #   (a) Frame check. If the exchange is a hypothetical / role-play /
    #       counterfactual / instructional frame, suppress all writes. The
    #       cost of poisoning memory with frame-bound content is asymmetric
    #       — a bad memory taints every future recall hit on that topic;
    #       a missed real fact just gets re-asserted next time.
    #   (b) For real exchanges, extract memories AND tag each with a
    #       category: fact (default) / preference / commitment.
    # Companion already absorbs personality/style/mood, so the bar for
    # memory is "would NOT be recoverable from the companion model on a
    # fresh re-read" — names, places, commitments, stable preferences.
    _REFLECT_SYS = (
        "You watch chats between {character} and {entity}. Your job has "
        "four stages.\n\n"
        "STAGE 1 — Frame check. Classify the latest exchange as one of:\n"
        "- `hypothetical`: 'imagine that…', 'suppose…', 'what if…'\n"
        "- `roleplay`: user asked {character} to take on a persona, character, or voice\n"
        "- `counterfactual`: discussion of an alternate world / past / scenario\n"
        "- `instructional`: user is teaching {character} how to behave, not stating facts\n"
        "- `none`: a real exchange where statements about {entity}, the world, or "
        "agreements between you carry their literal weight\n"
        "If the frame is anything other than `none`, return all lists "
        "empty (memories, user_concerns, user_concerns_closed, "
        "agent_concerns). When in doubt, prefer the more conservative "
        "classification.\n\n"
        "STAGE 2 — Memories. If frame is `none`, extract stable specifics "
        "that should survive into FUTURE conversations.\n"
        "CAPTURE as memories:\n"
        "- Personal facts the user shared (names, places, relationships) → category=`fact`\n"
        "- Long-running project/work context → category=`fact`\n"
        "- Stable preferences expressed plainly (modifiers, not actions) → category=`preference`\n"
        "- Specific commitments or follow-ups agreed → category=`commitment`\n"
        "- Things {entity} explicitly REJECTED, ruled out, or said they "
        "do NOT want → same categories as above, but set "
        "`polarity`=`negative`. Phrase the text negatively (e.g. "
        "\"User rejected ChromaDB as the vector store\" / \"Does NOT "
        "want emoji in replies\"). Default polarity is `positive`; only "
        "set `negative` for explicit dispreference.\n"
        "SKIP from memories:\n"
        "- Pleasantries, mood, conversational tone (companion handles these).\n"
        "- Anything already in the companion model verbatim.\n"
        "- Anything semantically equivalent to an item shown in "
        "\"## Existing memories\" below — do not re-emit a memory we "
        "already hold. Emit only what is genuinely new or refines a prior "
        "fact in a way the existing wording does not capture.\n"
        "- One-off questions with no stable signal.\n\n"
        "STAGE 3 — User concerns. Topics {entity} is currently preoccupied "
        "with, has surfaced repeatedly, or has explicitly asked {character} "
        "to attend to. DISTINCT from memories: a memory is a stable fact "
        "({entity}'s brother is named Joe); a user_concern is an active "
        "preoccupation ({entity} is investigating concerns redesign).\n"
        "User concerns DO NOT fire autonomously. Their job is to inform "
        "{character}'s responses by surfacing in the prompt when relevant. "
        "Strength decays each turn unless reinforced by the user touching "
        "the topic again — so capture liberally; the runtime prunes what's "
        "not engaged.\n"
        "CAPTURE as user_concerns:\n"
        "- Active investigations / current preoccupations evidenced this turn\n"
        "- Topics user keeps returning to across exchanges\n"
        "- Things user explicitly said they're tracking or thinking about\n"
        "SKIP from user_concerns:\n"
        "- Stable identity facts (those go to memories)\n"
        "- One-off questions with no preoccupation signal\n"
        "- Items already covered by an existing user_concern in the prompt\n"
        "Schema per user_concern: `text` (≤120 chars — a stable short "
        "handle) plus `context` (2–3 sentences) — the part that makes the "
        "concern usable later, when the headline alone has gone stale. "
        "Cover: (1) what {entity} cares about, concretely; (2) the "
        "evidence from THIS exchange (what they said or did); (3) what "
        "{entity} appears to want from {character} about it — to be "
        "witnessed, to have it tracked, to get hands-on help, to have it "
        "researched, to discuss it. State that stance in plain language "
        "grounded in the evidence; don't guess beyond it. Strength is "
        "set to 1.0 by the runtime — don't include strength in your output.\n"
        "UPDATE user_concerns: if this exchange materially DEVELOPS a "
        "concern listed under \"## Existing user_concerns\" — new "
        "evidence, a shift in what {entity} wants, real progress — emit "
        "{{\"text\": <its text copied exactly as listed>, \"context\": "
        "<rewritten context>}} in `user_concerns_updated`. Rewrite the "
        "context as the CURRENT state of the concern, not a changelog. "
        "Update only on material change; usually this list is empty.\n"
        "CLOSE user_concerns: if this exchange shows that a concern listed "
        "under \"## Existing user_concerns\" is RESOLVED or NO LONGER LIVE "
        "— {entity} explicitly disclaimed it ('I'm past that', 'no longer "
        "worried about X'), described it as finished, or settled the "
        "question it tracked — put its text in `user_concerns_closed`, "
        "copied exactly as listed. Close ONLY on clear evidence in this "
        "exchange; mere silence about a topic is NOT closure (the runtime "
        "ages untouched concerns on its own). Usually this list is empty.\n\n"
        "STAGE 4 — Agent concerns. {character}'s OWN action queue: things "
        "{character} should be ready to advance. Authored from user "
        "requests with explicit deferred action, OR from seed concerns + "
        "observed context where {character} would benefit from periodic "
        "attention. DISTINCT from user_concerns: agent_concerns drive "
        "{character}'s autonomous action; user_concerns shape responses.\n"
        "Agent concerns FIRE autonomously when their activation grows past "
        "threshold AND they carry an instruction. Activation grows on a "
        "per-concern rhythm (rhythm_hours); each fire decrements activation.\n"
        "CAPTURE as agent_concerns:\n"
        "- User explicitly asked {character} to do/track something ongoing\n"
        "- User stated a deferred action ('remind me about X tomorrow')\n"
        "- {character} noticed a pattern that genuinely warrants recurring "
        "attention (use sparingly — most observations belong in memories or "
        "user_concerns, not as agent_concerns)\n"
        "SKIP from agent_concerns:\n"
        "- Modifiers like 'be brief' / 'don't use emoji' (memories→preferences)\n"
        "- Items already covered by an existing agent_concern in the prompt\n"
        "- Speculative inferences without textual support\n"
        "- Requests the user-driven turn already fulfilled this exchange — "
        "skip them entirely rather than logging an unfireable record\n"
        "Schema per agent_concern:\n"
        "- `text` (≤120 chars): short summary of what the concern is about.\n"
        "- `instruction` (string|null): the procedure {character} executes "
        "when this concern fires. null = the concern is logged for context "
        "but never fires. Usually a one-line imperative, but when the user "
        "DICTATES a multi-step procedure (URL templates, ranges, output "
        "rules, edge cases), capture the FULL spec verbatim — multi-paragraph "
        "instructions are encouraged in that case. The autonomous fire path "
        "passes this string straight to the ReAct loop, so anything you write "
        "here is what {character} will see when she executes. Examples:\n"
        "    \"Search for today's S&P 500 close and summarize the move.\"\n"
        "    \"Pull recent papers on multi-agent coordination from arxiv.\"\n"
        "    \"Query http://192.168.68.56:8086/query?db=pv&q=SELECT last(\\\"value\\\"),"
        " time FROM \\\"voltage\\\" WHERE time > now() - 15m GROUP BY *. Healthy "
        "range 51.5–57.5V. Flag if stale (>10m) or out of range. Silent on "
        "healthy for auto-checks; manual checks always report raw values.\"\n"
        "- `rhythm_hours` (int|null): target fire interval in hours. MUST be "
        "one of {{1, 2, 4, 8, 12, 24, 168}} or null. Pick from the underlying "
        "signal, not how often you'd nag the user:\n"
        "    hourly events (breaking news, intraday): 1 or 2\n"
        "    several-times-a-day work / project: 4 or 8\n"
        "    daily event (S&P close, daily roundup): 12 or 24\n"
        "    weekly check-in (project, hobby): 168\n"
        "    null = no autonomous fire (concern still logs).\n"
        "- `rhythm_source` (\"external\"|\"urgency\"|\"default\"):\n"
        "    `external` if user specified rhythm or topic has natural cadence\n"
        "    `urgency` if user signaled importance ('track this closely')\n"
        "    `default` if you guessed (default to 168 / weekly when guessing)\n"
        "{narrowness_rule}\n\n"
        "STAGE 5 — Capability gap. If, during THIS exchange, {character} "
        "needed a tool or capability she does not have in order to fully "
        "help {entity} — she had to decline, do the thing manually or "
        "awkwardly, or simply could not act — describe that gap in ONE "
        "plain sentence: what she needed to do and couldn't. This is about "
        "{character}'s OWN tooling, NOT {entity}'s preoccupations. Most "
        "turns have no gap — return null. Do not invent a gap to seem "
        "useful; only report one the exchange actually evidenced.\n\n"
        "Output ONLY this JSON shape — nothing else, no prose:\n"
        "  {{\"frame\": \"<hypothetical|roleplay|counterfactual|instructional|none>\",\n"
        "   \"memories\": [{{\"text\": \"...\", \"category\": \"fact|preference|commitment\", \"polarity\": \"positive|negative\"}}, ...],\n"
        "   \"user_concerns\": [{{\"text\": \"...\", \"context\": \"<2-3 sentences>\"}}, ...],\n"
        "   \"user_concerns_updated\": [{{\"text\": \"<exact text of an existing user_concern>\", \"context\": \"<rewritten>\"}}, ...],\n"
        "   \"user_concerns_closed\": [\"<exact text of an existing user_concern>\", ...],\n"
        "   \"agent_concerns\": [{{\n"
        "     \"text\": \"...\",\n"
        "     \"instruction\": \"<imperative>|null\",\n"
        "     \"rhythm_hours\": <int|null>,\n"
        "     \"rhythm_source\": \"external|urgency|default\"\n"
        "   }}, ...],\n"
        "   \"capability_gap\": \"<one sentence>\"|null}}\n\n"
        "WORKED EXAMPLE 1. {entity}: \"Please keep an eye on S&P 500 "
        "closes — I want to hear about them every day.\"\n"
        "Output:\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [{{\"text\": \"S&P 500 daily performance\",\n"
        "    \"context\": \"{entity} wants to follow the S&P 500's daily closes. Asked directly this exchange ('keep an eye on... every day'). Wants {character} to track it and report daily — not just be aware of it.\"}}],\n"
        "  \"user_concerns_updated\": [],\n"
        "  \"user_concerns_closed\": [],\n"
        "  \"agent_concerns\": [{{\n"
        "    \"text\": \"Track S&P 500 closing price daily.\",\n"
        "    \"instruction\": \"Search for today's S&P 500 close and summarize the day's move.\",\n"
        "    \"rhythm_hours\": 24,\n"
        "    \"rhythm_source\": \"external\"\n"
        "  }}]}}\n\n"
        "WORKED EXAMPLE 2. {entity}: \"I'm thinking about how concerns and "
        "tasks differ.\" (Just thinking aloud — no action requested.)\n"
        "Output (user_concern only; no agent_concern since no action):\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [{{\"text\": \"concerns vs tasks distinction\",\n"
        "    \"context\": \"{entity} is thinking through how concerns differ from tasks — an open conceptual question, not a request. Surfaced unprompted this exchange. Appears to want a thinking partner to discuss it with when it comes up, not solutions.\"}}],\n"
        "  \"user_concerns_updated\": [],\n"
        "  \"user_concerns_closed\": [],\n"
        "  \"agent_concerns\": []}}\n\n"
        "WORKED EXAMPLE 3. Existing user_concerns include \"- coping with "
        "feelings of being lost\". {entity}: \"That lost feeling I told you "
        "about has really lifted since I started the new project.\"\n"
        "Output:\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [],\n"
        "  \"user_concerns_updated\": [],\n"
        "  \"user_concerns_closed\": [\"coping with feelings of being lost\"],\n"
        "  \"agent_concerns\": []}}\n\n"
        "WORKED EXAMPLE 4. Existing user_concerns include \"- concerns vs "
        "tasks distinction\". {entity}: \"I think I've cracked the concerns "
        "thing — they're persistent evaluative pressure, tasks are just "
        "what falls out when one crosses a threshold. Now I want to figure "
        "out the triage step in between.\"\n"
        "Output (material development → rewrite the context):\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [],\n"
        "  \"user_concerns_updated\": [{{\"text\": \"concerns vs tasks distinction\",\n"
        "    \"context\": \"{entity} has settled the core distinction (concerns = persistent evaluative pressure; tasks = what emerges past threshold) and moved on to designing the triage step between them. Wants a thinking partner on triage design specifically.\"}}],\n"
        "  \"user_concerns_closed\": [],\n"
        "  \"agent_concerns\": []}}\n\n"
        "WORKED EXAMPLE 5 (capability gap). {entity}: \"Can you convert 3pm "
        "Tokyo time to my timezone?\" {character} has no timezone tool and "
        "works it out by hand. A real exchange with no durable items, but a "
        "tooling gap surfaced:\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [],\n"
        "  \"user_concerns_updated\": [],\n"
        "  \"user_concerns_closed\": [],\n"
        "  \"agent_concerns\": [],\n"
        "  \"capability_gap\": \"I had to convert between timezones by hand "
        "because I have no tool for timezone conversion.\"}}\n\n"
        "If frame≠none or nothing qualifies: return the envelope with all "
        "lists empty and capability_gap null."
    )

    def _reflect_and_remember(self, source: str) -> Tuple[List[str], List[str], List[str]]:
        """Run a single reflection LLM call over the latest exchange; persist
        memories, user_concerns, agent_concerns. Returns the three written-text
        lists. Failure-tolerant: any error path returns three empty lists."""
        if not self._memories_collection_id:
            return ([], [], [])
        try:
            dialog = self._build_dialog(source, limit=4)
            if not dialog:
                return ([], [], [])
            convo = "\n".join(f"{t['source']}: {t['text']}" for t in dialog)
            companion = self._companion_state.get(source, '').strip()
            # Show the LLM the existing concerns from BOTH collections so it
            # doesn't re-derive ones we already track. Two compact sections,
            # one per collection.
            existing_agent = self._top_active_agent_concerns(n=10)
            existing_user = self._top_active_user_concerns(n=10)
            # Memory neighbors near the dialog topic — used by the LLM to
            # avoid re-emitting a memory we already hold (write-time
            # dedupe). Threshold lower than auto-RAG (0.4 vs 0.5) and
            # k larger (8 vs 3) because false-positives here suppress a
            # write that *might* be redundant; false-negatives produce a
            # duplicate. Suppression is the cheaper failure mode.
            existing_memories = self._recall(convo, k=8, threshold=0.4)
            # Pending fire outcomes (STAGE 6): read-only registry peek.
            # Empty on most turns — everything stage-6 below is gated on
            # this list so the empty-registry prompt stays byte-identical
            # to pre-capture behavior. Load failure → [] (stays pending).
            pending_fires = self._load_pending_fire_outcomes()
            sys_msg = self._REFLECT_SYS.format(
                character=self.character_name, entity=source,
                narrowness_rule=_CONCERN_INSTRUCTION_NARROWNESS_RULE)
            if pending_fires:
                sys_msg += "\n\n" + _REFLECT_STAGE6_RULE
            user_parts = []
            if companion:
                user_parts.append(
                    "## Existing companion model (do NOT re-extract from this; "
                    "use only to avoid duplicates)\n" + companion)
            if existing_memories:
                mem_lines = []
                for text, cat, pol in existing_memories:
                    marker = '[avoid] ' if pol == 'negative' else ''
                    mem_lines.append(f"- ({cat}) {marker}{text}")
                user_parts.append(
                    "## Existing memories (do NOT re-emit anything "
                    "semantically equivalent to these; emit only NEW or "
                    "genuinely-refining memories)\n" + "\n".join(mem_lines))
            if existing_user:
                # Context rides along so dedup / update / close judgments
                # see content, not just terse headlines (reduces
                # thematic-sibling creation).
                lines = []
                for _nid, text, _s, p in existing_user:
                    ctx = str((p or {}).get('context', '') or '').strip()
                    lines.append(f"- {text} — {ctx[:100]}" if ctx else f"- {text}")
                user_parts.append(
                    "## Existing user_concerns (do NOT re-emit; emit only "
                    "NEW user_concerns this exchange surfaced)\n" + "\n".join(lines))
            if existing_agent:
                lines = [f"- {text}" for _nid, text, _a, _p in existing_agent]
                user_parts.append(
                    "## Existing agent_concerns (do NOT re-emit; emit only "
                    "NEW agent_concerns this exchange surfaced)\n" + "\n".join(lines))
            if pending_fires:
                # Identity + reach-back beyond the 4-turn dialog window;
                # the reaction evidence itself is usually already in the
                # dialog above (auto_say lines to the same entity).
                lines = [
                    f"- [{r.get('fire_id', '')}] concern: "
                    f"{r.get('concern_text', '')} — {self.character_name} "
                    f"did/said: {r.get('reply_digest', '')} "
                    f"({int(r.get('user_turns_since', 0) or 0)} user turns ago)"
                    for r in pending_fires]
                user_parts.append(
                    "## Recent autonomous acts awaiting outcome\n"
                    + "\n".join(lines))
            user_parts.append("## Latest exchange\n" + convo)
            # Key list grows `fire_outcomes` only when the section above
            # was shown — byte-identical otherwise (KV-cache stability).
            return_keys = ("frame, memories, user_concerns, "
                           "user_concerns_updated, user_concerns_closed, "
                           "agent_concerns")
            if pending_fires:
                return_keys += ", fire_outcomes"
            user_parts.append(
                f"Return the JSON object now (keys: {return_keys}). All "
                "lists empty if frame≠none or nothing qualifies.")
            result = self._llm_generate(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': "\n\n".join(user_parts)}],
                max_tokens=8192, temperature=0.3, is_json=True,
                cot_profile='none')
            if not result.success:
                return ([], [], [])
            payload = result.text
            try:
                _preview = json.dumps(payload) if not isinstance(payload, str) else payload
            except Exception:
                _preview = repr(payload)
            logger.info(f"[{self.character_name}] reflection raw: {_preview[:800]}")
            if isinstance(payload, str):
                payload = repair_json_string(payload)
                if payload is None:
                    return ([], [], [])

            (frame, raw_memories, raw_user_concerns, raw_user_concerns_updated,
             raw_user_concerns_closed, raw_agent_concerns) = (
                self._parse_reflection_payload(payload))

            if frame != _REFLECT_FRAME_OK:
                logger.info(
                    f"[{self.character_name}] reflection suppressed "
                    f"(frame={frame!r}) — nothing written")
                return ([], [], [])

            # Capability gap (STAGE 5): {character} lacked a tool this turn.
            # Routed to the self-extension concern (WIP + evidence bump),
            # not the memory/concern lists. Read straight off the payload —
            # _parse_reflection_payload doesn't carry it.
            if isinstance(payload, dict):
                gap = str(payload.get('capability_gap') or '').strip()
                if gap and gap.lower() not in ('null', 'none'):
                    self._record_capability_gap(gap)

            # Fire outcomes (STAGE 6): judged acts leave the pending
            # registry and land in autonomy.jsonl. Read straight off the
            # payload like capability_gap — _parse_reflection_payload
            # doesn't carry it. Gated on pending_fires: we only accept
            # judgments for records the LLM was actually shown.
            if isinstance(payload, dict) and pending_fires:
                self._apply_fire_outcome_judgments(
                    payload.get('fire_outcomes'), pending_fires)

            mems_written: List[str] = []
            for text, category, polarity in raw_memories:
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._remember(text, entity=source, category=category,
                                  polarity=polarity):
                    mems_written.append(text)

            user_cons_written: List[str] = []
            for text, context in raw_user_concerns:
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._add_user_concern(text, entity=source, context=context):
                    user_cons_written.append(text)

            # Updates: the exchange materially developed an existing
            # concern — rewrite its context (current-state, not changelog).
            # Resolved against the same top-K list the LLM was shown.
            user_cons_updated: List[str] = []
            for text, context in raw_user_concerns_updated:
                nid = self._resolve_user_concern_by_text(text, existing_user)
                if not nid:
                    logger.info(
                        f"[{self.character_name}] reflection update skipped — "
                        f"no match for {text[:80]!r}")
                    continue
                if self._update_user_concern_context(nid, context):
                    user_cons_updated.append(text)

            # Closures: the missing half of the single-patch design — the
            # reflection LLM saw evidence this exchange that an existing
            # concern is resolved. Resolve text → note id against the same
            # top-K list it was shown (exact match first, then the 0.8
            # recurrence search) and mark satisfied. Reopens automatically
            # via recurrence bump if the theme genuinely returns.
            user_cons_closed: List[str] = []
            for text in raw_user_concerns_closed:
                nid = self._resolve_user_concern_by_text(text, existing_user)
                if not nid:
                    logger.info(
                        f"[{self.character_name}] reflection close skipped — "
                        f"no match for {text[:80]!r}")
                    continue
                ok, err = self._set_concern_status(nid, 'satisfied')
                if ok:
                    user_cons_closed.append(text)
                else:
                    logger.warning(
                        f"[{self.character_name}] reflection close failed "
                        f"for {nid}: {err}")

            agent_cons_written: List[str] = []
            for c in raw_agent_concerns:
                text = c.get('text', '')
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._add_agent_concern(
                        text, entity=source,
                        provenance='asserted',
                        seed=False,
                        rhythm_hours=c.get('rhythm_hours'),
                        rhythm_source=c.get('rhythm_source') or 'default',
                        instruction=c.get('instruction')):
                    agent_cons_written.append(text)

            if (mems_written or user_cons_written or user_cons_updated
                    or user_cons_closed or agent_cons_written):
                logger.info(
                    f"[{self.character_name}] reflection wrote "
                    f"{len(mems_written)} memory(s), "
                    f"{len(user_cons_written)} user_concern(s) "
                    f"(+{len(user_cons_updated)} updated, "
                    f"+{len(user_cons_closed)} closed), "
                    f"{len(agent_cons_written)} agent_concern(s) from {source}")
            return (mems_written, user_cons_written, agent_cons_written)
        except Exception as e:
            logger.warning(f"[{self.character_name}] _reflect_and_remember failed: {e}")
            return ([], [], [])

    @staticmethod
    def _parse_reflection_payload(
            payload: Any
    ) -> Tuple[str, List[Tuple[str, str, str]], List[Tuple[str, str]],
               List[Tuple[str, str]], List[str], List[Dict[str, Any]]]:
        """Normalize reflection output to (frame, memories, user_concerns,
        user_concerns_updated, user_concerns_closed, agent_concerns).
        memories: list of (text, category, polarity) tuples. Polarity is
                  'positive' (default) or 'negative'.
        user_concerns: list of (text, context) tuples; context may be ''.
        user_concerns_updated: list of (text, context) tuples — text names
                               an existing concern, context is its rewrite.
                               Entries without a context are dropped.
        user_concerns_closed: list of text strings naming existing
                              user_concerns the exchange showed resolved.
        agent_concerns: list of dicts with keys text, instruction,
                        rhythm_hours, rhythm_source (any may be missing/None).

        Accepts:
          - New envelope: {"frame", "memories", "user_concerns",
            "user_concerns_updated", "user_concerns_closed",
            "agent_concerns"} (updated/closed keys optional)
          - Legacy envelope with single "concerns" key (single-channel design):
            all concerns are routed to agent_concerns; cadence_hours field is
            read as rhythm_hours.
          - Bare list of strings: assumed frame=none, treated as memories.
        Anything else returns ('unknown', [], [], [], [], []) — caller treats
        non-`none` frame as suppression so this fails safe.
        """

        def _normalize_memories(raw: Any) -> List[Tuple[str, str, str]]:
            out: List[Tuple[str, str, str]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append((item.strip(), 'fact', 'positive'))
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    c = str(item.get('category', 'fact') or 'fact').strip().lower()
                    if c not in _MEMORY_CATEGORIES:
                        c = 'fact'
                    p = str(item.get('polarity', 'positive') or 'positive').strip().lower()
                    if p not in ('positive', 'negative'):
                        p = 'positive'
                    if t:
                        out.append((t, c, p))
            return out

        def _normalize_user_concern_texts(raw: Any) -> List[str]:
            out: List[str] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append(item.strip())
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    if t:
                        out.append(t)
            return out

        def _normalize_user_concern_pairs(raw: Any,
                                          require_context: bool = False
                                          ) -> List[Tuple[str, str]]:
            """(text, context) pairs. Bare strings get context='' unless
            require_context (the updated key is meaningless without one)."""
            out: List[Tuple[str, str]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    if not require_context:
                        out.append((item.strip(), ''))
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    c = str(item.get('context', '') or '').strip()
                    if t and (c or not require_context):
                        out.append((t, c))
            return out

        def _normalize_agent_concerns(raw: Any) -> List[Dict[str, Any]]:
            out: List[Dict[str, Any]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append({'text': item.strip(), 'instruction': None,
                                'rhythm_hours': None, 'rhythm_source': 'default'})
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    if not t:
                        continue
                    instr = item.get('instruction')
                    rhythm_h = item.get('rhythm_hours')
                    if rhythm_h is None:
                        rhythm_h = item.get('cadence_hours')   # legacy field
                    if rhythm_h is None and item.get('cadence_days') is not None:
                        try:
                            rhythm_h = float(item.get('cadence_days')) * 24.0
                        except (TypeError, ValueError):
                            rhythm_h = None
                    src = str(item.get('rhythm_source', 'default') or 'default').strip().lower()
                    if src not in ('external', 'urgency', 'default'):
                        src = 'default'
                    out.append({
                        'text': t,
                        'instruction': str(instr).strip() if instr else None,
                        'rhythm_hours': rhythm_h,
                        'rhythm_source': src,
                    })
            return out

        # Old shape — bare list. Assume frame=none, all memories.
        if isinstance(payload, list):
            return (_REFLECT_FRAME_OK, _normalize_memories(payload),
                    [], [], [], [])

        if isinstance(payload, dict):
            frame = str(payload.get('frame', '') or '').strip().lower() or 'unknown'
            mems = _normalize_memories(payload.get('memories', []))
            user_cons = _normalize_user_concern_pairs(
                payload.get('user_concerns', []))
            user_cons_updated = _normalize_user_concern_pairs(
                payload.get('user_concerns_updated', []), require_context=True)
            user_cons_closed = _normalize_user_concern_texts(
                payload.get('user_concerns_closed', []))
            # Prefer the new agent_concerns field; fall back to legacy
            # single-channel `concerns` (route to agent collection).
            agent_cons_raw = payload.get('agent_concerns')
            if agent_cons_raw is None:
                agent_cons_raw = payload.get('concerns', [])
            agent_cons = _normalize_agent_concerns(agent_cons_raw)
            return (frame, mems, user_cons, user_cons_updated,
                    user_cons_closed, agent_cons)

        return ('unknown', [], [], [], [], [])
