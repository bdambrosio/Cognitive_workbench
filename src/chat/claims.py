"""Claim-level provenance (Level-2 verifiability) — ClaimsMixin for ChatLoop.

Post-turn pass that decomposes the final reply into individual factual
claims, each attributed to its grounding: a tool observation ($stepN), a
recalled memory (Note_N), the user's own words, prior context, an
inference over those, or the model's parametric prior. Results are
appended to <memory>/claims.jsonl keyed by turn_seq — the same sidecar
pattern as memories.jsonl / autonomy.jsonl — because the reasoning trace
record is already written (append-only) by the time this runs.

Deliberately NO numeric confidences anywhere in the schema: the honest
v1 representation of uncertainty is the grounding type plus the evidence
pointers themselves. Probabilities enter later, if ever, as reliabilities
calibrated from outcome data — never as invented weights.

`attribute_claims` is a module-level function over a persisted trace
record dict, so the production path and offline validation run literally
the same code.
"""

from __future__ import annotations

import hashlib
import logging
import json
import re
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

logger = logging.getLogger('chat_loop')

CLAIM_GROUNDINGS = ('retrieved', 'memory', 'user_asserted', 'context',
                    'inferred', 'model_prior')

# Prompt-size caps for the attribution call: per-line and total caps on
# the working log (observations are untruncated in the trace and can be
# very large).
_LINE_CAP = 800
_LOG_CAP = 10_000

_STEP_LABEL_RE = re.compile(r'^\$step\d+', re.MULTILINE)
_NOTE_ID_RE = re.compile(r'\[(Note_\d+)')
_WS_RE = re.compile(r'\s+')


def _ws_normalize(s: str) -> str:
    """Collapse whitespace runs for the quote-verbatim check, so an
    honest copy isn't invalidated by line wrapping in the observation."""
    return _WS_RE.sub(' ', s).strip()

_ATTRIBUTION_SYS = """\
You are {character}'s provenance auditor. Given one conversational turn's \
records — the user's input, the assistant's final reply, recalled memories, \
and the working log of tool calls — decompose the REPLY into its factual \
claims and attribute each claim to its grounding.

A claim is a checkable factual assertion about the world, the user, or the \
assistant's own actions. Skip: greetings, opinions, questions, offers \
("let me know if..."), and speculation the reply itself marks as such.

Assign each claim exactly one grounding:
- retrieved: the claim's content appears in a tool observation this turn. \
refs = the $stepN binding(s) whose observation contains it. Also include \
"quote": a short excerpt (under 300 chars) copied character-for-character \
from that observation which contains or directly entails the claim.
- memory: the content comes from a recalled memory. refs = its Note_N id(s).
- user_asserted: the user stated it this turn. refs = ["user_input"].
- context: from earlier conversation shown in the prompt, not this turn's \
input or tools. refs = [].
- inferred: derived by reasoning over other evidence this turn. refs = the \
premises ($stepN / Note_N / user_input).
- model_prior: from the assistant's background knowledge; nothing in this \
turn's records supports it. refs = [].

Attribution discipline:
- Attribute by CONTENT SUPPORT, not co-occurrence. A tool having been \
called does not make a claim "retrieved" — the observation text must \
contain or directly entail the claim. If the reply asserts more than the \
evidence contains, the unsupported part is model_prior or inferred.
- Do not invent refs; use only refs from the ALLOWED REFS list.
- Quotes are machine-checked against the persisted observation and \
dropped if they do not match verbatim. Never paraphrase inside "quote"; \
if no verbatim span supports the claim, omit the field.
- No numeric confidence scores of any kind.

Output ONLY a JSON object of the form \
{{"claims": [{{"claim": "...", "grounding": "...", "refs": ["..."], \
"quote": "..."}}]}}. "quote" appears only on retrieved claims. \
Use an empty list if the reply makes no factual claims."""


def _cap_working_log(working_log: str) -> str:
    lines = []
    total = 0
    for line in (working_log or '').splitlines():
        if len(line) > _LINE_CAP:
            line = line[:_LINE_CAP] + ' …[capped]'
        total += len(line) + 1
        if total > _LOG_CAP:
            lines.append('…[working log capped]')
            break
        lines.append(line)
    return '\n'.join(lines)


def valid_refs_for(record: Dict[str, Any]) -> Set[str]:
    """The refs a claim may cite, derived structurally from the record:
    $stepN bindings present in the working log, Note_N ids present in
    the recall_hits tags, plus the literal "user_input"."""
    refs: Set[str] = {'user_input'}
    refs.update(_STEP_LABEL_RE.findall(record.get('working_log') or ''))
    for hit in (record.get('recall_hits') or []):
        refs.update(_NOTE_ID_RE.findall(str(hit)))
    return refs


def attribute_claims(record: Dict[str, Any],
                     llm_chat: Callable[[List[Dict[str, str]]], str],
                     character_name: str = 'the assistant'
                     ) -> Optional[List[Dict[str, Any]]]:
    """Decompose record['raw_response'] into attributed claims.

    `record` is one reasoning_trace.jsonl record dict. `llm_chat` maps a
    messages list to a completion string (caller bakes in model params).
    Returns the validated claims list ([] = reply carries no factual
    claims), or None when the LLM call or parse failed — the distinction
    matters downstream: [] is a finding, None is a pass that didn't run.
    """
    from utils.json_utils import repair_json_string

    reply = str(record.get('raw_response') or '').strip()
    if not reply or reply == '(no reply)':
        return []

    allowed = valid_refs_for(record)
    recall_block = '\n'.join(str(h) for h in (record.get('recall_hits') or []))
    user_parts = [
        f"## ALLOWED REFS\n{', '.join(sorted(allowed))}",
        f"## User input\n{record.get('user_input') or '(none — autonomous turn)'}",
        f"## Recalled memories\n{recall_block or '(none)'}",
        f"## Working log (tool calls and observations)\n"
        f"{_cap_working_log(record.get('working_log') or '') or '(no tool calls)'}",
        f"## REPLY to decompose\n{reply}",
    ]
    messages = [
        {'role': 'system',
         'content': _ATTRIBUTION_SYS.format(character=character_name)},
        {'role': 'user', 'content': '\n\n'.join(user_parts)},
    ]
    try:
        raw = llm_chat(messages)
    except Exception as e:
        logger.warning(f"claim attribution LLM call failed: {e}")
        return None
    obj = repair_json_string(raw or '')
    if not isinstance(obj, dict) or not isinstance(obj.get('claims'), list):
        logger.warning(
            f"claim attribution unparseable output: {str(raw)[:200]!r}")
        return None

    claims: List[Dict[str, Any]] = []
    for item in obj['claims']:
        if not isinstance(item, dict):
            continue
        text = str(item.get('claim') or '').strip()
        grounding = str(item.get('grounding') or '').strip()
        if not text:
            continue
        if grounding not in CLAIM_GROUNDINGS:
            logger.warning(
                f"claim attribution: unknown grounding {grounding!r} "
                f"for claim {text[:80]!r}; skipping")
            continue
        raw_refs = item.get('refs') or []
        refs = [r for r in raw_refs
                if isinstance(r, str) and r.strip() in allowed]
        dropped = [r for r in raw_refs if r not in refs]
        if dropped:
            logger.warning(
                f"claim attribution: dropped invalid refs {dropped} "
                f"for claim {text[:80]!r}")
        claim_rec = {'claim': text, 'grounding': grounding,
                     'refs': [r.strip() for r in refs]}
        # A quote must be a verbatim span of the PERSISTED working log
        # (the LLM saw a capped rendering, so anything it copied honestly
        # is a substring of the original). A failed check means the
        # excerpt was synthesized, not copied — drop the quote, keep the
        # claim: attribution stands, citation doesn't.
        quote = item.get('quote')
        if isinstance(quote, str) and quote.strip():
            log_text = record.get('working_log') or ''
            if _ws_normalize(quote) in _ws_normalize(log_text):
                claim_rec['quote'] = quote.strip()
            else:
                logger.warning(
                    f"claim attribution: quote failed verbatim check for "
                    f"claim {text[:80]!r}; dropped")
        claims.append(claim_rec)
    return claims


# ── justify: deterministic read path over the persisted provenance ──────
#
# The `justify` ReAct tool renders "why should I believe this?" from the
# records exactly as persisted (claims.jsonl + reasoning_trace.jsonl +
# memories.jsonl) — no LLM in the read path, so the trail cannot be
# re-synthesized or embellished. Module-level over a memory dir, same
# offline-equals-production property as attribute_claims.
#
# tools/trace_claim.py implements the same lookups independently: it is
# a standalone stdlib-only CLI by design (auditable without the src tree
# on the path), so the ~15 shared lines are deliberately not extracted.

_JUSTIFY_CLIP = 300


def _clip(s: Any, cap: int = _JUSTIFY_CLIP) -> str:
    s = str(s)
    return s if len(s) <= cap else s[:cap] + f' …[+{len(s) - cap} chars]'


def _scan_jsonl_last_match(path: Path, key: str, value: Any
                           ) -> Optional[Dict[str, Any]]:
    """Last matching record wins — same join semantics as the sidecar
    writers (a turn re-attributed, or a seq reused by an old session,
    is superseded by the latest record)."""
    if not path.is_file():
        return None
    found: Optional[Dict[str, Any]] = None
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get(key) == value:
                found = rec
    return found


def claims_for_turn(memory_dir: Path, turn_seq: int
                    ) -> Optional[Dict[str, Any]]:
    return _scan_jsonl_last_match(
        Path(memory_dir) / 'claims.jsonl', 'turn_seq', turn_seq)


def latest_turn_for_source(memory_dir: Path, source: str
                           ) -> Optional[Dict[str, Any]]:
    """Most recent reasoning-trace record for this conversation. Matching
    on `source` naturally excludes autonomous fires (their source is the
    character itself), so interleaved autonomous turns don't shadow the
    reply the user is asking about."""
    path = Path(memory_dir) / 'reasoning_trace.jsonl'
    if not path.is_file():
        return None
    found: Optional[Dict[str, Any]] = None
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get('source') == source:
                found = rec
    return found


def note_sidecar_lookup(memory_dir: Path, note_id: str
                        ) -> Optional[Dict[str, Any]]:
    """First write event for the note in memories.jsonl (notes are
    written once; later events for the same id are edits/supersessions
    and the original text is what the claim cited)."""
    path = Path(memory_dir) / 'memories.jsonl'
    if not path.is_file():
        return None
    with open(path, encoding='utf-8') as f:
        for line in f:
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get('note_id') == note_id:
                return rec
    return None


_GROUNDING_KEY = (
    "Grounding key: retrieved = stated in a tool observation that turn; "
    "memory = recalled from a persisted memory note; user_asserted = the "
    "user's own words that turn; context = earlier conversation; inferred = "
    "derived by reasoning from the cited evidence; model_prior = background "
    "knowledge — nothing recorded that turn supports it. Quoted spans are "
    "verbatim excerpts machine-checked against the persisted tool "
    "observation at attribution time.")


def render_justification(claims_rec: Dict[str, Any],
                         trace_rec: Dict[str, Any],
                         memory_dir: Path) -> str:
    """Render one turn's attributed claims plus a resolved evidence index
    as plain text. Deterministic — everything comes from the persisted
    records; nothing is summarized or re-judged by a model."""
    seq = claims_rec.get('turn_seq')
    replied_to = _clip(trace_rec.get('user_input') or '(autonomous turn)', 150)
    claims = claims_rec.get('claims') or []
    lines = [f"Provenance trail for your previous reply "
             f"(turn {seq}, replying to: {replied_to!r})."]
    if not claims:
        lines.append(
            "That reply made no checkable factual claims (greetings, "
            "opinions, questions and offers are not claims).")
        return '\n'.join(lines)

    lines.append(f"\nClaims ({len(claims)}):")
    cited: List[str] = []
    for i, c in enumerate(claims, 1):
        refs = c.get('refs') or []
        ref_txt = f" ← {', '.join(refs)}" if refs else ""
        lines.append(f"{i}. [{c.get('grounding')}{ref_txt}] {c.get('claim')}")
        if c.get('quote'):
            lines.append(f'   quote: "{_clip(c["quote"])}"')
        for r in refs:
            if r not in cited:
                cited.append(r)

    tool_meta = trace_rec.get('tool_meta') or {}
    ev: List[str] = []
    for ref in cited:
        if ref == 'user_input':
            ev.append(f"user_input — the user's own words that turn: "
                      f"{_clip(trace_rec.get('user_input') or '')!r}")
        elif ref.startswith('$step'):
            tm = tool_meta.get(ref)
            if not isinstance(tm, dict):
                ev.append(f"{ref} — tool observation that turn "
                          f"(no structured metadata recorded)")
                continue
            ev.append(f"{ref} — {tm.get('tool')}:")
            for entry in (tm.get('meta') or []):
                md = (entry or {}).get('tool_metadata') or {}
                query = md.get('query')
                sources = md.get('sources') or []
                if query:
                    ev.append(f"    query: {query}")
                for s in sources:
                    if isinstance(s, dict):
                        ev.append(f"    - {s.get('title') or s.get('domain') or '?'}"
                                  f" — {s.get('url') or ''}")
                if not query and not sources and md:
                    ev.append(f"    {_clip(json.dumps(md, default=str))}")
        elif ref.startswith('Note_'):
            rec = note_sidecar_lookup(memory_dir, ref)
            if rec is not None:
                date = str(rec.get('ts') or '')[:10]
                ev.append(f"{ref} — memory written {date}: "
                          f"{_clip(rec.get('text') or '')!r}")
            else:
                ev.append(f"{ref} — recalled memory "
                          f"(not found in memories.jsonl sidecar)")
        else:
            ev.append(f"{ref} — unrecognized ref form")
    if ev:
        lines.append("\nEvidence:")
        lines.extend(ev)

    profile: Dict[str, int] = {}
    for c in claims:
        g = str(c.get('grounding'))
        profile[g] = profile.get(g, 0) + 1
    lines.append("\nGrounding profile: " + ", ".join(
        f"{n} {g}" for g, n in
        sorted(profile.items(), key=lambda kv: (-kv[1], kv[0]))))
    # Structural audit trigger: fires on the validated grounding counts,
    # placed in the observation so the model reads it at exactly the
    # moment it decides whether the trail alone answers the user.
    if profile.get('model_prior'):
        lines.append(
            "Audit note: model_prior claims rest on training data with a "
            "cutoff. If any such claim concerns a current or changeable "
            "fact, verify it with a tool now before affirming it; if "
            "verification contradicts the original reply, lead with the "
            "correction.")
    lines.append("\n" + _GROUNDING_KEY)
    return '\n'.join(lines)


class ClaimsMixin:
    """Mixin for ChatLoop — post-turn claim extraction."""

    def _claims_log_path(self) -> 'Path':
        """Path to <memory>/claims.jsonl — one record per turn holding the
        reply's attributed claims. Append-only sidecar joined to
        reasoning_trace.jsonl by turn_seq; reply_sha1 ties the claims to
        the exact reply bytes they decompose."""
        return self._memory_dir() / 'claims.jsonl'

    def _load_trace_record(self, turn_seq: int) -> Optional[Dict[str, Any]]:
        """Fetch one persisted reasoning-trace record by turn_seq. Never
        trusts 'last line' (a newer turn may have appended while this
        post-turn job waited its executor slot), and takes the LAST
        match: turn_seq restarts at 1 each session, so earlier sessions'
        records can carry the same seq — the current session's record is
        always the latest occurrence."""
        path = self._reasoning_trace_path()
        if not path.is_file():
            return None
        found: Optional[Dict[str, Any]] = None
        try:
            with open(path, encoding='utf-8') as f:
                for line in f:
                    try:
                        rec = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if rec.get('turn_seq') == turn_seq:
                        found = rec
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] trace read for claims failed: {e}")
        return found

    def _extract_and_log_claims(self, turn_seq: int) -> None:
        """Attribute the turn's reply claims and append to claims.jsonl.
        Best-effort; reads the PERSISTED trace record so attribution
        audits exactly what is durable, not in-memory state."""
        record = self._load_trace_record(turn_seq)
        if record is None:
            logger.warning(
                f"[{self.character_name}] claims: no trace record for "
                f"turn_seq {turn_seq}")
            return
        reply = str(record.get('raw_response') or '').strip()
        if not reply or reply == '(no reply)':
            return

        def llm_chat(messages):
            return self.backend.chat(messages, max_tokens=1600,
                                     temperature=0.2, cot_profile='none')

        claims = attribute_claims(record, llm_chat,
                                  character_name=self.character_name)
        if claims is None:
            return  # already logged; nothing durable to write
        from utils.file_utils import append_jsonl
        append_jsonl(self._claims_log_path(), {
            'turn_seq': turn_seq,
            'source': record.get('source'),
            'reply_sha1': hashlib.sha1(reply.encode('utf-8')).hexdigest(),
            'claims': claims,
        }, character=self.character_name)

    def _run_justify(self, source: str) -> str:
        """Backend for the ReAct `justify` tool: render the provenance
        trail of the most recent reply to this conversation. Read-only
        and LLM-free — the observation is the persisted records."""
        memory_dir = self._memory_dir()
        trace_rec = latest_turn_for_source(memory_dir, source)
        if trace_rec is None:
            return ("EMPTY: no prior reply to this conversation in the "
                    "reasoning trace — nothing to justify yet.")
        seq = trace_rec.get('turn_seq')
        claims_rec = claims_for_turn(memory_dir, seq)
        if claims_rec is None:
            if trace_rec.get('autonomous'):
                return ("EMPTY: the previous turn here was autonomous; "
                        "claim attribution does not yet cover autonomous "
                        "turns, so no trail exists for it.")
            # Attribute on demand rather than waiting on the post-turn
            # executor — claims run last there, behind discourse and
            # reflection, which is minutes on a local backend (live
            # validation 2026-08-02: turn 2187's claims landed ~2.5 min
            # after the reply). A duplicate write from the still-queued
            # post-turn job is benign: claims_for_turn is last-match-wins.
            # Also covers turns persisted before claim attribution existed.
            self._extract_and_log_claims(int(seq))
            claims_rec = claims_for_turn(memory_dir, seq)
        if claims_rec is None:
            return (f"EMPTY: claim attribution for turn {seq} did not "
                    f"produce a record (see warning log).")
        return "OK: " + render_justification(claims_rec, trace_rec,
                                             memory_dir)
