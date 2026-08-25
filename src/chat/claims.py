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

# Taxonomy tags (docs/justification-taxonomy.md). Closed vocabularies,
# assigned semantically by the attribution LLM and validated here; an
# absent tag is always valid and never lowers a grade below the
# untagged default.
VOLATILITY_TAGS = ('volatile', 'stable')          # model_prior / memory
POLARITY_TAGS = ('presence', 'absence')           # retrieved
ADEQUACY_TAGS = ('adequate', 'inadequate')        # negation-from-absence
INFERENCE_TAGS = ('entailment', 'deduction', 'calculation',
                  'generalization', 'extrapolation', 'analogy',
                  'causal-attribution', 'negation-from-absence',
                  'evidence-repurposing', 'circular-support',
                  'paraphrase', 'corroboration')  # inferred

# Ordinal grades, best to worst. refuted/conflict exist in the taxonomy
# but are never produced by this reducer (they require a verification
# pass or disagreeing evidence, neither of which v1 models).
#
# The top grade is `sourced`, not `verified`. The audit notes tell the
# model to *verify* a weak claim with a tool; having done so, calling the
# claim "verified" is the obvious English move — and it names a formal
# grade only this reducer can assign. Observed live at turn 2403: a
# correct verification reported as "upgraded to verified", a grade change
# that appears nowhere in the tool output. The collision was in the
# vocabulary, not the reasoning, so the word is what changed.
GRADES = ('sourced', 'probable', 'unverified', 'suspect')
_GRADE_RANK = {g: i for i, g in enumerate(GRADES)}


def _worst(*grades: str) -> str:
    return max(grades, key=lambda g: _GRADE_RANK[g])

# Prompt-size caps for the attribution call: per-line and total caps on
# the working log. Observations in the stored trace are capped at
# _REASONING_HISTORY_OBS_CAP; _restore_observations puts the elided text
# back before these caps apply, so the total has to accommodate a
# restored observation rather than the 1000-char stub.
#
# Policy, not a context limit — one turn's log cannot exceed
# _ATTRIBUTION_OBS_CAP x REACT_MAX_ITERS (128k chars at 8k x 16) however these are
# set. Sized from the corpus: over 2848 turns the restored log runs 1k at
# the median and 52,910 at its worst; lines run 38 and 9,215. The old
# 800/24,000 cut 1.95% of lines and 0.53% of turns, and the turn cut is
# the expensive one — it lands mid-way through the tool calls, so whole
# observations never reach the attributor and facts the model read
# verbatim grade model_prior (turn 2847, steps 4-6 invisible, a
# verification fire spawned against evidence already in hand).
#
# Raising them is the fix; redistributing them is not. Clipping each
# observation to an equal share of the same budget was measured and is
# worse: the cuts land mid-span on quoted text and verbatim-quote
# failures went 6 -> 15 across the affected turns.
_LINE_CAP = 4_000
_LOG_CAP = 60_000

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
- context: from the assistant's own prompt — earlier conversation, or the \
ASSISTANT STATE section (its active concerns, and the harness provenance and \
body it was told about itself at session start) — not this turn's input or \
tools. refs = [].
- inferred: derived by reasoning over other evidence this turn. refs = the \
premises ($stepN / Note_N / user_input).
- model_prior: from the assistant's background knowledge; nothing in this \
turn's records supports it. refs = [].

An observation may be shown to you truncated, marked "…[capped]" or \
"…[working log capped]". The assistant read the whole thing; you are \
seeing less than it saw. Absence of evidence in a truncated observation \
is NOT evidence the claim came from background knowledge. When a claim \
is the kind of specific detail the truncated observation was plainly \
carrying — a further item in a list it had begun, another figure from \
the same table or passage — attribute it to that $stepN as retrieved and \
omit "quote" (you cannot copy text you were not shown) rather than \
grading it model_prior.

Additionally tag claims where the tag clearly applies (closed \
vocabularies; OMIT any tag you are unsure of — an absent tag is valid):
- model_prior and memory claims — "volatility": "volatile" (a fact of a \
kind that CAN change after a training cutoff: what state an organization \
is in, prices, versions, roles, availability, schedules — even if it has \
held for years) or "stable" (definitions, mathematics, settled past \
events). Judge the KIND of fact, not your confidence in it; when unsure, \
use volatile.
- retrieved claims — "polarity": "presence" (the observation contains \
the finding) or "absence" (the claim rests on the observation NOT \
containing something).
- inferred claims — "inference", one of: entailment (restates the cited \
evidence), deduction (follows logically from the cited premises), \
calculation (arithmetic/unit/date transform), generalization (instances \
to a rule), extrapolation (projects a trend forward), analogy (maps \
from a similar case), causal-attribution (cause from sequence or \
correlation), negation-from-absence (didn't find it, so it doesn't \
exist), evidence-repurposing (evidence gathered for a different \
question), circular-support (the conclusion, restated, is among its own \
premises), paraphrase (restates a source without a verbatim anchor), \
corroboration (multiple independent sources agree).
- negation-from-absence claims only — "query_adequacy": "adequate" (the \
probe was designed to find the thing if it existed) or "inadequate" \
(the evidence came from a probe shaped for a different question).

Attribution discipline:
- The assistant is authoritative for its own current state. A claim \
describing the assistant's own concerns, tasks, tools, configuration, the \
code revision it is running or the body it has, that is supported by the \
ASSISTANT STATE section, is "context", never model_prior — it restates \
recorded state, not background knowledge. This holds for what a commit \
subject in that section says the harness now does: read there, it is \
context, however much it also sounds like something the model might \
simply know.
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
"quote": "...", "volatility": "...", "polarity": "...", \
"inference": "...", "query_adequacy": "..."}}]}}. "quote" appears only \
on retrieved claims; tag fields only where defined above and known. \
Use an empty list if the reply makes no factual claims."""


_OBS_CAP_MARKER_RE = re.compile(r' …\[observation capped at \d+ chars\]')


def _restore_observations(working_log: str,
                          observations_full: Dict[str, str]) -> str:
    """Put elided observation text back into the working log.

    The stored trace caps each observation so the record stays cheap to
    re-inject into later prompts, but attribution reads the same field
    as evidence — and a truncated observation is indistinguishable from
    an absent one, so facts the model read verbatim get graded
    model_prior. Records written before `observations_full` existed
    simply have nothing to restore.
    """
    if not observations_full:
        return working_log
    out = working_log
    for label, text in observations_full.items():
        start = out.find(f"\n{label}: ")
        if start < 0:
            continue
        marker = _OBS_CAP_MARKER_RE.search(out, start)
        if marker is None:
            continue      # not the capped rendering this entry describes
        out = out[:start] + f"\n{label}: {text}" + out[marker.end():]
    return out


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


def _salvage_truncated_claims(raw: str) -> Optional[List[Dict[str, Any]]]:
    """Recover the complete claim objects from a cut-off attribution.

    Schema-aware on purpose, and local to this module rather than in
    json_utils: "drop the trailing partial element" is only sound because
    a claims array's elements are independent. Returns None when nothing
    complete can be recovered, so the caller can still distinguish a
    truncated pass from an empty finding.
    """
    text = str(raw or '')
    start = text.find('"claims"')
    if start < 0:
        return None
    bracket = text.find('[', start)
    if bracket < 0:
        return None
    out: List[Dict[str, Any]] = []
    depth, obj_start, in_str, esc = 0, None, False, False
    for i in range(bracket + 1, len(text)):
        ch = text[i]
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '{':
            if depth == 0:
                obj_start = i
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0 and obj_start is not None:
                try:
                    item = json.loads(text[obj_start:i + 1])
                except json.JSONDecodeError:
                    break
                if isinstance(item, dict):
                    out.append(item)
                obj_start = None
        elif ch == ']' and depth == 0:
            break
    return out or None


def attribute_claims(record: Dict[str, Any],
                     llm_chat: Callable[[List[Dict[str, str]]], str],
                     character_name: str = 'the assistant',
                     status: Optional[Dict[str, Any]] = None
                     ) -> Optional[List[Dict[str, Any]]]:
    """Decompose record['raw_response'] into attributed claims.

    `record` is one reasoning_trace.jsonl record dict. `llm_chat` maps a
    messages list to a completion string (caller bakes in model params).
    Returns the validated claims list ([] = reply carries no factual
    claims), or None when the LLM call or parse failed — the distinction
    matters downstream: [] is a finding, None is a pass that didn't run.

    `status`, when given, is filled in with what happened: 'truncated' and
    'salvaged' when the output was cut off and partially recovered,
    'error' with a short reason when the pass produced nothing. The caller
    needs that to mark a partial record and to say why a trail is missing.
    """
    from utils.json_utils import repair_json_string

    reply = str(record.get('raw_response') or '').strip()
    if not reply or reply == '(no reply)':
        return []

    allowed = valid_refs_for(record)
    # Evidence view: elided observations restored. Also the reference the
    # verbatim quote check runs against — the attributor is shown this
    # text, so a quote copied from a restored span must verify against it
    # rather than against the capped stored log.
    restored_log = _restore_observations(
        record.get('working_log') or '', record.get('observations_full') or {})
    recall_block = '\n'.join(str(h) for h in (record.get('recall_hits') or []))
    # The replying model saw its own active concerns in its prompt; the
    # attributor must see them too, or self-state claims have no visible
    # support and fall through to model_prior (live miss: turn 2316).
    # Operational self-state the replying model was shown: its concerns,
    # plus the harness provenance it was told at session start (which commit
    # it runs, what body it has). All three are prompt content, so a claim
    # restating them is `context` — but only if the attributor can see them.
    # It could not see the substrate line, and turn 2447's "you look like a
    # crow" — lifted verbatim from a commit subject in that block — graded
    # model_prior/volatile → suspect, then spawned a background job to go
    # web-verify a fact recorded in this repo's git log.
    state_lines = [str(c) for c in (record.get('active_concerns') or [])]
    # Keys are `substrate_line` / `embodiment_line`: chat_loop writes them as
    # `record[key.strip('_')]` from `_substrate_line` / `_embodiment_line`.
    # This read asked for 'substrate' / 'embodiment' and so never matched —
    # 0 of 3,010 jill_chat rows carry those names, 561 carry the _line ones.
    # The block below was added to close turn 2447 and had been inert since.
    for key, label in (('substrate_line', 'harness provenance'),
                       ('embodiment_line', 'body')):
        val = str(record.get(key) or '').strip()
        if val:
            state_lines.append(f"[{label}] {val}")
    state_block = _cap_working_log('\n'.join(state_lines))
    user_parts = [
        f"## ALLOWED REFS\n{', '.join(sorted(allowed))}",
        f"## ASSISTANT STATE (the assistant's own active concerns, plus the "
        f"harness provenance and body it was told at session start — all of "
        f"it shown in its prompt this turn)\n{state_block or '(none)'}",
        f"## User input\n{record.get('user_input') or '(none — autonomous turn)'}",
        f"## Recalled memories\n{recall_block or '(none)'}",
        f"## Working log (tool calls and observations)\n"
        f"{_cap_working_log(restored_log) or '(no tool calls)'}",
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
        if status is not None:
            status['error'] = f"backend call failed: {e}"
        return None
    obj = repair_json_string(raw or '')
    if not isinstance(obj, dict) or not isinstance(obj.get('claims'), list):
        # repair_json_string cannot rescue a truncated claims array — it
        # closes braces, never the array, and a half-written trailing
        # claim would survive if it did. Salvage the complete objects and
        # drop the partial one, so a cut-off pass yields a marked-partial
        # trail instead of nothing.
        salvaged = _salvage_truncated_claims(raw or '')
        if salvaged is None:
            logger.warning(
                f"claim attribution unparseable output ({len(str(raw))} chars, "
                f"unsalvageable): {str(raw)[:200]!r}")
            if status is not None:
                status['error'] = 'output unparseable'
            return None
        logger.warning(
            f"claim attribution truncated; salvaged {len(salvaged)} complete "
            f"claim(s) from {len(str(raw))} chars of output")
        if status is not None:
            status['truncated'] = True
            status['salvaged'] = len(salvaged)
        obj = {'claims': salvaged}

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
        # Taxonomy tags: keep only values from the closed vocabulary on
        # a grounding the tag is defined for; drop everything else with
        # a log line. An absent tag is always valid.
        tag_rules = (
            ('volatility', VOLATILITY_TAGS, grounding in ('model_prior',
                                                          'memory')),
            ('polarity', POLARITY_TAGS, grounding == 'retrieved'),
            ('inference', INFERENCE_TAGS, grounding == 'inferred'),
            ('query_adequacy', ADEQUACY_TAGS,
             item.get('inference') == 'negation-from-absence'),
        )
        for field, vocab, applicable in tag_rules:
            val = item.get(field)
            if val is None:
                continue
            if applicable and val in vocab:
                claim_rec[field] = val
            else:
                logger.warning(
                    f"claim attribution: dropped tag {field}={val!r} "
                    f"({grounding=}) for claim {text[:80]!r}")
        # A quote must be a verbatim span of the evidence the attributor
        # was shown (the restored log — a superset of the stored one). A
        # failed check means the excerpt was synthesized, not copied —
        # drop the quote, keep the claim: attribution stands, citation
        # doesn't.
        quote = item.get('quote')
        if isinstance(quote, str) and quote.strip():
            if _ws_normalize(quote) in _ws_normalize(restored_log):
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


def latest_turn_for_source(memory_dir: Path, source: str,
                           turn_seq: Optional[int] = None
                           ) -> Optional[Dict[str, Any]]:
    """Reasoning-trace record for this conversation: the most recent, or
    the one with `turn_seq` when given.

    Matching on `source` naturally excludes autonomous fires (their source
    is the character itself), so interleaved autonomous turns don't shadow
    the reply the user is asking about. The same predicate is what keeps a
    turn_seq from reaching into another conversation's trail — seqs are
    global, conversations are not, so a bare seq lookup could render a
    peer's or another channel's turn here.

    Last match wins: seqs 1..50 repeat across pre-seeding sessions, and the
    later record is the live one.
    """
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
            if rec.get('source') != source:
                continue
            if turn_seq is not None and rec.get('turn_seq') != turn_seq:
                continue
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


# Edge-type caps for inferred claims (taxonomy: inference edge types).
# Types absent from this map (entailment, deduction, calculation,
# corroboration) carry the premise grade through uncapped.
_INFERENCE_CAPS = {
    'generalization': 'probable',
    'extrapolation': 'probable',
    'paraphrase': 'probable',
    'analogy': 'unverified',
    'causal-attribution': 'unverified',
    'evidence-repurposing': 'unverified',
    'circular-support': 'suspect',
}


# Mediation of a tool's observation — orthogonal to the spec's
# `source-grade` (authority of the source *for the claim*). This axis asks
# a narrower question: did this harness read the evidence, or is the
# observation some model's account of something it read elsewhere?
#
#   'direct'   — the observation IS retrieved content: fetched bytes, API
#                rows, file text, a deterministic render.
#   'mediated' — an LLM wrote the observation. Local-ground-truth
#                subagents (inspect, security, recall) still land here:
#                the underlying reads were real, but the text the claim is
#                quoted against is a synthesis of them.
#
# Effect: a claim quoted against a mediated observation cannot exceed
# `probable`, whatever its source-grade — a verbatim match against a
# synthesis proves the synthesis said it, not that the source did.
# Unknown tools default to 'mediated': the cap is the safe direction.
_DIRECT_OBSERVATION_TOOLS = frozenset({
    # `tavily` sits here and `search-web` does not, which is the whole
    # difference between them: tavily's observation is the pages' own
    # text and verbatim snippets, search-web's is a model's synthesis
    # about those pages. The tool pins include_answer off and
    # include_raw_content on so the distinction holds for every call and
    # can be decided by name, as this set requires.
    'fetch-text', 'tavily', 'obsidian', 'semantic-scholar', 'check-x-feed',
    'check-email', 'stock-price', 'get-financial-statements',
    'calculate', 'text-find', 'justify',
    # Reads the machine directly (nvidia-smi, /proc, the health
    # script) rather than a model's account of it.
    'system-info',
    'fac-status', 'fac-observe', 'fac-inventory', 'fac-nearest',
    'world-look',
})


def observation_mediation(tool_name: Optional[str]) -> str:
    """'direct' if the named tool's observation is retrieved content
    rather than a model's rendering of it; 'mediated' otherwise."""
    if not tool_name:
        return 'mediated'
    name = str(tool_name).strip().replace('_', '-')
    return 'direct' if name in _DIRECT_OBSERVATION_TOOLS else 'mediated'


def synthesis_only_claims(trace_rec: Dict[str, Any],
                          claims: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Claims whose every supporting step is a model's synthesis rather
    than a document.

    The justify renderer already computes this and says so out loud —
    "Nothing here read the underlying source." — but only when a human
    types `justify`. Nothing acted on it. On turn 2787 all nine claims
    quoted a single search-web synthesis, graded `probable` (never worse,
    so the suspect gate stood down), and went out unaudited. One of them,
    a version number the synthesis had invented, was wrong; the three
    claims with issue numbers were right. A verbatim quote against a
    model's summary proves the summary said it, nothing more.

    Deliberately not a downgrade to `suspect`. The grade is honest —
    these ARE probable — and lowering it would misreport a well-sourced
    summary as weak evidence. What changes is only whether the audit runs.
    """
    tool_of = {ref: str((tm or {}).get('tool') or '')
               for ref, tm in (trace_rec.get('tool_meta') or {}).items()
               if isinstance(tm, dict)}
    # Mediated AND listed, which is the distinction the renderer already
    # draws and this first shipped without. "Mediated" alone is every tool
    # that isn't a raw read, including `display`, whose observation is a
    # receipt — "OK: rendered to canvas (158 bytes)". There is no document
    # behind a receipt to go and read, so flagging it produced an audit
    # that told the agent to fetch "the page, the release notes, the issue"
    # to confirm her own canvas render: a 63-second fire, live on
    # 2026-08-17, and the same mistake as auditing a peer exchange
    # (06928964, "it web-searched an agent's own eyes").
    #
    # A bibliography is what makes the claim checkable elsewhere. search-web
    # records a query and its sources; a receipt records nothing.
    listed = {ref for ref, tm in (trace_rec.get('tool_meta') or {}).items()
              if isinstance(tm, dict) and any(
                  (e or {}).get('tool_metadata')
                  for e in (tm.get('meta') or []) if isinstance(e, dict))}
    mediated = {ref for ref, tool in tool_of.items()
                if observation_mediation(tool) == 'mediated'} & listed
    if not mediated:
        return []
    out = []
    for c in claims:
        refs = c.get('refs') or []
        # Unsupported claims are the suspect gate's business, not this one.
        if not refs or not c.get('quote'):
            continue
        if all(r in mediated for r in refs):
            out.append(c)
    return out


def grade_claim(c: Dict[str, Any]) -> str:
    """Deterministic ordinal grade for one attributed claim — plain code
    over grounding + validated taxonomy tags, per the caps in
    docs/justification-taxonomy.md. Untagged claims grade exactly as
    before tags existed (legacy records keep their current behavior)."""
    g = c.get('grounding')
    if g == 'model_prior':
        v = c.get('volatility')
        if v == 'stable':
            return 'probable'
        if v == 'volatile':
            return 'suspect'
        return 'unverified'
    if g == 'inferred':
        refs = c.get('refs') or []
        # Evidence-rooted inference starts at its premises' grade (flat
        # v1: recorded evidence reads probable); an inference citing no
        # evidence at all is prior-rooted.
        base = 'probable' if refs else 'unverified'
        inf = c.get('inference')
        if inf == 'negation-from-absence':
            adequacy = c.get('query_adequacy')
            cap = {'adequate': 'probable',
                   'inadequate': 'suspect'}.get(adequacy, 'unverified')
        else:
            cap = _INFERENCE_CAPS.get(inf, GRADES[0])   # absent = no cap
        return _worst(base, cap)
    # retrieved / memory / user_asserted / context: recorded evidence or
    # testimony — probable. (sourced needs quote x primary-source; the
    # source-grade tag is specified in docs/justification-taxonomy.md but
    # is not yet emitted by the attribution pass, so nothing reaches it.
    # aged-memory cap needs evidence dates; ditto. Mediation is surfaced
    # in the render rather than capped here — with every path already at
    # probable a cap would be inert, and the reader is who needs to know
    # that a quote matched a synthesis rather than a source.)
    return 'probable'


def _weakest_claim(claims: List[Dict[str, Any]]) -> Optional[int]:
    """Index of the worst-graded claim (first among ties), or None when
    nothing grades below probable — no point flagging a healthy trail."""
    worst_i, worst_rank = None, _GRADE_RANK['probable']
    for i, c in enumerate(claims):
        rank = _GRADE_RANK[grade_claim(c)]
        if rank > worst_rank:
            worst_i, worst_rank = i, rank
    return worst_i


def _audit_notes(claims: List[Dict[str, Any]],
                 record_has_quotes: bool,
                 mediated_refs: Optional[Set[str]] = None) -> List[str]:
    """Review keys for the taxonomy patterns present in this trail,
    strongest first, capped at three so the observation stays readable.
    Pure pattern → text; the checks themselves are the model's to run."""
    notes: List[str] = []
    med = mediated_refs or set()
    if med and all((c.get('grounding') == 'retrieved'
                    and any(r in med for r in (c.get('refs') or [])))
                   for c in claims):
        notes.append(
            "every claim rests on a model-synthesised observation — no "
            "source document was read here, so the whole trail is one "
            "model's account of its reading. If a claim is load-bearing, "
            "fetch the source it names and check it now.")
    if any(c.get('inference') == 'circular-support' for c in claims):
        notes.append(
            "a claim's support includes its own conclusion (circular). "
            "Strike the conclusion from the premises — if nothing "
            "remains, the claim is ungrounded; verify it independently.")
    if any(c.get('grounding') == 'model_prior' and
           c.get('volatility') != 'stable' for c in claims):
        notes.append(
            "model_prior claims rest on training data with a cutoff. If "
            "any such claim concerns a current or changeable fact, "
            "verify it with a tool now before affirming it; if "
            "verification contradicts the original reply, lead with the "
            "correction.")
    if any(c.get('inference') == 'negation-from-absence' for c in claims):
        notes.append(
            "a claim infers non-existence from not finding something. "
            "Check the probe: was it designed to find the thing if it "
            "existed (right venue, existence-shaped query)? If not, run "
            "a targeted existence probe now.")
    if record_has_quotes and any(
            c.get('grounding') == 'retrieved' and not c.get('quote')
            for c in claims):
        notes.append(
            "a retrieved claim has no verbatim quote — the "
            "paraphrase-drift signature. Re-check that the observation "
            "actually contains what the claim asserts.")
    if any(c.get('grounding') == 'inferred' and not c.get('refs')
           for c in claims):
        notes.append(
            "an inference cites no recorded evidence — its premises are "
            "unstated, usually background knowledge. Name them; they "
            "inherit the model_prior checks above.")
    return notes[:3]


_GROUNDING_KEY = (
    "Grounding key: retrieved = stated in a tool observation that turn; "
    "memory = recalled from a persisted memory note; user_asserted = the "
    "user's own words that turn; context = earlier conversation; inferred = "
    "derived by reasoning from the cited evidence; model_prior = background "
    "knowledge — nothing recorded that turn supports it. Quoted spans are "
    "verbatim excerpts machine-checked against the persisted tool "
    "observation at attribution time. Grades (sourced > probable > "
    "unverified > suspect) are reduced deterministically from grounding "
    "plus tags — see docs/justification-taxonomy.md.\n"
    # The reducer's own account of its ceiling. Without it a bare grade
    # begs "why?" and gets an invented answer: at turn 2408 the reply
    # explained four probable grades as caused by the observation being a
    # model synthesis, which is false — mediation is not a cap, and those
    # claims would grade probable against a document read byte-for-byte.
    # Prohibiting the explanation has not worked (three catalog attempts);
    # stating the true one leaves nothing to invent. Update this text if
    # source-grade ever starts being emitted.
    "Ceiling: no claim can reach `sourced` yet — that grade requires a "
    "source-grade tag the attribution pass does not emit, so every "
    "recorded-evidence claim (retrieved / memory / user_asserted / "
    "context) grades `probable` flat. Mediation and a missing quote are "
    "reported above but are NOT applied as caps: they lowered nothing "
    "here, and a claim quoted against a synthesis grades the same as one "
    "read from the source. Anything below `probable` was lowered only by "
    "the grounding and tags shown on that claim.")


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

    # Which $stepN observations were written by a model rather than
    # retrieved. A verbatim quote against one of those proves the
    # synthesis said it, not that any source did — the reader has to be
    # able to tell those apart.
    tool_of: Dict[str, str] = {
        ref: str((tm or {}).get('tool') or '')
        for ref, tm in (trace_rec.get('tool_meta') or {}).items()
        if isinstance(tm, dict)}
    mediated_refs = {ref for ref, tool in tool_of.items()
                     if observation_mediation(tool) == 'mediated'}
    # Which refs recorded structured metadata (a query, sources). Every
    # step is named now, so "mediated" alone no longer implies there is a
    # bibliography behind it: a named-but-empty step is a model's own text
    # over material already in the turn, with nothing to cite.
    listed_refs = {
        ref for ref, tm in (trace_rec.get('tool_meta') or {}).items()
        if isinstance(tm, dict) and any(
            (e or {}).get('tool_metadata')
            for e in (tm.get('meta') or []) if isinstance(e, dict))}

    lines.append(f"\nClaims ({len(claims)}):")
    cited: List[str] = []
    for i, c in enumerate(claims, 1):
        refs = c.get('refs') or []
        ref_txt = f" ← {', '.join(refs)}" if refs else ""
        tag = c.get('volatility') or c.get('inference') or ''
        if tag and c.get('query_adequacy'):
            tag += f", {c['query_adequacy']} probe"
        tag_txt = f" ({tag})" if tag else ""
        lines.append(f"{i}. [{c.get('grounding')}{tag_txt}{ref_txt} | "
                     f"{grade_claim(c)}] {c.get('claim')}")
        if c.get('quote'):
            lines.append(f'   quote: "{_clip(c["quote"])}"')
            if any(r in mediated_refs for r in refs):
                med = sorted({tool_of[r] for r in refs if r in mediated_refs})
                over = ("a model's synthesis of the sources it listed"
                        if any(r in listed_refs for r in refs) else
                        "a model's own text over material already in this "
                        "turn, with no source outside it")
                lines.append(
                    f"   NB: quoted against {', '.join(med)} output, which is "
                    f"{over} — the span is verbatim from that, not from a "
                    f"source document. Nothing here read the underlying "
                    f"source.")
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
            # A step can be named without carrying sources (every built-in
            # is: process_text, recall, justify…). Say which case it is —
            # "the sources below" must not head an empty list.
            entries = [e for e in (tm.get('meta') or []) if isinstance(e, dict)]
            listed = ref in listed_refs
            if ref not in mediated_refs:
                med = ""
            elif listed:
                med = (" [model-synthesised observation; the sources below "
                       "were listed by that model, not read here]")
            else:
                med = (" [model-synthesised observation — this tool's own "
                       "output over what was already in the turn; no source "
                       "outside the conversation was read for it]")
            ev.append(f"{ref} — {tm.get('tool')}{med}" + (":" if listed else ""))
            for entry in entries:
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
                # source_turn_seq has been written on every memory (note
                # property + sidecar) since memories existed and never
                # rendered. It is the one hop that turns "recalled from a
                # note" into a followable trail: the note came from a turn,
                # and that turn has a trail of its own. Naming the number
                # is enough — the user can ask for it.
                #
                # But only when the number still resolves. Seqs restarted
                # at 1 each session until seeding landed, so an old
                # memory's seq now points at an unrelated recent turn
                # (live: Note_4191, written 2026-07-14 "from turn 14",
                # whose seq-14 record is a 2026-07-17 turn). Same date =
                # the same turn, since reflection writes the memory right
                # after the reply; a midnight-boundary miss degrades to
                # "unresolvable", which is the safe direction.
                src_turn = rec.get('source_turn_seq')
                from_turn = ""
                if src_turn:
                    origin = latest_turn_for_source(
                        memory_dir, trace_rec.get('source'), src_turn)
                    same_day = (origin is not None
                                and str(origin.get('ts'))[:10]
                                == str(rec.get('ts'))[:10])
                    from_turn = (
                        f" (written from turn {src_turn} — justify "
                        f"{src_turn} for that turn's own trail)" if same_day
                        else f" (written from turn {src_turn}, an earlier "
                             f"session's numbering that no longer resolves — "
                             f"do not justify that number)")
                ev.append(f"{ref} — memory written {date}: "
                          f"{_clip(rec.get('text') or '')!r}{from_turn}")
            else:
                ev.append(f"{ref} — recalled memory "
                          f"(not found in memories.jsonl sidecar)")
        else:
            ev.append(f"{ref} — unrecognized ref form")
    if ev:
        lines.append("\nEvidence:")
        lines.extend(ev)

    # Source composition — counts only, no quality judgment. The split
    # that matters is read-here vs named-by-a-model: the authority of a
    # source says nothing about a claim's support until someone opens it.
    read_here: Set[str] = set()
    named_only: Set[str] = set()
    for ref in cited:
        tm = tool_meta.get(ref)
        if not isinstance(tm, dict):
            continue
        bucket = named_only if ref in mediated_refs else read_here
        for entry in (tm.get('meta') or []):
            for s in ((entry or {}).get('tool_metadata') or {}).get('sources') or []:
                if isinstance(s, dict) and (s.get('domain') or s.get('url')):
                    bucket.add(str(s.get('domain') or s.get('url')))
    named_only -= read_here
    if read_here or named_only:
        parts = [f"{len(read_here) + len(named_only)} distinct source "
                 f"domain(s)"]
        parts.append(f"{len(read_here)} read here")
        parts.append(f"{len(named_only)} named in a model-synthesised "
                     f"observation but not opened")
        lines.append("\nSource composition: " + ", ".join(parts))
        if named_only:
            lines.append("  not opened: " + ", ".join(sorted(named_only)))

    profile: Dict[str, int] = {}
    for c in claims:
        g = str(c.get('grounding'))
        profile[g] = profile.get(g, 0) + 1
    lines.append("\nGrounding profile: " + ", ".join(
        f"{n} {g}" for g, n in
        sorted(profile.items(), key=lambda kv: (-kv[1], kv[0]))))
    weak_i = _weakest_claim(claims)
    if weak_i is not None:
        c = claims[weak_i]
        why = c.get('volatility') or c.get('inference') or 'untagged'
        lines.append(f"Weakest link: claim {weak_i + 1} "
                     f"({grade_claim(c)} — {c.get('grounding')}, {why}).")
    # Structural audit triggers: review keys for the taxonomy patterns
    # this trail exhibits, placed in the observation so the model reads
    # them at exactly the moment it decides whether the trail alone
    # answers the user.
    for note in _audit_notes(claims,
                             any(c.get('quote') for c in claims),
                             mediated_refs):
        lines.append(f"Audit note: {note}")
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

    def _extract_and_log_claims(self, turn_seq: int,
                                spawn_verification: bool = False) -> None:
        """Attribute the turn's reply claims and append to claims.jsonl.
        Best-effort; reads the PERSISTED trace record so attribution
        audits exactly what is durable, not in-memory state.

        spawn_verification is set only by the post-turn path: suspect
        grades then spawn a background verification concern. The justify
        on-demand path leaves it off — the justify turn itself performs
        the audit, so a background double-check would duplicate it."""
        record = self._load_trace_record(turn_seq)
        if record is None:
            logger.warning(
                f"[{self.character_name}] claims: no trace record for "
                f"turn_seq {turn_seq}")
            return
        reply = str(record.get('raw_response') or '').strip()
        if not reply or reply == '(no reply)':
            return

        # Already attributed? `justify` attributes on demand, so when the
        # user asks immediately the post-turn job arrives to find the work
        # done. Re-running costs another 11-20s call and appends a
        # duplicate line. Match on reply_sha1 and source, not turn_seq
        # alone — a seq can repeat across old sessions, and a stale record
        # for a different reply must not suppress a real pass.
        existing = claims_for_turn(self._memory_dir(), turn_seq)
        if (existing is not None
                and not existing.get('incomplete')
                and existing.get('source') == record.get('source')
                and existing.get('reply_sha1') == hashlib.sha1(
                    reply.encode('utf-8')).hexdigest()):
            logger.info(
                f"[{self.character_name}] claims: turn {turn_seq} already "
                f"attributed ({len(existing.get('claims') or [])} claim(s)); "
                f"skipping re-attribution")
            if spawn_verification:
                try:
                    self._maybe_spawn_suspect_verification(
                        record, existing.get('claims') or [])
                except Exception as e:
                    logger.warning(
                        f"[{self.character_name}] suspect-verification spawn "
                        f"failed for turn {turn_seq}: {e}")
            return

        # Budget comes from the same floor table _make_llm_callable uses,
        # so the two cannot drift. This call reaches self.backend.chat
        # directly and so never got that floor: the old literal 1600 cut
        # the JSON mid-object on any reply with many quoted claims and the
        # whole pass was dropped (live: turns 2393/2396, three
        # finish=length failures, no record written).
        floor = (self._PROFILE_TOKEN_FLOOR_CLOUD if self.backend.is_cloud
                 else self._PROFILE_TOKEN_FLOOR_LOCAL)
        budget = max(1600, int(floor.get('none', 0)))
        attempts = {'n': 0}

        def llm_chat(messages):
            # Retry once at double budget when the backend reports it ran
            # out of room. Lives here rather than in attribute_claims so
            # that function stays pure and offline-runnable.
            attempts['n'] += 1
            cap = budget * 2 if attempts['n'] > 1 else budget
            return self.backend.chat(messages, max_tokens=cap,
                                     cot_profile='none')

        status: Dict[str, Any] = {}
        claims = attribute_claims(record, llm_chat,
                                  character_name=self.character_name,
                                  status=status)
        # Retry whenever the first pass ran out of room — whether it was
        # unsalvageable (claims is None) or salvaged into a partial. A
        # partial beats nothing, but a complete pass beats a partial, so
        # the bigger attempt wins if it produces one.
        ran_out = (status.get('truncated')
                   or (claims is None
                       and self.backend.last_finish_reason in ('length',
                                                               'max_tokens')))
        if ran_out:
            logger.info(
                f"[{self.character_name}] claims: retrying turn {turn_seq} "
                f"at {budget * 2} tokens after a truncated first pass")
            retry_status: Dict[str, Any] = {}
            retry = attribute_claims(record, llm_chat,
                                     character_name=self.character_name,
                                     status=retry_status)
            # Keep the retry only if it is genuinely better: complete when
            # the first was partial, or non-empty when the first was None.
            if retry is not None and (claims is None
                                      or not retry_status.get('truncated')
                                      or len(retry) > len(claims)):
                claims, status = retry, retry_status
        if claims is None:
            # Nothing durable to write. Never write claims: [] here — that
            # is a real finding ("no checkable claims") and must not be
            # manufactured by a failure.
            logger.warning(
                f"[{self.character_name}] claims: attribution produced no "
                f"record for turn {turn_seq} "
                f"({status.get('error') or 'unknown'}; "
                f"finish={self.backend.last_finish_reason}, "
                f"attempts={attempts['n']})")
            return
        from utils.file_utils import append_jsonl
        rec: Dict[str, Any] = {
            'turn_seq': turn_seq,
            'source': record.get('source'),
            'reply_sha1': hashlib.sha1(reply.encode('utf-8')).hexdigest(),
            'claims': claims,
        }
        if status.get('truncated'):
            rec['incomplete'] = {'reason': 'length',
                                 'recovered': int(status.get('salvaged') or 0)}
        append_jsonl(self._claims_log_path(), rec, character=self.character_name)
        logger.info(
            f"[{self.character_name}] claims: turn {turn_seq} attributed "
            f"{len(claims)} claim(s)"
            + (" (PARTIAL — output was truncated)"
               if status.get('truncated') else ""))
        if spawn_verification:
            try:
                self._maybe_spawn_suspect_verification(record, claims)
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] suspect-verification spawn "
                    f"failed for turn {turn_seq}: {e}")

    def _maybe_spawn_suspect_verification(self, record: Dict[str, Any],
                                          claims: List[Dict[str, Any]]
                                          ) -> Optional[str]:
        """Stage 5: answer now, audit behind. When post-turn grading finds
        suspect claims in a delivered reply, spawn a one-shot agent_concern
        (same vehicle as user-yield continuations) that verifies them in
        the background and posts a correction ONLY if one is refuted.
        Silent on confirmed — the success path leaves no message.

        No loop risk: verification fires are autonomous turns, which get
        no claim pass, so they can never re-trigger this. Returns the new
        concern id, or None when nothing warranted a spawn."""
        if not getattr(self, '_autonomy_enabled', False):
            return None
        if record.get('autonomous'):
            return None
        # User-facing replies only. A peer exchange is the wrong audience
        # for this: the correction path posts to the conversation, and the
        # claims in a message to a partner are typically first-person
        # reports of what this agent just did or saw. Restated on a later
        # turn those carry no in-turn evidence, so the attributor grades
        # them model_prior/volatile and this fired a web search against an
        # agent's own eyes — observed 2026-08-12, twice in one run, on
        # "I've spotted a marker at (62.1, 101.6)". The real repair is a
        # `context turn:N` ref form so a prior-turn observation grades as
        # context; until that exists, don't audit peer traffic.
        if str(record.get('source') or '') in (getattr(self, '_peers', None) or []):
            return None
        suspects = [c for c in claims if grade_claim(c) == 'suspect']
        # Second trigger: claims that grade fine but rest entirely on a
        # model's summary of sources nobody opened. The suspect gate can
        # never catch these — they are `probable`, which is the correct
        # grade — so before this they went out unaudited. See
        # synthesis_only_claims for the turn that cost.
        unread = [c for c in synthesis_only_claims(record, claims)
                  if c not in suspects]
        if not suspects and not unread:
            return None
        source = str(record.get('source') or 'User')
        seq = record.get('turn_seq')
        user_input = ' '.join(str(record.get('user_input') or '').split())
        reply = ' '.join(str(record.get('raw_response') or '').split())
        claim_lines = '\n'.join(
            f"{i}. {c.get('claim')} "
            f"[{c.get('grounding')}, {c.get('volatility') or c.get('inference')}]"
            for i, c in enumerate(suspects, 1))
        unread_lines = '\n'.join(
            f"{i}. {c.get('claim')}\n   quoted from a summary: "
            f"\"{_clip(str(c.get('quote') or ''))}\""
            for i, c in enumerate(unread, 1))
        suspect_section = (
            f"These claims from that reply graded suspect — volatile "
            f"background knowledge or weak inference, unverified at answer "
            f"time:\n\n{claim_lines}\n\n" if suspects else "")
        # A quote against a summary proves the summary said it. The probe
        # for these is not "search again" — that asks the same model the
        # same question and gets the same answer, which is how an invented
        # version number survived its own verification. Go to the document.
        unread_section = (
            f"These claims are not weakly grounded — they are quoted "
            f"accurately — but every source behind them is a model's "
            f"summary that nobody opened. The quote proves the summary "
            f"said it, not that any document does:\n\n{unread_lines}\n\n"
            f"For these, read the underlying source itself — fetch the "
            f"page, the release notes, the issue, the file. Searching "
            f"again re-asks the summarizer that produced the claim and "
            f"will agree with itself. Check the load-bearing specifics "
            f"first: a number, a version, a date or a name the user might "
            f"act on, ahead of anything already carrying its own "
            f"citation.\n\n" if unread else "")
        instruction = (
            f"Background verification (auto-spawned after post-turn claim "
            f"grading of my reply to {source}, turn {seq}). "
            f"{suspect_section}{unread_section}"
            f"The exchange, for context:\n"
            f"They said: {user_input[:300]}\n"
            f"I replied: {reply[:500]}\n\n"
            # Naming search-web/fetch-text here sent every audit to the
            # internet regardless of what the claim was about. Observed
            # 2026-08-16: "I'm on a slope" and "Jill isn't on a slope" —
            # both answerable by looking, in one step — were dispatched as
            # web searches. Say what a good probe IS and let the agent pick
            # from the tools it actually has.
            f"Verify each claim above now — design each probe to test the "
            f"claim directly, using whichever of your tools can actually "
            f"settle it. Match the probe to the kind of claim: something "
            f"about your own situation, surroundings or state is settled "
            f"by observing, not by searching; something about the outside "
            f"world needs a source. "
            f"If verification REFUTES any claim, post a brief correction "
            f"to the conversation: lead with the correction, cite what "
            f"you found now, and do not defend the original reply. "
            f"Silent if every claim is confirmed or the checks are "
            f"inconclusive — this verification reports nothing on "
            f"success.")
        from chat.concerns import _AGENT_CONCERN_FIRE_THRESHOLD
        new_id = self._add_agent_concern(
            text=(f"verify suspect claims from my reply to {source}"
                  if suspects else
                  f"read the sources behind my reply to {source}"),
            entity=source, provenance='inferred', seed=False, name='',
            rhythm_hours=1, rhythm_source='urgency',
            instruction=instruction,
            skip_recurrence=True,
            category='one_shot',
            extra_properties={
                # Prime AT threshold so the next tick fires it, matching
                # the yield/successor spawn paths in concerns.py.
                'activation': _AGENT_CONCERN_FIRE_THRESHOLD,
                # System-spawned: reflection must not be able to close
                # this. Observed live 2026-08-13 — a verification concern
                # was spawned and closed by the very next reflection 17s
                # later, so four ungrounded claims were never checked.
                # The audit trail looked healthy while the audit never ran.
                'system_spawned': True,
            },
        )
        if new_id:
            logger.info(
                f"[{self.character_name}] spawned suspect-verification "
                f"concern {new_id} for turn {seq} "
                f"({len(suspects)} suspect claim(s))")
            self._write_autonomy_event({
                'event': 'successor_spawned',
                'parent_concern_id': None,
                'successor_concern_id': new_id,
                'via': 'suspect_verification',
            })
        return new_id

    def _run_justify(self, source: str,
                     turn_seq: Optional[int] = None) -> str:
        """Backend for the ReAct `justify` tool: render the provenance
        trail of a reply to this conversation — the most recent, or the
        one the user named by turn number. Read-only, and the observation
        is built from persisted records alone.

        That last property is what the old most-recent-only rule was
        really protecting: a trail must never be reconstructed from recall
        or conversation memory (paraphrase-as-provenance, observed live at
        turn 2219). Neither attribute_claims nor render_justification can
        reach anything but the records, so auditing by turn number is
        exactly as laundering-proof as auditing the latest reply.
        """
        memory_dir = self._memory_dir()
        trace_rec = latest_turn_for_source(memory_dir, source, turn_seq)
        if trace_rec is None:
            if turn_seq is not None:
                return (f"EMPTY: no reply to this conversation with turn "
                        f"number {turn_seq}. Check the number against the "
                        f"one shown beside the reply. Do NOT substitute a "
                        f"different turn and do NOT reconstruct a trail "
                        f"from memory.")
            return ("EMPTY: no prior reply to this conversation in the "
                    "reasoning trace — nothing to justify yet.")
        seq = trace_rec.get('turn_seq')
        claims_rec = claims_for_turn(memory_dir, seq)
        # Guard the join. reply_sha1 has been written since attribution
        # existed and never once read; it is what catches a claims record
        # belonging to a different reply under the same seq (seqs 1..50
        # repeat across pre-seeding sessions). A stale record is worse
        # than none: it renders a real, well-formed trail for the wrong
        # text. Treat as absent and re-attribute.
        if claims_rec is not None:
            reply_txt = str(trace_rec.get('raw_response') or '').strip()
            expected = hashlib.sha1(reply_txt.encode('utf-8')).hexdigest()
            recorded = claims_rec.get('reply_sha1')
            if (claims_rec.get('source') != trace_rec.get('source')
                    or (recorded is not None and recorded != expected)):
                logger.warning(
                    f"[{self.character_name}] justify: claims record for turn "
                    f"{seq} does not match that turn's reply "
                    f"(source/sha1 mismatch); re-attributing")
                claims_rec = None
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
        elif claims_rec.get('incomplete'):
            # A partial record from a truncated pass. Try once for a
            # complete one; last-match-wins means the better result stands.
            self._extract_and_log_claims(int(seq))
            claims_rec = claims_for_turn(memory_dir, seq) or claims_rec
        if claims_rec is None:
            # ERROR, not EMPTY: per the observation convention EMPTY means
            # "ran clean, nothing to show", which reads as a retryable
            # wait and produced the retry-then-apologise loop at turn 2393.
            return (f"ERROR: provenance attribution for turn {seq} failed, so "
                    f"no trail was recorded. Tell the user plainly that the "
                    f"audit for that reply could not be produced. Do NOT "
                    f"reconstruct a trail from memory or recall, and do NOT "
                    f"say the record is still being written — this tool "
                    f"attributes on demand, so there is nothing to wait for "
                    f"and calling it again unchanged will fail the same way.")
        out = render_justification(claims_rec, trace_rec, memory_dir)
        inc = claims_rec.get('incomplete')
        if isinstance(inc, dict):
            out = (f"PARTIAL TRAIL — attribution was cut off after "
                   f"{inc.get('recovered')} claim(s); anything the reply "
                   f"asserted beyond that point is unattributed here. Present "
                   f"it as covering only part of the reply.\n\n" + out)
        return "OK: " + out
