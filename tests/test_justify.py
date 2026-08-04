"""Tests for the `justify` read path (chat.claims): deterministic
rendering of a turn's attributed claims + resolved evidence.

Synthetic records in tmp_path only — never touches live world state.
"""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from chat.claims import (
    ClaimsMixin,
    attribute_claims,
    claims_for_turn,
    latest_turn_for_source,
    note_sidecar_lookup,
    render_justification,
)


def _write_jsonl(path: Path, records):
    with open(path, "w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


def _memory_dir(tmp_path: Path) -> Path:
    d = tmp_path / "memory"
    d.mkdir()
    return d


TRACE = [
    # Older reply to User (superseded by turn 12 for "latest").
    {"turn_seq": 10, "source": "User", "autonomous": False,
     "user_input": "old question", "raw_response": "old answer",
     "working_log": "$step1 OK: something"},
    # The reply to justify: one search step with structured sources.
    {"turn_seq": 12, "source": "User", "autonomous": False,
     "user_input": "Can I copy/paste blueprints?",
     "raw_response": "No direct copy; use the Designer.",
     "working_log": "$step1:\nOK: the wiki states there is no direct "
                    "copy/paste of world objects; the Designer tool is "
                    "the supported path.\n",
     "tool_meta": {"$step1": {"tool": "search-web", "meta": [
         {"source_skill": "search-web", "tool_metadata": {
             "query": "blueprint copy paste",
             "sources": [{"url": "https://wiki.example/bp",
                          "domain": "wiki.example",
                          "title": "Blueprint - Wiki",
                          "excerpt": "..."}]}}]}}},
    # Interleaved autonomous fires after the user turn.
    {"turn_seq": 13, "source": "Jill", "autonomous": True,
     "user_input": "A concern of mine has fired: ...",
     "raw_response": "digest", "working_log": ""},
    {"turn_seq": 14, "source": "Jill", "autonomous": True,
     "user_input": "A concern of mine has fired: ...",
     "raw_response": "digest2", "working_log": ""},
]

CLAIMS = [
    {"turn_seq": 12, "source": "User", "reply_sha1": "ab" * 20,
     "claims": [
         {"claim": "There is no direct world copy/paste.",
          "grounding": "retrieved", "refs": ["$step1"],
          "quote": "there is no direct copy/paste of world objects"},
         {"claim": "The user asked about blueprints.",
          "grounding": "user_asserted", "refs": ["user_input"]},
         {"claim": "The user prefers modular builds.",
          "grounding": "memory", "refs": ["Note_7"]},
         {"claim": "A mod would be needed for bulk capture.",
          "grounding": "inferred", "refs": ["$step1"]},
         {"claim": "The game engine is Unreal.",
          "grounding": "model_prior", "refs": []},
     ]},
]

MEMORIES = [
    {"event": "write", "note_id": "Note_7",
     "text": "User prefers modular factory builds.",
     "ts": "2026-07-27T19:45:38", "character": "Jill"},
]


def _populate(md: Path):
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    _write_jsonl(md / "claims.jsonl", CLAIMS)
    _write_jsonl(md / "memories.jsonl", MEMORIES)


# ── lookups ────────────────────────────────────────────────────────────

def test_latest_turn_for_source_skips_autonomous(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    rec = latest_turn_for_source(md, "User")
    assert rec["turn_seq"] == 12  # not 13/14 (source=Jill), not 10 (older)
    assert latest_turn_for_source(md, "Nobody") is None


def test_claims_for_turn_last_match_wins(tmp_path):
    md = _memory_dir(tmp_path)
    reattributed = CLAIMS + [{"turn_seq": 12, "reply_sha1": "cd" * 20,
                              "claims": []}]
    _write_jsonl(md / "claims.jsonl", reattributed)
    assert claims_for_turn(md, 12)["reply_sha1"] == "cd" * 20
    assert claims_for_turn(md, 99) is None


def test_note_sidecar_lookup(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    assert note_sidecar_lookup(md, "Note_7")["text"].startswith("User prefers")
    assert note_sidecar_lookup(md, "Note_999") is None


# ── rendering ──────────────────────────────────────────────────────────

def test_render_justification_full_trail(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    out = render_justification(CLAIMS[0], TRACE[1], md)
    # Every claim present with its grounding tag and reduced grade.
    assert ("[retrieved ← $step1 | probable] "
            "There is no direct world copy/paste.") in out
    assert "[model_prior | unverified] The game engine is Unreal." in out
    # The verified quote renders under its claim.
    assert 'quote: "there is no direct copy/paste of world objects"' in out
    # Evidence resolved: search source URL, query, note text with date.
    assert "https://wiki.example/bp" in out
    assert "query: blueprint copy paste" in out
    assert "Note_7 — memory written 2026-07-27" in out
    assert "User prefers modular factory builds." in out
    assert "user_input — the user's own words" in out
    # Grounding profile summarizes the validated grounding counts; the
    # untagged model_prior claim is the weakest link and triggers the
    # volatility audit note.
    assert ("Grounding profile: 1 inferred, 1 memory, 1 model_prior, "
            "1 retrieved, 1 user_asserted") in out
    assert "Weakest link: claim 5 (unverified — model_prior, untagged)." in out
    assert "Audit note: model_prior claims rest on training data" in out
    # Grounding key present.
    assert "model_prior = background" in out


def test_render_justification_no_audit_note_without_prior(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    claims_rec = {"turn_seq": 12, "claims": [
        {"claim": "There is no direct world copy/paste.",
         "grounding": "retrieved", "refs": ["$step1"]},
    ]}
    out = render_justification(claims_rec, TRACE[1], md)
    assert "Grounding profile: 1 retrieved" in out
    assert "Audit note:" not in out
    assert "Weakest link" not in out  # nothing below probable


def test_render_justification_no_claims(tmp_path):
    md = _memory_dir(tmp_path)
    out = render_justification({"turn_seq": 5, "claims": []},
                               {"turn_seq": 5, "user_input": "hi"}, md)
    assert "no checkable factual claims" in out


def test_render_justification_missing_note_and_meta(tmp_path):
    md = _memory_dir(tmp_path)  # no sidecars at all
    claims_rec = {"turn_seq": 5, "claims": [
        {"claim": "x", "grounding": "retrieved", "refs": ["$step1"]},
        {"claim": "y", "grounding": "memory", "refs": ["Note_1"]},
    ]}
    out = render_justification(claims_rec, {"turn_seq": 5,
                                            "user_input": "q"}, md)
    assert "no structured metadata recorded" in out
    assert "not found in memories.jsonl sidecar" in out


# ── quote verbatim check ───────────────────────────────────────────────

def _attribution_record():
    return {"turn_seq": 12, "source": "User",
            "user_input": "Can I copy/paste blueprints?",
            "raw_response": "No direct copy; use the Designer.",
            "working_log": "$step1:\nOK: the wiki states there is no\n"
                           "direct copy/paste of world objects; the\n"
                           "Designer tool is the supported path.\n",
            "recall_hits": []}


def _canned_llm(claims):
    return lambda messages: json.dumps({"claims": claims})


def test_attribute_claims_keeps_verbatim_quote():
    # Quote spans a line wrap in the observation — whitespace-normalized
    # matching must still accept it.
    out = attribute_claims(_attribution_record(), _canned_llm([
        {"claim": "There is no direct copy/paste.",
         "grounding": "retrieved", "refs": ["$step1"],
         "quote": "there is no direct copy/paste of world objects"},
    ]))
    assert out[0]["quote"] == "there is no direct copy/paste of world objects"


def test_attribute_claims_drops_synthesized_quote():
    out = attribute_claims(_attribution_record(), _canned_llm([
        {"claim": "A mod is needed for bulk capture.",
         "grounding": "retrieved", "refs": ["$step1"],
         "quote": "bulk capture requires installing a mod"},
    ]))
    # Claim survives; the unverifiable quote does not.
    assert out[0]["claim"] == "A mod is needed for bulk capture."
    assert "quote" not in out[0]


def test_attribute_claims_no_quote_field_unchanged():
    out = attribute_claims(_attribution_record(), _canned_llm([
        {"claim": "The user asked about blueprints.",
         "grounding": "user_asserted", "refs": ["user_input"]},
    ]))
    assert out == [{"claim": "The user asked about blueprints.",
                    "grounding": "user_asserted", "refs": ["user_input"]}]


# ── reducer goldens: the incident rows from the taxonomy caps table ────

def test_reducer_spacex_shape(tmp_path):
    """model_prior x volatile -> suspect; prior-rooted inference ->
    unverified; weakest link is the volatile prior."""
    md = _memory_dir(tmp_path)
    claims_rec = {"turn_seq": 5, "claims": [
        {"claim": "SpaceX is a private company.",
         "grounding": "model_prior", "refs": [], "volatility": "volatile"},
        {"claim": "There is no public share price to track.",
         "grounding": "inferred", "refs": [], "inference": "deduction"},
    ]}
    out = render_justification(claims_rec, {"turn_seq": 5,
                                            "user_input": "q"}, md)
    assert ("[model_prior (volatile) | suspect] "
            "SpaceX is a private company.") in out
    assert ("[inferred (deduction) | unverified] "
            "There is no public share price to track.") in out
    assert "Weakest link: claim 1 (suspect — model_prior, volatile)." in out
    assert "Audit note: model_prior claims rest on training data" in out
    assert "cites no recorded evidence" in out  # hidden-premise key


def test_reducer_qwen_shape(tmp_path):
    """negation-from-absence x inadequate probe -> suspect even though
    the inference cites recorded evidence."""
    md = _memory_dir(tmp_path)
    claims_rec = {"turn_seq": 5, "claims": [
        {"claim": "Qwen3.8-27B is unavailable.",
         "grounding": "inferred", "refs": ["$step1"],
         "inference": "negation-from-absence",
         "query_adequacy": "inadequate"},
    ]}
    out = render_justification(claims_rec, {"turn_seq": 5,
                                            "user_input": "q"}, md)
    assert ("[inferred (negation-from-absence, inadequate probe) "
            "← $step1 | suspect] Qwen3.8-27B is unavailable.") in out
    assert "Audit note: a claim infers non-existence" in out


def test_reducer_stable_prior_stays_quiet(tmp_path):
    """model_prior x stable -> probable; no weakest link, no volatility
    note — the false-positive direction the tags exist to fix."""
    md = _memory_dir(tmp_path)
    claims_rec = {"turn_seq": 5, "claims": [
        {"claim": "Paris is the capital of France.",
         "grounding": "model_prior", "refs": [], "volatility": "stable"},
    ]}
    out = render_justification(claims_rec, {"turn_seq": 5,
                                            "user_input": "q"}, md)
    assert ("[model_prior (stable) | probable] "
            "Paris is the capital of France.") in out
    assert "Weakest link" not in out
    assert "Audit note:" not in out


def test_reducer_circular_and_adequacy_default():
    from chat.claims import grade_claim
    assert grade_claim({"claim": "x", "grounding": "inferred",
                        "refs": ["$step1"],
                        "inference": "circular-support"}) == "suspect"
    # negation-from-absence with UNKNOWN adequacy: middle ground.
    assert grade_claim({"claim": "x", "grounding": "inferred",
                        "refs": ["$step1"],
                        "inference": "negation-from-absence"}) == "unverified"
    assert grade_claim({"claim": "x", "grounding": "inferred",
                        "refs": ["$step1"],
                        "inference": "entailment"}) == "probable"


# ── tag validation at attribution ──────────────────────────────────────

def test_attribute_claims_tag_validation():
    out = attribute_claims(_attribution_record(), _canned_llm([
        # Valid: volatility on model_prior.
        {"claim": "a", "grounding": "model_prior", "refs": [],
         "volatility": "volatile"},
        # Invalid value: dropped, claim kept.
        {"claim": "b", "grounding": "model_prior", "refs": [],
         "volatility": "very-volatile"},
        # Misplaced: volatility is not defined for retrieved.
        {"claim": "c", "grounding": "retrieved", "refs": ["$step1"],
         "volatility": "volatile", "polarity": "presence"},
        # query_adequacy only rides on negation-from-absence.
        {"claim": "d", "grounding": "inferred", "refs": ["$step1"],
         "inference": "deduction", "query_adequacy": "adequate"},
    ]))
    assert out[0]["volatility"] == "volatile"
    assert "volatility" not in out[1]
    assert "volatility" not in out[2] and out[2]["polarity"] == "presence"
    assert out[3]["inference"] == "deduction"
    assert "query_adequacy" not in out[3]


# ── _run_justify observation protocol ──────────────────────────────────

class _Backend:
    """Canned attribution backend for the on-demand path."""

    def __init__(self, raise_instead=False):
        self.raise_instead = raise_instead
        self.calls = 0

    def chat(self, messages, **kwargs):
        self.calls += 1
        if self.raise_instead:
            raise RuntimeError("backend down")
        return json.dumps({"claims": [
            {"claim": "There is no direct world copy/paste.",
             "grounding": "retrieved", "refs": ["$step1"]}]})


class _Stub(ClaimsMixin):
    character_name = "TestJill"

    def __init__(self, md: Path, backend=None, autonomy=False):
        self._md = Path(md)
        self.backend = backend or _Backend()
        self._autonomy_enabled = autonomy
        self.spawned = []          # (text, instruction, kwargs)
        self.autonomy_events = []

    def _memory_dir(self) -> Path:
        return self._md

    def _reasoning_trace_path(self) -> Path:
        return self._md / "reasoning_trace.jsonl"

    def _add_agent_concern(self, text, instruction=None, **kwargs):
        self.spawned.append((text, instruction, kwargs))
        return f"Note_test_{len(self.spawned)}"

    def _write_autonomy_event(self, event):
        self.autonomy_events.append(event)


def test_run_justify_ok(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    stub = _Stub(md)
    obs = stub._run_justify("User")
    assert obs.startswith("OK: Provenance trail")
    assert "turn 12" in obs
    assert stub.backend.calls == 0  # claims already on disk; no LLM call


def test_run_justify_no_trace(tmp_path):
    md = _memory_dir(tmp_path)
    obs = _Stub(md)._run_justify("User")
    assert obs.startswith("EMPTY: no prior reply")


def test_run_justify_attributes_on_demand(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)  # no claims.jsonl
    stub = _Stub(md)
    obs = stub._run_justify("User")
    assert stub.backend.calls == 1
    assert obs.startswith("OK: Provenance trail")
    assert ("[retrieved ← $step1 | probable] "
            "There is no direct world copy/paste.") in obs
    # The on-demand pass persisted the record (durable, not one-shot).
    assert claims_for_turn(md, 12) is not None


def test_run_justify_attribution_fails(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    obs = _Stub(md, _Backend(raise_instead=True))._run_justify("User")
    assert obs.startswith("EMPTY: claim attribution for turn 12")


# ── Stage 5: suspect-verification spawn ────────────────────────────────

_SUSPECT_RECORD = {
    "turn_seq": 30, "source": "User", "autonomous": False,
    "user_input": "How much will the share price drop on Thursday?",
    "raw_response": "It is a private company; nothing to track.",
}

_SUSPECT_CLAIMS = [
    {"claim": "SpaceX is a private company.",
     "grounding": "model_prior", "refs": [], "volatility": "volatile"},
    {"claim": "There is no public share price to track.",
     "grounding": "inferred", "refs": [], "inference": "deduction"},
]


def test_suspect_spawn_fires_on_suspect(tmp_path):
    stub = _Stub(_memory_dir(tmp_path), autonomy=True)
    new_id = stub._maybe_spawn_suspect_verification(
        _SUSPECT_RECORD, _SUSPECT_CLAIMS)
    assert new_id is not None
    (text, instruction, kwargs), = stub.spawned
    # The one suspect claim is named; the unverified one is not.
    assert "SpaceX is a private company." in instruction
    assert "There is no public share price" not in instruction
    assert "lead with the correction" in instruction
    assert "Silent" in instruction
    assert kwargs["category"] == "one_shot"
    assert kwargs["skip_recurrence"] is True
    assert stub.autonomy_events[0]["via"] == "suspect_verification"


def test_suspect_spawn_quiet_without_suspects(tmp_path):
    stub = _Stub(_memory_dir(tmp_path), autonomy=True)
    ok_claims = [{"claim": "x", "grounding": "model_prior", "refs": [],
                  "volatility": "stable"},
                 {"claim": "y", "grounding": "retrieved",
                  "refs": ["$step1"]}]
    assert stub._maybe_spawn_suspect_verification(
        _SUSPECT_RECORD, ok_claims) is None
    assert stub.spawned == []


def test_suspect_spawn_gated_off(tmp_path):
    # Autonomy off → never spawns, even on suspect claims.
    stub = _Stub(_memory_dir(tmp_path), autonomy=False)
    assert stub._maybe_spawn_suspect_verification(
        _SUSPECT_RECORD, _SUSPECT_CLAIMS) is None
    # Autonomous turns get no spawn (and no loop: verification fires are
    # autonomous, so they can never re-trigger themselves).
    stub2 = _Stub(tmp_path / "memory", autonomy=True)
    rec = dict(_SUSPECT_RECORD, autonomous=True)
    assert stub2._maybe_spawn_suspect_verification(
        rec, _SUSPECT_CLAIMS) is None


def test_on_demand_attribution_does_not_spawn(tmp_path):
    """The justify path attributes with spawn_verification off — the
    justify turn performs the audit itself."""
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)  # no claims.jsonl

    class _SuspectBackend(_Backend):
        def chat(self, messages, **kwargs):
            self.calls += 1
            return json.dumps({"claims": [
                {"claim": "It is private.", "grounding": "model_prior",
                 "refs": [], "volatility": "volatile"}]})

    stub = _Stub(md, backend=_SuspectBackend(), autonomy=True)
    obs = stub._run_justify("User")
    assert obs.startswith("OK: Provenance trail")
    assert stub.spawned == []


def test_run_justify_autonomous_uncovered(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    # Justify during an autonomous run: latest source=Jill record is
    # autonomous and has no claims — no on-demand attempt is made.
    stub = _Stub(md)
    obs = stub._run_justify("Jill")
    assert obs.startswith("EMPTY: the previous turn here was autonomous")
    assert stub.backend.calls == 0
