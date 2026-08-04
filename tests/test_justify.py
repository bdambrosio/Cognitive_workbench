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
    # Every claim present with its grounding tag.
    assert "[retrieved ← $step1] There is no direct world copy/paste." in out
    assert "[model_prior] The game engine is Unreal." in out
    # The verified quote renders under its claim.
    assert 'quote: "there is no direct copy/paste of world objects"' in out
    # Evidence resolved: search source URL, query, note text with date.
    assert "https://wiki.example/bp" in out
    assert "query: blueprint copy paste" in out
    assert "Note_7 — memory written 2026-07-27" in out
    assert "User prefers modular factory builds." in out
    assert "user_input — the user's own words" in out
    # Grounding profile summarizes the validated grounding counts, and
    # the model_prior claim triggers the audit note.
    assert ("Grounding profile: 1 inferred, 1 memory, 1 model_prior, "
            "1 retrieved, 1 user_asserted") in out
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

    def __init__(self, md: Path, backend=None):
        self._md = Path(md)
        self.backend = backend or _Backend()

    def _memory_dir(self) -> Path:
        return self._md

    def _reasoning_trace_path(self) -> Path:
        return self._md / "reasoning_trace.jsonl"


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
    assert "[retrieved ← $step1] There is no direct world copy/paste." in obs
    # The on-demand pass persisted the record (durable, not one-shot).
    assert claims_for_turn(md, 12) is not None


def test_run_justify_attribution_fails(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    obs = _Stub(md, _Backend(raise_instead=True))._run_justify("User")
    assert obs.startswith("EMPTY: claim attribution for turn 12")


def test_run_justify_autonomous_uncovered(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    # Justify during an autonomous run: latest source=Jill record is
    # autonomous and has no claims — no on-demand attempt is made.
    stub = _Stub(md)
    obs = stub._run_justify("Jill")
    assert obs.startswith("EMPTY: the previous turn here was autonomous")
    assert stub.backend.calls == 0
