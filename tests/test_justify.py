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

def _sha1_of_reply(turn_seq: int) -> str:
    """The reply hash justify checks the claims record against. Computed
    from the fixture rather than hardcoded so it cannot drift out of sync
    with TRACE and silently start exercising the mismatch path."""
    import hashlib
    rec = next(r for r in TRACE if r["turn_seq"] == turn_seq)
    return hashlib.sha1(
        str(rec["raw_response"]).strip().encode("utf-8")).hexdigest()


CLAIMS = [
    {"turn_seq": 12, "source": "User", "reply_sha1": _sha1_of_reply(12),
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
    """A healthy trail off a directly-retrieved observation raises nothing.

    The observation here is fetch-text — bytes this harness pulled — so
    the model-synthesis note does not apply and no claim grades below
    probable.
    """
    md = _memory_dir(tmp_path)
    _populate(md)
    trace = dict(TRACE[1])
    trace["tool_meta"] = {"$step1": {"tool": "fetch-text", "meta": []}}
    claims_rec = {"turn_seq": 12, "claims": [
        {"claim": "There is no direct world copy/paste.",
         "grounding": "retrieved", "refs": ["$step1"]},
    ]}
    out = render_justification(claims_rec, trace, md)
    assert "Grounding profile: 1 retrieved" in out
    assert "Audit note:" not in out
    assert "Weakest link" not in out  # nothing below probable


def test_render_flags_model_synthesised_observation(tmp_path):
    """A trail resting wholly on search-web is one model's account of its
    reading — the render has to say so, per claim and once as an audit
    note, even though every claim grades probable and nothing is a
    'weakest link'."""
    md = _memory_dir(tmp_path)
    _populate(md)
    claims_rec = {"turn_seq": 12, "claims": [
        {"claim": "There is no direct world copy/paste.",
         "grounding": "retrieved", "refs": ["$step1"],
         "quote": "the wiki states there is no direct copy/paste"},
    ]}
    out = render_justification(claims_rec, TRACE[1], md)
    assert "not from a source document" in out
    assert "model-synthesised observation" in out
    assert "every claim rests on a model-synthesised observation" in out
    assert "Weakest link" not in out  # still nothing below probable


def test_named_step_without_sources_is_flagged_mediated_not_bibliographic(tmp_path):
    """A claim quoted against a `process_text` synthesis must carry the
    mediation warning — that step is a second model pass over the turn's
    own material, the likeliest place for drift to enter. Before every
    step was named, such a ref was absent from tool_meta and so got no
    mediation verdict at all, while the same claim against search-web got
    one. It must not promise "the sources below" either: there are none.
    """
    md = _memory_dir(tmp_path)
    _populate(md)
    trace = dict(TRACE[1])
    trace["tool_meta"] = {
        "$step1": {"tool": "search-web", "meta": [
            {"tool_metadata": {"sources": [
                {"url": "https://wiki.example/bp", "domain": "wiki.example"}]}}]},
        "$step2": {"tool": "process_text", "meta": []},
    }
    claims_rec = {"turn_seq": 12, "claims": [
        {"claim": "a", "grounding": "retrieved", "refs": ["$step2"],
         "quote": "there is no direct copy/paste of world objects"},
    ]}
    out = render_justification(claims_rec, trace, md)
    assert "quoted against process_text output" in out
    assert "a model's own text over material already in this turn" in out
    assert "the sources below" not in out
    assert "$step2 — process_text [model-synthesised observation" in out
    # No dangling colon: nothing is listed under this step.
    assert "was read for it]:" not in out


def test_grounding_key_states_the_ceiling_it_computed(tmp_path):
    """The reducer explains its own ceiling. A bare grade begs "why?" and
    gets an invented answer — turn 2408 explained four probable grades as
    caused by the observation being a model synthesis, which is false
    (mediation is not a cap). Stating the true reason leaves nothing to
    invent."""
    md = _memory_dir(tmp_path)
    _populate(md)
    out = render_justification(CLAIMS[0], TRACE[1], md)
    assert "no claim can reach `sourced` yet" in out
    assert "NOT applied as caps" in out


def test_render_source_composition_splits_read_from_named(tmp_path):
    """Composition counts domains and says which were opened here.

    No quality judgment — the split that matters before any tiering is
    read-here vs named-by-a-model, since a source nobody opened cannot
    support a claim however authoritative it is.
    """
    md = _memory_dir(tmp_path)
    _populate(md)
    trace = dict(TRACE[1])
    trace["tool_meta"] = {
        "$step1": {"tool": "search-web", "meta": [
            {"tool_metadata": {"sources": [
                {"url": "https://wiki.example/bp", "domain": "wiki.example"},
                {"url": "https://news.example/x", "domain": "news.example"}]}}]},
        "$step2": {"tool": "fetch-text", "meta": [
            {"tool_metadata": {"sources": [
                {"url": "https://wiki.example/bp", "domain": "wiki.example"}]}}]},
    }
    claims_rec = {"turn_seq": 12, "claims": [
        {"claim": "a", "grounding": "retrieved", "refs": ["$step1"]},
        {"claim": "b", "grounding": "retrieved", "refs": ["$step2"]},
    ]}
    out = render_justification(claims_rec, trace, md)
    # wiki.example was opened, so it counts as read even though search
    # also named it; news.example was only ever named.
    assert "2 distinct source domain(s), 1 read here" in out
    assert "not opened: news.example" in out
    # Not every claim is mediated now, so the blanket audit note is gone.
    assert "every claim rests on a model-synthesised observation" not in out


def test_render_justification_no_claims(tmp_path):
    md = _memory_dir(tmp_path)
    out = render_justification({"turn_seq": 5, "claims": []},
                               {"turn_seq": 5, "user_input": "hi"}, md)
    assert "no checkable factual claims" in out


def test_memory_names_its_originating_turn_only_when_it_resolves(tmp_path):
    """A memory's source_turn_seq becomes a followable hop only if the
    number still points at the turn that wrote it. Seqs restarted at 1
    each session before seeding landed, so an old memory's seq now names
    an unrelated recent turn — inviting `justify` on it would render a
    real, well-formed trail for the wrong text."""
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", [
        {"turn_seq": 10, "source": "User", "autonomous": False,
         "ts": "2026-07-20T10:00:00+00:00",
         "user_input": "old question", "raw_response": "old answer",
         "working_log": ""},
        {"turn_seq": 12, "source": "User", "autonomous": False,
         "ts": "2026-07-27T10:00:00+00:00",
         "user_input": "q", "raw_response": "a", "working_log": ""},
    ])
    _write_jsonl(md / "memories.jsonl", [
        {"event": "write", "note_id": "Note_A", "text": "fresh memory",
         "ts": "2026-07-20T10:00:09+00:00", "source_turn_seq": 10},
        {"event": "write", "note_id": "Note_B", "text": "old memory",
         "ts": "2026-06-01T10:00:09+00:00", "source_turn_seq": 10},
        {"event": "write", "note_id": "Note_C", "text": "pre-provenance",
         "ts": "2026-06-01T10:00:09+00:00"},
    ])
    claims_rec = {"turn_seq": 12, "source": "User", "claims": [
        {"claim": "a", "grounding": "memory", "refs": ["Note_A"]},
        {"claim": "b", "grounding": "memory", "refs": ["Note_B"]},
        {"claim": "c", "grounding": "memory", "refs": ["Note_C"]},
    ]}
    out = render_justification(claims_rec, {"turn_seq": 12, "source": "User",
                                            "user_input": "q"}, md)
    assert "written from turn 10 — justify 10 for that turn's own trail" in out
    assert ("written from turn 10, an earlier session's numbering that no "
            "longer resolves") in out
    # A memory with no recorded originating turn says nothing about one.
    assert "Note_C — memory written 2026-06-01: 'pre-provenance'" in out


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


def test_attributor_sees_the_harness_provenance_it_was_told(tmp_path):
    """The Substrate block is prompt content, so a claim restating it is
    `context` — but the attributor could not see it. Live at turn 2447:
    "you definitely look like a crow" came straight out of a commit subject
    in that block, graded model_prior/volatile -> suspect (the worst grade
    available), and spawned a background job to web-verify a fact recorded
    in this repo's git log. Same fix as active_concerns at turn 2316."""
    from chat.claims import attribute_claims

    rec = {
        "turn_seq": 2447, "source": "User",
        "user_input": "Do I look like a crow?",
        "raw_response": "Yes, you definitely look like a crow.",
        "working_log": "$step1 OK: Bruce (the human) - 0.8 m away",
        "active_concerns": ["[agent 1.00] Track what the user wants"],
        "substrate": ("running commit 51f86f04; 1 commit(s) since my last "
                      "session: world: creature avatars - Bruce a crow, "
                      "Jill a kitten, Sentinel an owl"),
        "embodiment": "body: the shared world",
        "recall_hits": [],
    }
    seen = {}

    def _llm(messages):
        seen["sys"], seen["user"] = messages[0]["content"], messages[-1]["content"]
        return '{"claims": []}'

    attribute_claims(rec, _llm, character_name="Jill")
    state = seen["user"][seen["user"].index("## ASSISTANT STATE"):
                         seen["user"].index("## User input")]
    assert "Bruce a crow" in state, "the substrate line must be visible"
    assert "[harness provenance]" in state and "[body]" in state
    # Concerns still there — this adds a channel, it does not replace one.
    assert "Track what the user wants" in state
    # And the rule tells the attributor what to do with it.
    assert "code revision it is running" in seen["sys"]


def test_no_harness_provenance_recorded_is_simply_absent():
    """Legacy records predate the fields; they must render as before."""
    from chat.claims import attribute_claims

    seen = {}

    def _llm(messages):
        seen["user"] = messages[-1]["content"]
        return '{"claims": []}'

    attribute_claims({"turn_seq": 1, "raw_response": "hi there.",
                      "active_concerns": ["[agent 1.00] x"]},
                     _llm, character_name="Jill")
    assert "[harness provenance]" not in seen["user"]
    assert "[agent 1.00] x" in seen["user"]


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
    """Canned attribution backend for the on-demand path.

    Mirrors the two real-backend attributes the claims path reads:
    `is_cloud` (picks the token floor) and `last_finish_reason` (drives
    retry-on-truncation). Both are set in ChatBackend.__init__, so the
    production code reads them directly rather than defensively.
    """

    def __init__(self, raise_instead=False):
        self.raise_instead = raise_instead
        self.calls = 0
        self.is_cloud = False
        self.last_finish_reason = 'stop'
        self.max_tokens_seen = []

    def chat(self, messages, **kwargs):
        self.calls += 1
        self.max_tokens_seen.append(kwargs.get('max_tokens'))
        if self.raise_instead:
            raise RuntimeError("backend down")
        return json.dumps({"claims": [
            {"claim": "There is no direct world copy/paste.",
             "grounding": "retrieved", "refs": ["$step1"]}]})


class _Stub(ClaimsMixin):
    character_name = "TestJill"

    # Read the real tables rather than copying the numbers: the whole
    # point of the claims path sourcing its budget from them is that the
    # two cannot drift, and a hardcoded copy here would hide exactly the
    # drift these tests exist to catch.
    from chat.chat_loop import ChatLoop as _CL
    _PROFILE_TOKEN_FLOOR_LOCAL = _CL._PROFILE_TOKEN_FLOOR_LOCAL
    _PROFILE_TOKEN_FLOOR_CLOUD = _CL._PROFILE_TOKEN_FLOOR_CLOUD
    del _CL

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
    """A hard failure is ERROR, not EMPTY, and records nothing.

    EMPTY means "ran clean, nothing to show", which reads as retryable and
    produced the retry-then-apologise loop at turn 2393. The message must
    also not suggest waiting: this path attributes on demand.
    """
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    obs = _Stub(md, _Backend(raise_instead=True))._run_justify("User")
    assert obs.startswith("ERROR:")
    assert "turn 12" in obs
    assert "nothing to wait for" in obs
    # A failure must never manufacture a record — claims: [] is a real
    # finding ("no checkable claims"), not a failure mode.
    assert claims_for_turn(md, 12) is None
    assert not (md / "claims.jsonl").exists() or \
        (md / "claims.jsonl").read_text().strip() == ""


def test_attribution_budget_comes_from_the_floor_table(tmp_path):
    """The regression test for turn 2393.

    The claims call reaches backend.chat directly and so never got
    _make_llm_callable's profile floor; a literal 1600 truncated the JSON
    on long replies and the whole pass was dropped silently.
    """
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)

    local = _Backend()
    _Stub(md, local)._extract_and_log_claims(12)
    assert local.max_tokens_seen == [_Stub._PROFILE_TOKEN_FLOOR_LOCAL["none"]]
    assert local.max_tokens_seen[0] >= 16384

    md2 = tmp_path / "cloud" / "memory"
    md2.mkdir(parents=True)
    _write_jsonl(md2 / "reasoning_trace.jsonl", TRACE)
    cloud = _Backend()
    cloud.is_cloud = True
    _Stub(md2, cloud)._extract_and_log_claims(12)
    assert cloud.max_tokens_seen == [_Stub._PROFILE_TOKEN_FLOOR_CLOUD["none"]]


class _TruncatingBackend(_Backend):
    """First call returns a cut-off claims array, second returns valid JSON."""

    def chat(self, messages, **kwargs):
        self.calls += 1
        self.max_tokens_seen.append(kwargs.get("max_tokens"))
        if self.calls == 1:
            self.last_finish_reason = "length"
            return ('{"claims": [{"claim": "a", "grounding": "retrieved", '
                    '"refs": ["$step1"]}, {"claim": "b", "grou')
        self.last_finish_reason = "stop"
        return json.dumps({"claims": [
            {"claim": "a", "grounding": "retrieved", "refs": ["$step1"]},
            {"claim": "b", "grounding": "retrieved", "refs": ["$step1"]}]})


def test_truncated_attribution_salvages_then_retries_bigger(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    be = _TruncatingBackend()
    stub = _Stub(md, be)
    stub._extract_and_log_claims(12)
    # Salvage recovered the one complete claim, so the first pass returned
    # a partial rather than None — the retry then supersedes it.
    rec = claims_for_turn(md, 12)
    assert rec is not None
    assert len(rec["claims"]) == 2
    assert "incomplete" not in rec
    assert be.max_tokens_seen[-1] == be.max_tokens_seen[0] * 2


def test_salvage_recovers_complete_claims_and_drops_the_partial():
    from chat.claims import _salvage_truncated_claims
    cut = ('```json\n{\n  "claims": [\n'
           '    {"claim": "first", "grounding": "retrieved", "refs": ["$step1"]},\n'
           '    {"claim": "second", "grounding": "retrieved", "refs": ["$step1"]},\n'
           '    {"claim": "third", "groun')
    out = _salvage_truncated_claims(cut)
    assert [c["claim"] for c in out] == ["first", "second"]
    # Nothing complete to recover → None, so the caller can still tell a
    # truncated pass from an empty finding.
    assert _salvage_truncated_claims('{"claims": [{"claim": "only par') is None
    assert _salvage_truncated_claims("not json at all") is None


def test_already_attributed_turn_is_not_re_attributed(tmp_path):
    """justify attributes on demand, so the post-turn job often arrives to
    find the work done. Re-running costs another call and appends a
    duplicate line."""
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    stub = _Stub(md, _Backend())
    stub._extract_and_log_claims(12)                       # on-demand
    stub._extract_and_log_claims(12, spawn_verification=True)  # post-turn
    assert stub.backend.calls == 1
    lines = [l for l in (md / "claims.jsonl").read_text().splitlines() if l.strip()]
    assert len(lines) == 1


def test_stale_record_for_a_different_reply_does_not_suppress(tmp_path):
    """The skip matches reply_sha1 and source, not turn_seq alone — a seq
    can repeat across old sessions."""
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    _write_jsonl(md / "claims.jsonl", [{
        "turn_seq": 12, "source": "User", "reply_sha1": "0" * 40,
        "claims": [{"claim": "stale", "grounding": "retrieved",
                    "refs": ["$step1"]}],
    }])
    stub = _Stub(md, _Backend())
    stub._extract_and_log_claims(12)
    assert stub.backend.calls == 1                      # ran anyway
    assert claims_for_turn(md, 12)["claims"][0]["claim"] != "stale"


def test_partial_record_renders_a_coverage_caveat(tmp_path):
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    _write_jsonl(md / "claims.jsonl", [{
        "turn_seq": 12, "source": "User",
        "incomplete": {"reason": "length", "recovered": 1},
        "claims": [{"claim": "a", "grounding": "retrieved", "refs": ["$step1"]}],
    }])
    # Backend returns a complete pass, so the re-attempt supersedes the
    # partial and the caveat is gone.
    stub = _Stub(md, _Backend())
    obs = stub._run_justify("User")
    assert stub.backend.calls == 1
    assert obs.startswith("OK: Provenance trail")
    assert "PARTIAL TRAIL" not in obs


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


def test_suspect_spawn_skips_peer_exchanges(tmp_path):
    """A reply to a co-resident agent is not audited. Its claims are
    first-person reports of what this agent did or saw; restated a turn
    later they carry no in-turn evidence and grade model_prior/volatile,
    which fired a web search against an agent's own eyes (2026-08-12,
    "I've spotted a marker at (62.1, 101.6)"). User-facing replies still
    spawn — the same record from the User does."""
    stub = _Stub(_memory_dir(tmp_path), autonomy=True)
    stub._peers = ["Jack", "Sentinel"]
    assert stub._maybe_spawn_suspect_verification(
        dict(_SUSPECT_RECORD, source="Jack"), _SUSPECT_CLAIMS) is None
    assert stub.spawned == []
    assert stub._maybe_spawn_suspect_verification(
        _SUSPECT_RECORD, _SUSPECT_CLAIMS) is not None


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


# --- elided-observation restoration (turn 2349 regression) ------------
# The stored trace caps each observation so the record stays cheap to
# re-inject, but attribution reads the same field as evidence. Before
# `observations_full`, facts the model had read verbatim past the cap
# were graded model_prior with empty refs — a false "I guessed this"
# about content it actually sourced.

_CAPPED = ("--- iter 1 ---\nACTION: {\"tool\": \"fetch-text\"}\n"
           "$step1: OK: head of the section"
           " …[observation capped at 1000 chars]\n")


def test_restore_observations_puts_elided_text_back():
    from chat.claims import _restore_observations
    out = _restore_observations(
        _CAPPED, {"$step1": "OK: head of the section AND THE ELIDED TAIL"})
    assert "AND THE ELIDED TAIL" in out
    assert "observation capped at" not in out
    assert out.startswith("--- iter 1 ---")


def test_restore_observations_noop_without_field():
    """Records written before the field existed must pass through."""
    from chat.claims import _restore_observations
    assert _restore_observations(_CAPPED, {}) == _CAPPED


def test_restore_observations_skips_unmatched_labels():
    from chat.claims import _restore_observations
    assert _restore_observations(_CAPPED, {"$step9": "x"}) == _CAPPED
    uncapped = "--- iter 1 ---\n$step1: complete already\n"
    assert _restore_observations(uncapped, {"$step1": "x"}) == uncapped


def test_quote_from_restored_span_survives_verbatim_check():
    """The attributor is shown the restored log, so a quote copied from a
    previously elided span must verify against it — checking against the
    capped `working_log` would drop honest citations."""
    record = {
        "raw_response": "The tail value was 47.",
        "working_log": _CAPPED,
        "observations_full": {
            "$step1": "OK: head of the section, and the tail value was 47."},
    }
    claims = attribute_claims(
        record,
        lambda _m: json.dumps({"claims": [{
            "claim": "The tail value was 47",
            "grounding": "retrieved",
            "refs": ["$step1"],
            "quote": "the tail value was 47",
        }]}),
        character_name="TestAgent")
    assert len(claims) == 1
    assert claims[0]["grounding"] == "retrieved"
    assert claims[0]["refs"] == ["$step1"]
    assert claims[0]["quote"] == "the tail value was 47", \
        "quote copied from the restored span was dropped"


# ── post-turn stage isolation (Ship 2) ─────────────────────────────────

def test_post_turn_stages_are_isolated_and_claims_run_first():
    """One failing stage must not skip the rest, and claims must lead.

    Claims used to run 4th, behind three LLM stages, so it landed 37-98s
    after the reply and any exception escaping an earlier stage skipped it
    entirely. Calls the real ChatLoop method against a fake self.
    """
    from chat.chat_loop import ChatLoop

    order = []

    class _Fake:
        character_name = "TestJill"

        def _extract_and_log_claims(self, seq, spawn_verification=False):
            order.append("claims")

        def _update_discourse_async(self, source):
            order.append("discourse")
            raise RuntimeError("discourse exploded")

        def _reflect_and_remember(self, source):
            order.append("reflection")

        def _update_thread_centroids(self):
            order.append("centroids")

        def _persist_to_disk(self):
            order.append("persist")

    ChatLoop._post_turn_work(_Fake(), "User", False, 12)
    # Claims first, and the discourse blow-up did not stop what follows.
    assert order[0] == "claims"
    assert order == ["claims", "discourse", "reflection", "centroids",
                     "persist"]


def test_post_turn_survives_a_failing_claims_stage():
    """The isolation cuts both ways: claims failing must not cost the
    turn its memory writes."""
    from chat.chat_loop import ChatLoop

    order = []

    class _Fake:
        character_name = "TestJill"

        def _extract_and_log_claims(self, seq, spawn_verification=False):
            order.append("claims")
            raise RuntimeError("attribution exploded")

        def _update_discourse_async(self, source):
            order.append("discourse")

        def _reflect_and_remember(self, source):
            order.append("reflection")

        def _update_thread_centroids(self):
            order.append("centroids")

        def _persist_to_disk(self):
            order.append("persist")

    ChatLoop._post_turn_work(_Fake(), "User", False, 12)
    assert order == ["claims", "discourse", "reflection", "centroids",
                     "persist"]


# ── by-turn targeting (Ship 3) ─────────────────────────────────────────

def test_justify_targets_an_earlier_turn_by_seq(tmp_path):
    """The user reads the turn number off the reply and names it, so an
    older answer stays auditable instead of being lost the moment another
    turn lands."""
    md = _memory_dir(tmp_path)
    _populate(md)
    stub = _Stub(md, _Backend())
    obs = stub._run_justify("User", 10)          # not the latest (12)
    assert obs.startswith("OK: Provenance trail")
    assert "turn 10" in obs
    assert "old question" in obs                 # echoes what it answered


def test_justify_by_seq_cannot_reach_another_conversation(tmp_path):
    """Seqs are global, conversations are not. The source predicate is
    what stops a number naming a peer's or another channel's turn."""
    md = _memory_dir(tmp_path)
    trace = TRACE + [{"turn_seq": 77, "source": "Sentinel", "autonomous": False,
                      "user_input": "patrol?", "raw_response": "all clear",
                      "working_log": ""}]
    _write_jsonl(md / "reasoning_trace.jsonl", trace)
    obs = _Stub(md, _Backend())._run_justify("User", 77)
    assert obs.startswith("EMPTY: no reply to this conversation with turn "
                          "number 77")
    assert "reconstruct a trail from memory" in obs


def test_justify_by_seq_prefers_the_live_record_on_a_collision(tmp_path):
    """Seqs 1..50 repeat across pre-seeding sessions; the later record is
    the live one."""
    md = _memory_dir(tmp_path)
    trace = [
        {"turn_seq": 7, "source": "User", "autonomous": False,
         "user_input": "ancient", "raw_response": "stale answer",
         "working_log": ""},
        {"turn_seq": 7, "source": "User", "autonomous": False,
         "user_input": "recent", "raw_response": "live answer",
         "working_log": ""},
    ]
    _write_jsonl(md / "reasoning_trace.jsonl", trace)
    assert latest_turn_for_source(md, "User", 7)["raw_response"] == \
        "live answer"


def test_justify_rejects_a_claims_record_for_a_different_reply(tmp_path):
    """reply_sha1 finally earns its keep: a stale claims record under a
    reused seq would render a real trail for the wrong text."""
    md = _memory_dir(tmp_path)
    _write_jsonl(md / "reasoning_trace.jsonl", TRACE)
    _write_jsonl(md / "claims.jsonl", [{
        "turn_seq": 12, "source": "User", "reply_sha1": "f" * 40,
        "claims": [{"claim": "belongs to another reply",
                    "grounding": "retrieved", "refs": ["$step1"]}],
    }])
    stub = _Stub(md, _Backend())
    obs = stub._run_justify("User")
    assert stub.backend.calls == 1               # re-attributed
    assert "belongs to another reply" not in obs


def test_justify_defaults_to_latest_when_no_seq_given(tmp_path):
    md = _memory_dir(tmp_path)
    _populate(md)
    obs = _Stub(md, _Backend())._run_justify("User")
    assert "turn 12" in obs and "Can I copy/paste blueprints?" in obs


# ── turn number shown to the user matches the recorded turn ────────────

def test_assigned_seq_is_stable_within_a_turn_and_advances_between(tmp_path):
    """The number shown beside a reply must be the number that reply is
    recorded under — otherwise 'justify 2393' audits the wrong text.

    _assign_turn_seq is called twice per turn: once before publish (to
    display) and once from the trace write. Both must yield the same value.
    """
    from chat.chat_loop import ChatLoop

    trace = tmp_path / "reasoning_trace.jsonl"
    _write_jsonl(trace, [{"turn_seq": n} for n in range(1, 2393)])

    class _Fake(ChatLoop):
        def __init__(self):
            self.character_name = "TestJill"
            self._turn_seq = 0

        def _reasoning_trace_path(self):
            return trace

    loop = _Fake()
    # Seeded from the line count, not restarted at 1.
    first = loop._assign_turn_seq()
    assert first == 2393
    assert loop._assign_turn_seq() == 2393      # idempotent within the turn

    loop._pending_turn_seq = None               # next turn
    assert loop._assign_turn_seq() == 2394
