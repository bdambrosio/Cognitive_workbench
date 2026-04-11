"""Unit tests for bench/trial.py::run_trial.

All harvester/verdict/reviewer dependencies are monkeypatched. No launcher,
no network, no Zenoh session — pass SENTINEL_SESSION through the API.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(_HERE, "..", "bench"))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

pytest.importorskip("zenoh")  # trial → harvester → zenoh at import time

import harvester  # noqa: E402
import trial  # noqa: E402
from trial import run_trial  # noqa: E402


SENTINEL_SESSION = object()


# ──────────────────────────────────────────────────────────────────────
# Shared harvester-mock helper
# ──────────────────────────────────────────────────────────────────────

def _install_happy_harvester(monkeypatch, *, status="completed", quality="passed"):
    """Patch harvester.* so a run step completes cleanly without touching
    Zenoh. Also patches trial.compute_verdict to a canned verdict.
    """
    pre_record = {
        "goal_id": "goal_17",
        "name": "G01",
        "status": status,
        "updated": "2026-04-11T10:00:00",
    }
    post_record = dict(pre_record, updated="2026-04-11T10:05:00")

    monkeypatch.setattr(
        harvester, "query_goal_record",
        lambda session, character, goal_id, timeout=3.0: pre_record,
    )
    monkeypatch.setattr(
        harvester, "submit_goal_run",
        lambda session, character, goal_id, mode=None: True,
    )
    monkeypatch.setattr(
        harvester, "wait_for_completion",
        lambda session, character, goal_id, poll_interval, timeout, pre_submit_updated: post_record,
    )
    monkeypatch.setattr(
        harvester, "slice_log_for_goal",
        lambda log_path, goal_id: [],
    )
    monkeypatch.setattr(
        harvester, "compute_metrics",
        lambda record, log_lines, goal_id, submitted_mode: {
            "goal_id": goal_id,
            "status": record["status"],
            "quality_status": quality,
            "last_run_mode": submitted_mode,
            "_harvest_error": None,
        },
    )
    monkeypatch.setattr(
        trial, "compute_verdict",
        lambda row, session=None, character=None, model=None, api_key=None: {
            **row,
            "opus_verdict": {"verdict": "satisfied", "score_percent": 100},
        },
    )


# ──────────────────────────────────────────────────────────────────────
# 1. Step validation
# ──────────────────────────────────────────────────────────────────────

_INVALID_STEP_CASES = [
    pytest.param([],                                          "non-empty list",         id="empty"),
    pytest.param("not a list",                                "non-empty list",         id="not-list"),
    pytest.param([{"kind": "bogus"}],                         "kind must be one of",    id="bad-kind"),
    pytest.param([{"kind": "run", "mode": "bogus"}],          "run mode must be one of", id="bad-mode"),
    pytest.param([{"kind": "run"}],                           "run mode must be one of", id="run-no-mode"),
    pytest.param([{"kind": "review"}],                        "must specify 'commit'",  id="review-no-commit"),
    pytest.param([{"kind": "review", "commit": "yes"}],       "commit must be bool",    id="commit-not-bool"),
]


@pytest.mark.parametrize("steps,fragment", _INVALID_STEP_CASES)
def test_validation_rejects_malformed_steps(steps, fragment):
    result = run_trial("goal_17", steps=steps, session=SENTINEL_SESSION)
    assert result["error"] is not None
    assert "validation failed" in result["error"]
    assert fragment in result["error"]
    assert result["steps"] == []


# ──────────────────────────────────────────────────────────────────────
# 2. Happy path: two runs, trial_context passthrough
# ──────────────────────────────────────────────────────────────────────

def test_happy_path_two_runs_and_context_passthrough(monkeypatch):
    _install_happy_harvester(monkeypatch)
    ctx = {"phase": "A", "seed": 42, "note": "smoke"}

    result = run_trial(
        "goal_17",
        steps=[
            {"kind": "run", "mode": "replan"},
            {"kind": "run", "mode": "replay"},
        ],
        trial_context=ctx,
        session=SENTINEL_SESSION,
    )

    assert result["error"] is None
    assert result["trial_context"] == ctx  # opaque passthrough, verbatim
    assert len(result["steps"]) == 2
    for step, expected_mode in zip(result["steps"], ["replan", "replay"]):
        assert step["kind"] == "run"
        assert step["mode"] == expected_mode
        assert step["error"] is None
        assert step["row"] is not None
        assert step["row"]["last_run_mode"] == expected_mode
        assert step["verdict"] == {"verdict": "satisfied", "score_percent": 100}


# ──────────────────────────────────────────────────────────────────────
# 3. Fail-fast: harvester error aborts the rest of the trial
# ──────────────────────────────────────────────────────────────────────

def test_fail_fast_on_harvest_error_aborts_remaining_steps(monkeypatch):
    _install_happy_harvester(monkeypatch)

    timeout_record = {
        "goal_id": "goal_17",
        "status": "running",
        "updated": "2026-04-11T10:00:00",
        "_harvest_error": "timeout",
    }
    monkeypatch.setattr(
        harvester, "wait_for_completion",
        lambda *a, **k: timeout_record,
    )

    review_called = {"n": 0}
    def exploding_review(*a, **k):
        review_called["n"] += 1
        return {"llm_called": True, "committed": True}
    monkeypatch.setattr(trial, "review_goal", exploding_review)

    result = run_trial(
        "goal_17",
        steps=[
            {"kind": "run", "mode": "replan"},
            {"kind": "review", "commit": True},
            {"kind": "run", "mode": "replay"},
        ],
        session=SENTINEL_SESSION,
    )

    assert len(result["steps"]) == 1  # steps 1 and 2 never ran
    assert result["steps"][0]["error"] is not None
    assert "timeout" in result["steps"][0]["error"]
    assert result["error"] is not None
    assert "step 0" in result["error"]
    assert review_called["n"] == 0


# ──────────────────────────────────────────────────────────────────────
# 4. Verdict failure does not fail the run step
# ──────────────────────────────────────────────────────────────────────

def test_verdict_failure_leaves_row_populated_and_step_clean(monkeypatch):
    _install_happy_harvester(monkeypatch)

    def failing_verdict(*a, **k):
        raise RuntimeError("api down")
    monkeypatch.setattr(trial, "compute_verdict", failing_verdict)

    result = run_trial(
        "goal_17",
        steps=[{"kind": "run", "mode": "replan"}],
        session=SENTINEL_SESSION,
    )

    assert result["error"] is None
    assert len(result["steps"]) == 1
    step = result["steps"][0]
    assert step["error"] is None
    assert step["row"] is not None
    assert step["verdict"] is None
    assert "RuntimeError" in step.get("verdict_error", "")
    assert "api down" in step.get("verdict_error", "")


# ──────────────────────────────────────────────────────────────────────
# 5. Review step: dry-run vs commit semantics
# ──────────────────────────────────────────────────────────────────────

def test_review_dry_run_succeeds_with_llm_called(monkeypatch):
    captured = {}
    def fake_review(goal_id, character, session, model, api_key, dry_run):
        captured["dry_run"] = dry_run
        return {"llm_called": True, "committed": False, "plan_text": "REWRITE", "learnings_count": 3}
    monkeypatch.setattr(trial, "review_goal", fake_review)

    result = run_trial(
        "goal_17",
        steps=[{"kind": "review", "commit": False}],
        session=SENTINEL_SESSION,
    )

    assert result["error"] is None
    assert captured["dry_run"] is True
    step = result["steps"][0]
    assert step["kind"] == "review"
    assert step["commit"] is False
    assert step["error"] is None
    assert step["review"]["llm_called"] is True
    assert step["review"]["committed"] is False
    # plan_text is split out of the review sub-dict by _execute_review_step
    assert step["plan_text"] == "REWRITE"


def test_review_commit_fails_when_not_actually_committed(monkeypatch):
    # commit=True requested, but the reviewer claims it didn't land.
    def fake_review(goal_id, character, session, model, api_key, dry_run):
        return {"llm_called": True, "committed": False, "plan_text": None, "learnings_count": 0}
    monkeypatch.setattr(trial, "review_goal", fake_review)

    result = run_trial(
        "goal_17",
        steps=[{"kind": "review", "commit": True}],
        session=SENTINEL_SESSION,
    )

    assert result["error"] is not None
    assert "step 0" in result["error"]
    step = result["steps"][0]
    assert step["error"] == "review completed but commit did not succeed"
