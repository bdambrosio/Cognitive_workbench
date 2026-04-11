"""Live smoke tests for bench/ — require a running Jill launcher with
G01 available. Self-skipping: if the launcher can't be reached or G01
isn't found, the tests skip silently so this file is safe to include in
the default suite.

To run these explicitly:
    pytest tests/test_bench_smoke.py -v

The full-cycle test additionally requires CLAUDE_API_KEY (or
ANTHROPIC_API_KEY) in the environment, since it calls the reviewer LLM.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(_HERE, "..", "bench"))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

pytest.importorskip("zenoh")

import harvester  # noqa: E402
from trial import run_trial  # noqa: E402


CHARACTER = "Jill"
SMOKE_GOAL_NAME = "G01"


def _has_anthropic_key() -> bool:
    return bool(os.environ.get("CLAUDE_API_KEY") or os.environ.get("ANTHROPIC_API_KEY"))


@pytest.fixture(scope="module")
def live_session_and_goal():
    """Open a Zenoh session and resolve G01. Skip the whole module on
    any failure — no Jill, no test, no noise.
    """
    try:
        session = harvester.open_zenoh()
    except Exception as e:
        pytest.skip(f"Zenoh session unavailable: {e}")

    try:
        record = harvester.resolve_goal_handle(session, CHARACTER, SMOKE_GOAL_NAME, timeout=2.0)
    except Exception as e:
        try:
            session.close()
        except Exception:
            pass
        pytest.skip(f"Could not reach {CHARACTER} launcher: {e}")

    if record is None:
        try:
            session.close()
        except Exception:
            pass
        pytest.skip(
            f"{SMOKE_GOAL_NAME} not found on {CHARACTER} — "
            f"launcher not running or goals not seeded"
        )

    goal_id = record["goal_id"]
    yield session, goal_id

    try:
        session.close()
    except Exception:
        pass


def test_smoke_one_step_replan_against_G01(live_session_and_goal):
    """End-to-end: resolver + harvester + trial driver + one live replan
    run. Proves the full unaided-planner path works.

    Does NOT assert on verdict contents — if CLAUDE_API_KEY isn't set,
    verdict will fail gracefully and trial.py records verdict_error
    without failing the step. We only require that the RUN portion
    produced a valid row.
    """
    session, goal_id = live_session_and_goal

    result = run_trial(
        goal_id,
        steps=[{"kind": "run", "mode": "replan"}],
        session=session,
    )

    assert result["error"] is None, f"Trial errored: {result['error']}"
    assert len(result["steps"]) == 1

    step = result["steps"][0]
    assert step["kind"] == "run"
    assert step["mode"] == "replan"
    assert step["error"] is None, f"Run step errored: {step['error']}"

    row = step["row"]
    assert row is not None
    assert row.get("goal_id", "").startswith("goal_")
    assert row.get("status") in {"completed", "failed", "ready"}, \
        f"unexpected status: {row.get('status')!r}"
    # quality_status may be blank on replan runs with no $eval_target;
    # empty string is an accepted value — we just care the key is set.
    assert "quality_status" in row
    assert (row.get("cached_plan_step_count") or 0) >= 1, \
        "planner should have produced at least one step"


def test_smoke_full_cycle_replan_then_dry_review(live_session_and_goal):
    """Optional: run G01 in replan, then invoke the reviewer in dry-run
    mode. Verifies trial.py sequences a multi-step protocol correctly
    against a live agent and that dry-run review is observational only
    (no commit, no mutation).

    Skips without an Anthropic API key, since the reviewer LLM call is
    real and costs tokens (~$0.05-0.20 depending on model).
    """
    if not _has_anthropic_key():
        pytest.skip("No CLAUDE_API_KEY/ANTHROPIC_API_KEY — reviewer LLM unavailable")

    session, goal_id = live_session_and_goal

    result = run_trial(
        goal_id,
        steps=[
            {"kind": "run",    "mode":   "replan"},
            {"kind": "review", "commit": False},
        ],
        session=session,
    )

    assert result["error"] is None, f"Trial errored: {result['error']}"
    assert len(result["steps"]) == 2

    run_step, review_step = result["steps"]
    assert run_step["error"] is None
    assert run_step["row"] is not None

    assert review_step["error"] is None, \
        f"Review step errored: {review_step['error']}"
    review = review_step["review"] or {}
    assert review.get("llm_called") is True, "dry-run should still invoke the LLM"
    assert review.get("committed") is False, "dry-run must not commit"
