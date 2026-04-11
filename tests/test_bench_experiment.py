"""Unit tests for bench/experiment.py: plan builders + run_experiment.

Builders are pure functions — tested directly. The runner is tested with
monkeypatched launcher/snapshot/session/run_trial so no launcher is
required. All tests complete in under a second.
"""
import json
import os
import sys

import pytest

_HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(_HERE, "..", "bench"))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

pytest.importorskip("zenoh")  # experiment → harvester → zenoh at import

import experiment  # noqa: E402
import harvester  # noqa: E402
import snapshot  # noqa: E402
from experiment import (  # noqa: E402
    build_trial_A,
    build_trial_B,
    build_trial_C,
    read_reviewed_from_jsonl,
    run_experiment,
    TRIAL_A,
    TRIAL_B,
    TRIAL_C,
)


SENTINEL_SESSION = object()


# ──────────────────────────────────────────────────────────────────────
# Builder tests — pure, no mocking
# ──────────────────────────────────────────────────────────────────────

def test_build_trial_A_structure_and_determinism():
    goals = ["G01", "G02", "G03"]
    plan_a = build_trial_A(goals, seed=42)
    plan_b = build_trial_A(goals, seed=42)
    plan_c = build_trial_A(goals, seed=43)

    # Seeded shuffle is deterministic.
    assert plan_a == plan_b
    # Different seed → likely different order (may collide for tiny sets,
    # but with 3 goals seeds 42 vs 43 do in fact differ).
    assert plan_a["goals"] != plan_c["goals"]

    assert plan_a["trial_type"] == TRIAL_A
    assert plan_a["seed"] == 42
    assert plan_a["name"] == "trial-A-seed-42"
    assert set(plan_a["goals"]) == set(goals)
    assert len(plan_a["goals"]) == 3
    for g in goals:
        assert plan_a["per_goal_protocol"][g] == [{"kind": "run", "mode": "replan"}]
        assert plan_a["per_goal_reset"][g] == {
            "label": "baseline-clean",
            "preserve": [],
        }
    assert plan_a["phase_boundaries"] == {}
    assert plan_a["post_trial_snapshot"] is None


def test_build_trial_B_structure():
    goals = ["G01", "G02", "G03", "G04", "G05", "G06", "G07", "G08", "G09", "G10"]
    plan = build_trial_B(goals, seed=42, split_ratio=0.5)

    assert plan["trial_type"] == TRIAL_B
    assert plan["name"] == "trial-B-seed-42"
    assert len(plan["goals"]) == 10

    first_half = plan["metadata"]["first_half"]
    second_half = plan["metadata"]["second_half"]
    assert len(first_half) == 5
    assert len(second_half) == 5
    assert set(first_half) | set(second_half) == set(goals)
    assert not (set(first_half) & set(second_half))

    # First-half goal 0: full baseline restore.
    assert plan["per_goal_reset"][first_half[0]] == {
        "label": "baseline-clean", "preserve": [],
    }
    # First-half goals 1..4: no reset (launcher stays up so cached plans
    # and world model accumulate).
    for g in first_half[1:]:
        assert plan["per_goal_reset"][g] is None
    # First-half protocol is the 3-step accumulating-review cycle.
    for g in first_half:
        assert plan["per_goal_protocol"][g] == [
            {"kind": "run",    "mode": "replan"},
            {"kind": "review", "commit": True},
            {"kind": "run",    "mode": "replay"},
        ]

    # Second-half: full restore of post-firsthalf snapshot, replan only.
    expected_label = "post-B-firsthalf-seed-42"
    for g in second_half:
        assert plan["per_goal_reset"][g] == {"label": expected_label, "preserve": []}
        assert plan["per_goal_protocol"][g] == [{"kind": "run", "mode": "replan"}]

    # Phase boundary: snapshot immediately after the last first-half goal.
    assert plan["phase_boundaries"] == {
        len(first_half) - 1: {"action": "snapshot", "label": expected_label},
    }
    assert plan["metadata"]["post_firsthalf_snapshot"] == expected_label
    assert plan["post_trial_snapshot"] is None


def test_build_trial_C_picks_replay_for_reviewed_goals():
    goals = ["G01", "G02", "G03", "G04", "G05"]
    reviewed = ["G01", "G03", "G05"]
    plan = build_trial_C(
        goals, seed=42,
        post_firsthalf_label="post-B-firsthalf-seed-42",
        reviewed_in_b=reviewed,
    )

    assert plan["trial_type"] == TRIAL_C
    assert plan["name"] == "trial-C-seed-42"
    assert set(plan["goals"]) == set(goals)

    for g in goals:
        expected_mode = "replay" if g in reviewed else "replan"
        assert plan["per_goal_protocol"][g] == [
            {"kind": "run", "mode": expected_mode}
        ]
        assert plan["per_goal_reset"][g] == {
            "label": "post-B-firsthalf-seed-42",
            "preserve": [],
        }

    assert plan["metadata"]["source_snapshot"] == "post-B-firsthalf-seed-42"
    assert set(plan["metadata"]["reviewed_in_b"]) == set(reviewed)


# ──────────────────────────────────────────────────────────────────────
# Runner fakes — installed by the fixture below
# ──────────────────────────────────────────────────────────────────────

class FakeLauncher:
    """Minimal stand-in for bench.launcher.LauncherProcess."""
    def __init__(self):
        self._running = False
        self.calls = []

    def start(self):
        self.calls.append("start")
        self._running = True

    def wait_ready(self, timeout=None):
        self.calls.append("wait_ready")

    def stop(self, timeout=None):
        self.calls.append("stop")
        self._running = False

    def is_running(self):
        return self._running


@pytest.fixture
def mocked_runtime(monkeypatch):
    """Replace every external dependency of run_experiment with a
    recording stub. Returns a dict of the stubs so tests can inspect
    and override behavior.
    """
    state = {
        "launcher": FakeLauncher(),
        "restore_calls": [],      # list of (label, preserve)
        "snapshot_calls": [],     # list of labels
        "sessions_opened": 0,
        "resolved": {},           # goal_ref → goal_id (populated by tests)
        "run_trial_calls": [],    # list of (goal_id, steps, trial_context)
        "run_trial_result": None, # default result; tests can override
    }

    def fake_make_launcher(scenario=None, character=None):
        return state["launcher"]

    def fake_open_session():
        state["sessions_opened"] += 1
        # Return a dummy with a close() that does nothing.
        class _S:
            def close(self):
                pass
        return _S()

    def fake_restore(label, character=None, preserve=None):
        state["restore_calls"].append(
            (label, list(preserve) if preserve else [])
        )

    def fake_snapshot(label, character=None):
        state["snapshot_calls"].append(label)

    def fake_resolve(session, character, goal_ref, timeout=3.0):
        gid = state["resolved"].get(goal_ref)
        if gid is None:
            return None
        return {"goal_id": gid, "name": goal_ref}

    def fake_run_trial(goal_id, *, steps, session, character, trial_context,
                       run_timeout):
        state["run_trial_calls"].append(
            {"goal_id": goal_id, "steps": list(steps), "trial_context": dict(trial_context)}
        )
        default = {
            "trial_context": dict(trial_context),
            "goal_id": goal_id,
            "steps": [{"kind": s["kind"], "error": None} for s in steps],
            "error": None,
            "started_at": "2026-04-11T10:00:00",
            "elapsed_ms": 1234,
        }
        if state["run_trial_result"] is not None:
            default.update(state["run_trial_result"])
        return default

    monkeypatch.setattr(experiment, "_make_launcher", fake_make_launcher)
    monkeypatch.setattr(experiment, "_open_session", fake_open_session)
    monkeypatch.setattr(snapshot, "restore", fake_restore)
    monkeypatch.setattr(snapshot, "snapshot", fake_snapshot)
    monkeypatch.setattr(harvester, "resolve_goal_handle", fake_resolve)
    monkeypatch.setattr(experiment, "run_trial", fake_run_trial)

    return state


# ──────────────────────────────────────────────────────────────────────
# Runner tests
# ──────────────────────────────────────────────────────────────────────

def test_run_experiment_trial_A_happy_path(mocked_runtime, tmp_path):
    mocked_runtime["resolved"] = {"G01": "goal_17", "G02": "goal_18"}
    plan = build_trial_A(["G01", "G02"], seed=42)
    out = tmp_path / "a.jsonl"

    records = run_experiment(plan, out_path=out)

    assert len(records) == 2
    assert all(r.get("error") is None for r in records)

    # Each goal got a full-baseline restore followed by a launcher bounce.
    assert mocked_runtime["restore_calls"] == [
        ("baseline-clean", []),
        ("baseline-clean", []),
    ]
    # start/wait_ready called once per goal, stop before each reset and
    # at the end.
    assert mocked_runtime["launcher"].calls.count("start") == 2
    assert mocked_runtime["launcher"].calls.count("wait_ready") == 2

    # run_trial received the resolved goal_id, not the name.
    assert [c["goal_id"] for c in mocked_runtime["run_trial_calls"]] == \
        [mocked_runtime["resolved"][g] for g in plan["goals"]]

    # JSONL sidecar contains two lines and a summary file exists.
    written = out.read_text(encoding="utf-8").strip().splitlines()
    assert len(written) == 2
    for line in written:
        rec = json.loads(line)
        assert rec["batch_runner"]["reset_label"] == "baseline-clean"
        assert rec["batch_runner"]["reset_preserve"] == []
    summary_path = out.with_name(out.name + ".summary.json")
    assert summary_path.exists()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["summary"]["trial_type"] == TRIAL_A
    assert summary["summary"]["goals_attempted"] == 2
    assert summary["summary"]["fatal_error"] is None


def test_run_experiment_trial_B_phase1_no_reset_and_boundary_snapshot(
    mocked_runtime, tmp_path,
):
    # 4 goals, 50/50 split → 2 first-half + 2 second-half.
    goals = ["G01", "G02", "G03", "G04"]
    mocked_runtime["resolved"] = {g: f"goal_{i+1}" for i, g in enumerate(goals)}
    plan = build_trial_B(goals, seed=42, split_ratio=0.5)
    out = tmp_path / "b.jsonl"

    run_experiment(plan, out_path=out)

    first_half = plan["metadata"]["first_half"]
    snap_label = plan["metadata"]["post_firsthalf_snapshot"]

    # Phase 1: exactly one baseline restore (before goal 0 of first half).
    # Phase 2: one post-firsthalf restore per second-half goal.
    assert mocked_runtime["restore_calls"] == [
        ("baseline-clean", []),
        (snap_label, []),
        (snap_label, []),
    ]
    # Exactly one snapshot call for the phase boundary (no post_trial).
    assert mocked_runtime["snapshot_calls"] == [snap_label]

    # Launcher lifecycle sanity: at least one start per reset + one start
    # after the phase-boundary snapshot. We don't check exact ordering
    # beyond the count — the restore_calls ordering above already proves
    # the reset strategy.
    starts = mocked_runtime["launcher"].calls.count("start")
    assert starts == 4  # 1 (phase1 start) + 1 (after boundary snapshot) + 2 (phase2 resets)

    # Each first-half goal saw the 3-step protocol; each second-half goal
    # saw [replan]. first_half flag stamped on trial_context.
    calls = mocked_runtime["run_trial_calls"]
    assert len(calls) == 4
    for c in calls:
        goal_ref = c["trial_context"]["goal_ref"]
        if goal_ref in first_half:
            assert len(c["steps"]) == 3
            assert c["trial_context"]["first_half"] is True
        else:
            assert len(c["steps"]) == 1
            assert c["trial_context"]["first_half"] is False


def test_run_experiment_per_goal_failure_continues(mocked_runtime, tmp_path):
    mocked_runtime["resolved"] = {"G01": "goal_17", "G03": "goal_19"}
    # Intentionally omit G02 → resolve returns None → synthetic failure.
    plan = build_trial_A(["G01", "G02", "G03"], seed=42)
    out = tmp_path / "a.jsonl"

    records = run_experiment(plan, out_path=out)

    assert len(records) == 3
    errors = [r.get("error") for r in records]
    # Exactly one record errored (the unresolved one); the other two are clean.
    assert sum(1 for e in errors if e is not None) == 1
    bad = next(r for r in records if r.get("error"))
    assert "could not resolve goal ref 'G02'" in bad["error"]
    assert bad["goal_id"] is None
    # Summary reflects the error without marking the run fatal.
    summary_path = out.with_name(out.name + ".summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))["summary"]
    assert summary["fatal_error"] is None
    assert summary["errored_records"] == 1
    assert summary["goals_attempted"] == 3


def test_run_experiment_fatal_launcher_failure_aborts(mocked_runtime, tmp_path):
    mocked_runtime["resolved"] = {"G01": "goal_17", "G02": "goal_18", "G03": "goal_19"}
    plan = build_trial_A(["G01", "G02", "G03"], seed=42)

    # Make wait_ready explode on the 2nd call (i.e. after goal 0 succeeds).
    orig_wait_ready = mocked_runtime["launcher"].wait_ready
    call_count = {"n": 0}
    def flaky_wait_ready(timeout=None):
        call_count["n"] += 1
        if call_count["n"] == 2:
            raise RuntimeError("agent never became ready")
        orig_wait_ready(timeout=timeout)
    mocked_runtime["launcher"].wait_ready = flaky_wait_ready

    out = tmp_path / "a.jsonl"
    records = run_experiment(plan, out_path=out)

    # First goal ran. Second goal's reset failed. Third never attempted.
    assert len(records) == 1
    assert records[0].get("error") is None
    summary = json.loads(
        out.with_name(out.name + ".summary.json").read_text(encoding="utf-8"),
    )["summary"]
    assert summary["fatal_error"] is not None
    assert "agent never became ready" in summary["fatal_error"]
    assert summary["goals_attempted"] == 1


def test_read_reviewed_from_jsonl_extracts_first_half(tmp_path):
    path = tmp_path / "b.jsonl"
    rows = [
        {"trial_context": {"trial_type": "B", "goal_ref": "G03", "first_half": True}},
        {"trial_context": {"trial_type": "B", "goal_ref": "G07", "first_half": True}},
        {"trial_context": {"trial_type": "B", "goal_ref": "G01", "first_half": False}},
        # Duplicate should be deduped.
        {"trial_context": {"trial_type": "B", "goal_ref": "G03", "first_half": True}},
        # Wrong trial type should be ignored.
        {"trial_context": {"trial_type": "A", "goal_ref": "G05", "first_half": True}},
    ]
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
        # Add a malformed line to prove it's skipped, not fatal.
        f.write("not-valid-json\n")

    reviewed = read_reviewed_from_jsonl(path)
    assert reviewed == ["G03", "G07"]
