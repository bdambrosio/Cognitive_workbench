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
DEFAULT_BASELINE = "baseline-empty"


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
    assert plan_a["goals"] != plan_c["goals"]

    assert plan_a["trial_type"] == TRIAL_A
    assert plan_a["seed"] == 42
    assert plan_a["name"] == "trial-A-seed-42"
    assert set(plan_a["goals"]) == set(goals)
    for g in goals:
        assert plan_a["per_goal_protocol"][g] == [{"kind": "run", "mode": "replan"}]
        # Every Trial A reset is a full restore of the empty baseline with
        # no overlays — no carryover between goals.
        assert plan_a["per_goal_reset"][g] == {
            "label": DEFAULT_BASELINE,
            "overlays": [],
        }
        assert plan_a["per_goal_capture"][g] == []


def test_build_trial_B_overlay_accumulation():
    goals = ["G01", "G02", "G03", "G04", "G05", "G06"]
    plan = build_trial_B(goals, seed=42, split_ratio=0.5, experiment_dir="/tmp/tb")

    assert plan["trial_type"] == TRIAL_B
    assert plan["name"] == "trial-B-seed-42"

    first_half = plan["metadata"]["first_half"]
    second_half = plan["metadata"]["second_half"]
    assert len(first_half) == 3
    assert len(second_half) == 3
    assert set(first_half) | set(second_half) == set(goals)

    # Every reset targets the empty baseline.
    for g in goals:
        assert plan["per_goal_reset"][g]["label"] == DEFAULT_BASELINE

    # Phase 1 goal 0: no overlays (fresh start of the accumulating chain).
    g0 = first_half[0]
    assert plan["per_goal_reset"][g0]["overlays"] == []
    assert plan["per_goal_protocol"][g0] == [
        {"kind": "run",    "mode": "replan"},
        {"kind": "review", "commit": True},
        {"kind": "run",    "mode": "replay"},
    ]

    # Phase 1 goal 1: world_model from after goal 0 + cached_plan for goal 0.
    g1 = first_half[1]
    ov1 = plan["per_goal_reset"][g1]["overlays"]
    assert {"op": "world_model", "source": "/tmp/tb/world_model_after_0.json"} in ov1
    assert {
        "op": "cached_plan",
        "goal_ref": g0,
        "source": f"/tmp/tb/cached_plan_{g0}.json",
    } in ov1
    assert len(ov1) == 2

    # Phase 1 goal 2: world_model from after goal 1 + cached plans for 0..1.
    g2 = first_half[2]
    ov2 = plan["per_goal_reset"][g2]["overlays"]
    wm_ops = [o for o in ov2 if o["op"] == "world_model"]
    cp_ops = [o for o in ov2 if o["op"] == "cached_plan"]
    assert len(wm_ops) == 1
    assert wm_ops[0]["source"] == "/tmp/tb/world_model_after_1.json"
    assert {o["goal_ref"] for o in cp_ops} == {g0, g1}

    # Phase 2 goals: empty baseline + final phase-1 world_model only.
    final_wm = "/tmp/tb/world_model_after_2.json"
    for g in second_half:
        assert plan["per_goal_protocol"][g] == [{"kind": "run", "mode": "replan"}]
        assert plan["per_goal_reset"][g]["overlays"] == [
            {"op": "world_model", "source": final_wm},
        ]
        # Second-half goals don't capture anything — they don't contribute
        # to the accumulating state (no review commit in their protocol).
        assert plan["per_goal_capture"][g] == []

    # Each phase-1 goal captures world_model + its own cached_plan.
    for i, g in enumerate(first_half):
        caps = plan["per_goal_capture"][g]
        cap_ops = {c["op"] for c in caps}
        assert "world_model" in cap_ops
        assert "cached_plan" in cap_ops
        wm_cap = next(c for c in caps if c["op"] == "world_model")
        assert wm_cap["dest"] == f"/tmp/tb/world_model_after_{i}.json"


def test_build_trial_C_mode_and_overlay_selection():
    goals = ["G01", "G02", "G03", "G04", "G05"]
    reviewed = ["G02", "G04"]
    plan = build_trial_C(
        goals, seed=42,
        reviewed_in_b=reviewed,
        final_world_model_path="/tmp/tb/world_model_after_2.json",
        cached_plan_paths={
            "G02": "/tmp/tb/cached_plan_G02.json",
            "G04": "/tmp/tb/cached_plan_G04.json",
        },
    )

    assert plan["trial_type"] == TRIAL_C

    for g in goals:
        ov = plan["per_goal_reset"][g]["overlays"]
        assert plan["per_goal_reset"][g]["label"] == DEFAULT_BASELINE
        # Every goal gets the final world_model overlay.
        assert any(
            o["op"] == "world_model"
            and o["source"] == "/tmp/tb/world_model_after_2.json"
            for o in ov
        )
        if g in reviewed:
            assert plan["per_goal_protocol"][g] == [{"kind": "run", "mode": "replay"}]
            # Reviewed goals get a cached_plan overlay targeting this goal.
            assert any(
                o["op"] == "cached_plan" and o["goal_ref"] == g for o in ov
            )
        else:
            assert plan["per_goal_protocol"][g] == [{"kind": "run", "mode": "replan"}]
            # Unreviewed goals only get the world_model overlay.
            assert all(o["op"] != "cached_plan" for o in ov)


def test_build_trial_C_rejects_missing_cached_plan_paths():
    with pytest.raises(ValueError, match="no cached_plan_paths"):
        build_trial_C(
            ["G01", "G02"], seed=42,
            reviewed_in_b=["G02"],
            final_world_model_path="/tmp/wm.json",
            cached_plan_paths={},  # missing G02
        )


# ──────────────────────────────────────────────────────────────────────
# Runner fakes
# ──────────────────────────────────────────────────────────────────────

class FakeLauncher:
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
    state = {
        "launcher": FakeLauncher(),
        "restore_calls": [],            # list of (label, world)
        "world_model_overlays": [],     # list of source paths
        "cached_plan_overlays": [],     # list of (goal_id, source path)
        "world_model_captures": [],     # list of dest paths
        "cached_plan_captures": [],     # list of (goal_id, dest path)
        "sessions_opened": 0,
        "resolved": {},                 # goal_ref → goal_id
        "run_trial_calls": [],
        "run_trial_result": None,
    }

    def fake_make_launcher(scenario=None, character=None):
        return state["launcher"]

    def fake_open_session():
        state["sessions_opened"] += 1
        class _S:
            def close(self):
                pass
        return _S()

    def fake_restore(label, character=None, world=None, force=False):
        state["restore_calls"].append((label, world))

    def fake_world_model_overlay(source, character=None, world=None, force=False):
        state["world_model_overlays"].append(str(source))

    def fake_cached_plan_overlay(goal_id, source, character=None, world=None, force=False):
        state["cached_plan_overlays"].append((goal_id, str(source)))

    def fake_world_model_capture(dest, character=None, world=None):
        state["world_model_captures"].append(str(dest))

    def fake_cached_plan_capture(goal_id, dest, character=None, world=None):
        state["cached_plan_captures"].append((goal_id, str(dest)))

    def fake_resolve(session, character, goal_ref, timeout=3.0):
        gid = state["resolved"].get(goal_ref)
        if gid is None:
            return None
        return {"goal_id": gid, "name": goal_ref}

    def fake_run_trial(goal_id, *, steps, session, character, trial_context, run_timeout):
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

    def fake_resolve_map_from_resources(*, character, world):
        # Tests pre-populate `resolved` with goal_ref → goal_id mappings
        # for the goals they care about. Return the same map here so
        # overlay application can translate refs to ids.
        return dict(state["resolved"])

    def fake_resolve_map_live(session, character):
        # Live (session-based) variant used for captures after a goal
        # run — same source of truth as the offline variant above.
        return dict(state["resolved"])

    monkeypatch.setattr(experiment, "_make_launcher", fake_make_launcher)
    monkeypatch.setattr(experiment, "_open_session", fake_open_session)
    monkeypatch.setattr(snapshot, "restore", fake_restore)
    monkeypatch.setattr(snapshot, "apply_world_model_overlay", fake_world_model_overlay)
    monkeypatch.setattr(snapshot, "apply_cached_plan_overlay", fake_cached_plan_overlay)
    monkeypatch.setattr(snapshot, "capture_world_model", fake_world_model_capture)
    monkeypatch.setattr(snapshot, "capture_cached_plan", fake_cached_plan_capture)
    monkeypatch.setattr(harvester, "resolve_goal_handle", fake_resolve)
    monkeypatch.setattr(experiment, "run_trial", fake_run_trial)
    monkeypatch.setattr(
        experiment, "_resolve_goal_ref_map_from_resources",
        fake_resolve_map_from_resources,
    )
    monkeypatch.setattr(
        experiment, "_resolve_goal_ref_map",
        fake_resolve_map_live,
    )
    monkeypatch.setattr(
        experiment, "_preflight_shutdown_external_launcher",
        lambda character, **kw: True,
    )
    # The save-before-capture trigger is a live-agent Zenoh operation
    # that tests don't want to run. Short-circuit it to return True.
    monkeypatch.setattr(
        experiment, "_trigger_agent_save_and_wait",
        lambda session, character, world, **kw: True,
    )
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

    # Each goal got a full restore of baseline-empty. No overlays.
    labels = [c[0] for c in mocked_runtime["restore_calls"]]
    assert labels == ["baseline-empty", "baseline-empty"]
    assert mocked_runtime["world_model_overlays"] == []
    assert mocked_runtime["cached_plan_overlays"] == []

    # start/wait_ready called once per goal.
    assert mocked_runtime["launcher"].calls.count("start") == 2
    assert mocked_runtime["launcher"].calls.count("wait_ready") == 2

    # run_trial received the resolved goal_id for each goal.
    assert [c["goal_id"] for c in mocked_runtime["run_trial_calls"]] == \
        [mocked_runtime["resolved"][g] for g in plan["goals"]]

    # JSONL + summary file written.
    written = out.read_text(encoding="utf-8").strip().splitlines()
    assert len(written) == 2
    for line in written:
        rec = json.loads(line)
        assert rec["batch_runner"]["reset_label"] == "baseline-empty"
        assert rec["batch_runner"]["overlays_applied"] == []
    summary_path = out.with_name(out.name + ".summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))["summary"]
    assert summary["trial_type"] == TRIAL_A
    assert summary["fatal_error"] is None
    assert summary["goals_attempted"] == 2


def test_run_experiment_trial_B_overlays_and_captures(mocked_runtime, tmp_path):
    goals = ["G01", "G02", "G03", "G04"]
    mocked_runtime["resolved"] = {g: f"goal_{i+1}" for i, g in enumerate(goals)}
    exp_dir = tmp_path / "tb"
    plan = build_trial_B(
        goals, seed=42, split_ratio=0.5, experiment_dir=str(exp_dir),
    )
    out = tmp_path / "b.jsonl"

    run_experiment(plan, out_path=out)

    first_half = plan["metadata"]["first_half"]
    second_half = plan["metadata"]["second_half"]

    # Every goal (4 total) triggered a baseline-empty restore.
    labels = [c[0] for c in mocked_runtime["restore_calls"]]
    assert labels == ["baseline-empty"] * 4

    # Phase 1 goal 0: no overlays (no prior state to carry). Phase 1
    # goal 1: 1 world_model + 1 cached_plan (for goal 0). Phase 1 goal 2:
    # no — wait, this is a 2+2 split on 4 goals.
    # Actually with 4 goals and split_ratio=0.5 → split=2, first half=2.
    assert len(first_half) == 2
    assert len(second_half) == 2

    # Check overlay sequence:
    # Goal 0 (phase 1): 0 overlays.
    # Goal 1 (phase 1): 1 wm + 1 cp.
    # Goal 2 (phase 2, first second-half): 1 wm (final).
    # Goal 3 (phase 2, second second-half): 1 wm (final).
    assert len(mocked_runtime["world_model_overlays"]) == 3  # 1 + 1 + 1
    assert len(mocked_runtime["cached_plan_overlays"]) == 1  # phase 1 goal 1

    cp_overlay = mocked_runtime["cached_plan_overlays"][0]
    g0_id = mocked_runtime["resolved"][first_half[0]]
    assert cp_overlay[0] == g0_id

    # Captures: phase 1 goals each captured world_model + cached_plan.
    # Phase 2 captured nothing.
    assert len(mocked_runtime["world_model_captures"]) == 2
    assert len(mocked_runtime["cached_plan_captures"]) == 2

    # run_trial protocols match the trial type.
    calls = mocked_runtime["run_trial_calls"]
    assert len(calls) == 4
    for c in calls:
        goal_ref = c["trial_context"]["goal_ref"]
        if goal_ref in first_half:
            assert len(c["steps"]) == 3  # replan + review + replay
            assert c["trial_context"]["first_half"] is True
        else:
            assert len(c["steps"]) == 1  # replan only
            assert c["trial_context"]["first_half"] is False


def test_run_experiment_per_goal_failure_continues(mocked_runtime, tmp_path):
    mocked_runtime["resolved"] = {"G01": "goal_17", "G03": "goal_19"}
    # G02 missing → resolve returns None → synthetic failure.
    plan = build_trial_A(["G01", "G02", "G03"], seed=42)
    out = tmp_path / "a.jsonl"

    records = run_experiment(plan, out_path=out)

    assert len(records) == 3
    errs = [r.get("error") for r in records]
    assert sum(1 for e in errs if e is not None) == 1
    bad = next(r for r in records if r.get("error"))
    assert "could not resolve goal ref 'G02'" in bad["error"]
    assert bad["goal_id"] is None
    summary = json.loads(
        out.with_name(out.name + ".summary.json").read_text(encoding="utf-8"),
    )["summary"]
    assert summary["fatal_error"] is None
    assert summary["errored_records"] == 1
    assert summary["goals_attempted"] == 3


def test_run_experiment_fatal_launcher_failure_aborts(mocked_runtime, tmp_path):
    mocked_runtime["resolved"] = {"G01": "goal_17", "G02": "goal_18", "G03": "goal_19"}
    plan = build_trial_A(["G01", "G02", "G03"], seed=42)

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

    assert len(records) == 1
    assert records[0].get("error") is None
    summary = json.loads(
        out.with_name(out.name + ".summary.json").read_text(encoding="utf-8"),
    )["summary"]
    assert summary["fatal_error"] is not None
    assert "agent never became ready" in summary["fatal_error"]
    assert summary["goals_attempted"] == 1


def test_run_experiment_preflight_aborts_when_external_launcher_refuses_shutdown(
    mocked_runtime, monkeypatch, tmp_path,
):
    monkeypatch.setattr(
        experiment, "_preflight_shutdown_external_launcher",
        lambda character, **kw: False,
    )
    plan = build_trial_A(["G01", "G02"], seed=42)
    out = tmp_path / "a.jsonl"

    records = run_experiment(plan, out_path=out)

    assert records == []
    assert mocked_runtime["restore_calls"] == []
    assert mocked_runtime["run_trial_calls"] == []

    summary = json.loads(
        out.with_name(out.name + ".summary.json").read_text(encoding="utf-8"),
    )["summary"]
    assert summary["fatal_error"] is not None
    assert "preflight" in summary["fatal_error"]


def test_read_reviewed_from_jsonl_extracts_first_half(tmp_path):
    path = tmp_path / "b.jsonl"
    rows = [
        {"trial_context": {"trial_type": "B", "goal_ref": "G03", "first_half": True}},
        {"trial_context": {"trial_type": "B", "goal_ref": "G07", "first_half": True}},
        {"trial_context": {"trial_type": "B", "goal_ref": "G01", "first_half": False}},
        {"trial_context": {"trial_type": "B", "goal_ref": "G03", "first_half": True}},
        {"trial_context": {"trial_type": "A", "goal_ref": "G05", "first_half": True}},
    ]
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
        f.write("not-valid-json\n")

    reviewed = read_reviewed_from_jsonl(path)
    assert reviewed == ["G03", "G07"]
