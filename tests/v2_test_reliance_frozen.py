"""A reliance statement frozen from another engagement is not rewritten.

measure/full_run.py copies one engagement's statement into a comparison
engagement so that every model's tiers rest on the same statement; the
enumerate job then runs reliance.py, which must leave it alone and call no
model. Without the check in `run`, the copy is overwritten by a fresh
statement and the comparison measures the reliance step as well as the tiers.
"""
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import engagement_state as state                 # noqa: E402
from workflowsv2.claims_audit import reliance                      # noqa: E402
from workflowsv2.claims_audit import runner                        # noqa: E402

STATEMENT = {"at": "2026-09-20T23-54-59Z", "model": "m", "use": "The buyer runs it.",
             "items": [{"item": "Shortening", "reliance": "depends", "if_it_failed": "The plan ends.",
                        "source": "buyer", "buyer_words": "a link shortener"}]}


def _engagement(root: Path, name: str) -> Path:
    d = root / name
    (d / "target").mkdir(parents=True)
    (d / "engagement.yaml").write_text("target: target\nclaim_sources:\n  - README.md\n", encoding="utf-8")
    (d / "state.json").write_text("{}\n", encoding="utf-8")
    (d / "brief.md").write_text("A brief.\n", encoding="utf-8")
    return d


def test_a_frozen_reliance_statement_is_kept_and_no_model_is_called(tmp_path, monkeypatch):
    root = tmp_path / "engagements"
    src = _engagement(root, "base")
    (src / state.SURFACE).mkdir()
    (src / state.SURFACE / reliance.RECORD).write_text(json.dumps(STATEMENT), encoding="utf-8")
    dst = _engagement(root, "cmp")
    monkeypatch.setattr(state, "ENGAGEMENTS", root)
    monkeypatch.setattr(runner, "ENGAGEMENTS", root)

    frozen = reliance.freeze_from(src, dst, "base")
    assert frozen["frozen_from"] == "base" and frozen["items"] == STATEMENT["items"]

    def no_model(_path):
        raise AssertionError("a frozen statement must not reach the model")
    monkeypatch.setattr(reliance, "backend_from_model", no_model)
    got = reliance.run("cmp", Path("unused.yaml"))
    assert got == reliance.load(dst) == dict(STATEMENT, frozen_from="base")


def test_freezing_from_an_engagement_without_a_statement_refuses(tmp_path):
    src = _engagement(tmp_path, "base")
    dst = _engagement(tmp_path, "cmp")
    with pytest.raises(SystemExit, match="no reliance statement"):
        reliance.freeze_from(src, dst, "base")
