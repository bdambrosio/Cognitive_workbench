"""The intake and post-delivery sessions: the runner logic behind one object
the terminal and the browser both drive. Constructed here without a
ChatLoop, by bypassing __init__ and supplying a fake loop."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from workflowsv2.intake import schemas as sch                    # noqa: E402
from workflowsv2.intake.session import IntakeSession             # noqa: E402


class _Store:
    def __init__(self):
        self.turns = {"User": [], "Practice": []}

    def get_recent_turns(self, entity, limit=20, scope="all"):
        return self.turns[entity][-limit:]


class _Loop:
    """Records what it was told and answers with a fixed reply."""
    def __init__(self):
        self.store = _Store()
        self.told = []
        self.notes = []

    def _process_user_turn(self, source, text, close):
        self.told.append((source, text))
        self.store.turns[source] += [{"direction": "in", "text": text},
                                     {"direction": "out", "text": f"reply to {source}"}]

    class _Ex:
        def shutdown(self, wait=True):
            pass
    _post_turn_executor = _Ex()

    def _persist_to_disk(self):
        self.persisted = True


def _session(tmp_path, monkeypatch):
    s = IntakeSession.__new__(IntakeSession)
    s.engagement = "e"; s.eng_dir = tmp_path / "e"; s.eng_dir.mkdir()
    s.intake_id = "I1"; s.intake_dir = s.eng_dir / "intakes" / "I1"; s.intake_dir.mkdir(parents=True)
    s.form_path = s.intake_dir / "intake.json"; s.form = sch.empty_form()
    s.max_tokens = 100; s.world = "w"; s.returning = False; s.name = "Jill"
    s.loop = _Loop(); s.method_text = "METHOD"; s.transcript = []; s.turns = 0; s.opened = False
    from workflowsv2.intake import runner as rn
    monkeypatch.setattr(rn, "mirror_note", lambda loop, form: loop.notes.append(form))

    def emit(loop, system, user, schema, max_tokens):
        f = json.loads(json.dumps(s.form)); f["identify"]["client"] = "Acme"
        return {"obj": f, "parse": "ok"}
    monkeypatch.setattr("workflowsv2.intake.session.emit", emit)
    return s


def test_open_turn_history_and_document(tmp_path, monkeypatch):
    s = _session(tmp_path, monkeypatch)
    assert s.open() == "reply to Practice" and s.loop.told[0][0] == "Practice"
    res = s.turn("we are Acme")
    assert res["reply"] == "reply to User"
    assert s.loop.told[1][1].startswith("we are Acme\n\n[form:")      # the ledger rides the turn
    assert res["form"]["identify"]["client"] == "Acme" and res["check"]["filled"] == 1
    assert json.loads(s.form_path.read_text())["identify"]["client"] == "Acme"
    assert s.loop.notes[-1]["identify"]["client"] == "Acme"             # mirrored
    h = s.history()
    assert h[0] == {"who": "client", "text": "we are Acme"}               # ledger stripped
    assert h[1] == {"who": "agent", "text": "reply to User"}
    d = s.document()
    assert d["kind"] == "form" and d["slots"]["identify"][0] == "client"
    assert Path(d["uploads_dir"]).is_dir()
    s.close()
    assert s.loop.persisted


def test_history_shows_the_greeting_before_any_client_turn(tmp_path, monkeypatch):
    s = _session(tmp_path, monkeypatch)
    s.open()
    assert s.history() == [{"who": "agent", "text": "reply to Practice"}]


def test_post_session_findings_for_the_evidence_pane(tmp_path):
    from workflowsv2.claims_audit.post_session import PostSession
    s = PostSession.__new__(PostSession)
    s.engagement = "e"; s.intake_id = "I1"; s.merged_dir = tmp_path
    s.merged = {"findings": [
        {"claim_source": "README.md", "claim_id": 20, "quote": "q", "lines": [53, 53],
         "statement": "st", "adjudication": {"verdict": "partial", "gap": "g"},
         "review": {"outcome": "holds"},
         "evidence": [{"form": "citation", "document": "a.py", "lines": [1, 2], "quote": "x", "shows": "y", "extra": 1},
                      {"form": "search", "kind": "lexical", "performed": "p", "result": "r", "candidates": []},
                      {"form": "derived", "basis": [], "derivation": "d", "consequence": "c"}]},
        {"claim_source": "README.md", "claim_id": 2, "quote": "q2", "lines": [4, 4],
         "adjudication": {"verdict": "unverifiable", "unresolved_because": "outside_the_materials"},
         "evidence": []}]}
    f = s.findings()
    assert set(f) == {"README.md#20", "README.md#2"}
    assert f["README.md#20"]["verdict"] == "partial" and f["README.md#20"]["review"] == "holds"
    assert [e["form"] for e in f["README.md#20"]["evidence"]] == ["citation", "search", "derived"]
    assert "extra" not in f["README.md#20"]["evidence"][0]
    assert f["README.md#2"]["unresolved_because"] == "outside_the_materials" and f["README.md#2"]["review"] is None
