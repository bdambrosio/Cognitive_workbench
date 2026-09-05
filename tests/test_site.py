"""The client site over a temporary engagements directory, with no access
check (identity from ?as=) and fake sessions: roles, the letter, the intake
finish, the surface, the practice buttons, the release gate on the report."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient                       # noqa: E402

from workflowsv2 import engagement_state as st                  # noqa: E402
from client_ui import jobs, mail, site                          # noqa: E402
from client_ui.access import Access                             # noqa: E402

PRACTICE = "bruce@example.test"
CLIENT = "client@example.test"
OTHER = "other@example.test"


class FakeSession:
    def __init__(self, key, eng_dir):
        self.key = key
        self.eng_dir = eng_dir
        self.intake_dir = eng_dir / "intakes" / "I1"
        self.intake_dir.mkdir(parents=True, exist_ok=True)
        self.form = {"identify": {"client": "Acme"}}
        self.hist = []

    def open(self):
        self.hist.append({"who": "agent", "text": "hello"}); return "hello"

    def turn(self, text):
        self.hist += [{"who": "client", "text": text}, {"who": "agent", "text": "got: " + text}]
        return {"reply": "got: " + text}

    def history(self):
        return list(self.hist)

    def document(self):
        if self.key[0] == "intake":
            return {"kind": "form", "engagement": self.key[1], "form": self.form,
                    "check": {"empty": {}}, "ledger": "", "slots": {}, "uploads_dir": str(self.eng_dir / "uploads")}
        return {"kind": "report", "engagement": self.key[1], "html": "<p>r</p>", "findings": {}, "banner": "b"}

    def uploads_dir(self):
        d = self.eng_dir / "uploads"; d.mkdir(exist_ok=True); return d

    def close(self):
        pass


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    monkeypatch.setattr(site, "LETTER_TEMPLATE", tmp_path / "LETTER_TEMPLATE.md")
    (tmp_path / "LETTER_TEMPLATE.md").write_text("# Letter\n\nTerms **here**.\n")
    monkeypatch.setenv("MAIL_DRY_RUN", "1")
    monkeypatch.setenv("PRACTICE_EMAILS", PRACTICE)
    mail.sent.clear()
    jobs._steps_for = None
    access = Access([PRACTICE], no_access=True)
    app = site.make_site_app(access, None, build=lambda key: FakeSession(key, tmp_path / key[1]))
    c = TestClient(app)
    yield c, tmp_path
    jobs._steps_for = None


def _as(email):
    return f"?as={email}"


def _new(c, name="e1", emails=(CLIENT,)):
    r = c.post("/p/api/engagements" + _as(PRACTICE), json={"name": name, "client_emails": list(emails)})
    assert r.status_code == 200, r.text
    return r.json()


def test_roles_and_the_front_door(env):
    c, root = env
    _new(c)
    _new(c, "e2", emails=(OTHER,))
    assert c.get("/", follow_redirects=False).status_code == 403
    assert c.get("/" + _as(PRACTICE), follow_redirects=False).headers["location"] == "/p/"
    assert c.get("/" + _as(CLIENT), follow_redirects=False).headers["location"] == "/e/e1/"
    assert c.get("/e/e1/api/status" + _as(CLIENT)).json()["role"] == "client"
    assert c.get("/e/e2/api/status" + _as(CLIENT)).status_code == 403       # not their engagement
    assert c.get("/e/e1/api/status" + _as(OTHER)).status_code == 403
    assert c.get("/e/e1/api/status" + _as(PRACTICE)).json()["role"] == "practice"
    assert c.get("/p/api/engagements" + _as(CLIENT)).status_code == 403
    assert c.get("/e/nope/api/status" + _as(PRACTICE)).status_code == 404
    # the welcome went to the client
    assert mail.sent[0]["to"] == [CLIENT] and "/e/e1/" in mail.sent[0]["body"]


def test_letter_then_intake_then_finish(env):
    c, root = env
    _new(c)
    s = c.get("/e/e1/api/status" + _as(CLIENT)).json()
    assert s["next"]["stage"] == "letter" and "Terms **here**" in s["letter"]
    s = c.post("/e/e1/api/letter/accept" + _as(CLIENT)).json()
    assert s["stages"]["letter"]["by"] == CLIENT and s["next"]["stage"] == "intake"
    # the intake page and its finish
    assert c.get("/e/e1/intake/" + _as(CLIENT)).status_code == 200
    d = c.get("/e/e1/intake/api/document" + _as(CLIENT)).json()
    assert d["finish"] == {"allowed": True, "done": False, "empty": {}}
    with c.websocket_connect("/e/e1/intake/ws" + _as(CLIENT)) as ws:
        assert [ws.receive_json()["type"] for _ in range(3)] == ["history", "document", "status"]
        assert ws.receive_json()["state"] == "thinking"                 # the opening, once
        assert ws.receive_json() == {"type": "say", "text": "hello"}
        ws.receive_json()
        ws.send_json({"type": "turn", "text": "Acme buys Widget"})
        got = [ws.receive_json() for _ in range(5)]
        assert {"type": "say", "text": "got: Acme buys Widget"} in got
    mail.sent.clear()
    r = c.post("/e/e1/intake/api/finish" + _as(CLIENT))
    assert r.status_code == 200 and r.json()["next"]["stage"] == "materials"
    assert (root / "e1" / "intakes" / "I1" / "intake_meta.json").is_file()
    assert st.stage_value(root / "e1", "intake") == "done"
    assert mail.sent[-1]["to"] == [PRACTICE] and "intake finished" in mail.sent[-1]["subject"]
    d = c.get("/e/e1/intake/api/document" + _as(CLIENT)).json()
    assert d["finish"]["done"] is True


def _enumerated(root, name, source, claims):
    eng = root / name
    run = eng / "runs" / f"2026-09-05T00-00-00Z_enum_{name}_{jobs.slug(source)}_T"
    run.mkdir(parents=True)
    (run / "claims.json").write_text(json.dumps({"claim_source": source, "claims": claims}))
    (eng / "engagement.yaml").write_text(
        f"target: target\nclaim_sources: [{source}]\nclient_emails: ['{CLIENT}']\n")


def test_surface_comment_edit_and_freeze(env):
    c, root = env
    _new(c)
    claims = [{"id": 1, "lines": [1, 1], "quote": "q1", "statement": "s1", "about": "target"},
              {"id": 2, "lines": [2, 3], "quote": "q2", "statement": "s2", "about": "target"}]
    _enumerated(root, "e1", "README.md", claims)
    s = c.get("/e/e1/surface/api" + _as(CLIENT)).json()
    assert s["editable"] is False
    src = s["sources"][0]
    assert src["origin"] == "enumeration" and len(src["claims"]) == 2 and not src["frozen"]
    r = c.post("/e/e1/surface/api/comment" + _as(CLIENT), json={"source": "README.md", "claim_id": 2, "text": "not a claim"})
    assert r.status_code == 200 and r.json()["comments"][0]["by"] == CLIENT
    assert c.post("/e/e1/surface/api/comment" + _as(CLIENT), json={"source": "x.md", "claim_id": 2, "text": "t"}).status_code == 400
    # the practice drops claim 2 and freezes
    p = c.get("/p/surface/e1/api" + _as(PRACTICE)).json()
    assert p["editable"] is True and p["sources"][0]["comments"][0]["text"] == "not a claim"
    r = c.post("/p/surface/e1/api/draft" + _as(PRACTICE), json={"source": "README.md", "claims": claims[:1]})
    assert r.json()["origin"] == "draft" and len(r.json()["claims"]) == 1
    mail.sent.clear()
    r = c.post("/p/surface/e1/api/freeze" + _as(PRACTICE), json={"source": "README.md"})
    assert r.status_code == 200 and r.json()["frozen"] is True
    frozen = json.loads(jobs.surface_file(root / "e1", "README.md").read_text())
    assert frozen == {"claim_source": "README.md", "claims": claims[:1]}
    assert st.stage_value(root / "e1", "surface") == "frozen"
    assert mail.sent[-1]["to"] == [CLIENT] and "frozen" in mail.sent[-1]["subject"]
    # frozen: no more drafts, no second freeze; the client sees the frozen list
    assert c.post("/p/surface/e1/api/draft" + _as(PRACTICE), json={"source": "README.md", "claims": claims}).status_code == 400
    assert c.post("/p/surface/e1/api/freeze" + _as(PRACTICE), json={"source": "README.md"}).status_code == 400
    assert c.get("/e/e1/surface/api" + _as(CLIENT)).json()["sources"][0]["origin"] == "frozen"


def test_practice_buttons_jobs_and_the_lock(env, tmp_path):
    c, root = env
    _new(c)
    eng = root / "e1"
    (eng / "engagement.yaml").write_text(f"target: target\nclaim_sources: [README.md]\nclient_emails: ['{CLIENT}']\n")
    # a stage a button does not set
    assert c.post("/p/api/engagements/e1/stage" + _as(PRACTICE), json={"stage": "letter", "value": "accepted"}).status_code == 400
    r = c.post("/p/api/engagements/e1/stage" + _as(PRACTICE), json={"stage": "materials", "value": "ready"})
    assert r.status_code == 200 and st.stage_value(eng, "materials") == "ready"
    # a job that blocks until released
    release = tmp_path / "go"
    def steps(eng_dir, kind, model, ts):
        yield ("wait", ["sh", "-c", f"while [ ! -f {release} ]; do sleep 0.05; done"])
    jobs._steps_for = steps
    r = c.post("/p/api/engagements/e1/jobs/enumerate" + _as(PRACTICE))
    assert r.status_code == 200 and r.json()["job"]["kind"] == "enumerate"
    e = next(x for x in r.json()["engagements"] if x["name"] == "e1")
    assert e["job"]["kind"] == "enumerate" and e["site"] is True
    assert c.post("/p/api/engagements/e1/jobs/chain" + _as(PRACTICE)).status_code == 409
    assert c.post("/p/api/engagements/e1/jobs/sweep" + _as(PRACTICE)).status_code == 400
    assert c.post("/p/api/engagements/e1/jobs/enumerate" + _as(CLIENT)).status_code == 403
    job_id = r.json()["job"]["id"]
    release.write_text("")
    import time
    for _ in range(100):
        if st.running_job(eng) is None:
            break
        time.sleep(0.05)
    assert st.running_job(eng) is None and st.stage_value(eng, "enumeration") == "done"
    log = c.get(f"/p/api/engagements/e1/jobs/{job_id}/log" + _as(PRACTICE))
    assert log.status_code == 200 and "=== wait" in log.text
    # release needs a report; close needs release
    assert c.post("/p/api/engagements/e1/stage" + _as(PRACTICE), json={"stage": "release", "value": "released"}).status_code == 400


def test_report_is_gated_on_release(env):
    c, root = env
    _new(c)
    eng = root / "e1"
    assert c.get("/e/e1/report/" + _as(CLIENT)).status_code == 404
    assert c.get("/e/e1/report/" + _as(PRACTICE)).status_code == 200      # the practice reads before release
    # a run with a report, then release
    merged = eng / "merged" / "2026-09-05T01-00-00Z_chain_T"
    merged.mkdir(parents=True)
    (merged / "meta.json").write_text(json.dumps({"intake": None}))
    (merged / "report.md").write_text("# r\n")
    mail.sent.clear()
    r = c.post("/p/api/engagements/e1/stage" + _as(PRACTICE), json={"stage": "release", "value": "released"})
    assert r.status_code == 200
    assert mail.sent[-1]["to"] == [CLIENT] and "/e/e1/report/" in mail.sent[-1]["body"]
    assert c.get("/e/e1/report/" + _as(CLIENT)).status_code == 200
    with c.websocket_connect("/e/e1/report/ws" + _as(CLIENT)) as ws:
        assert [ws.receive_json()["type"] for _ in range(3)] == ["history", "document", "status"]
    s = c.get("/e/e1/api/status" + _as(CLIENT)).json()
    assert s["released"] and s["next"]["stage"] == "report"
    r = c.post("/p/api/engagements/e1/stage" + _as(PRACTICE), json={"stage": "closed", "value": "closed"})
    assert r.status_code == 200 and c.get("/e/e1/api/status" + _as(CLIENT)).json()["next"]["who"] == "done"


def test_next_step_order(tmp_path, monkeypatch):
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    eng = st.new_engagement(tmp_path / "e")
    seq = []
    for stage, value in [("letter", "accepted"), ("intake", "done"), ("materials", "ready"),
                         ("enumeration", "running"), ("enumeration", "done"), ("surface", "frozen"),
                         ("chain", "running"), ("chain", "done"), ("release", "released"), ("closed", "closed")]:
        seq.append(site.next_step(eng)["stage"])
        st.set_stage(eng, stage, value)
    seq.append(site.next_step(eng)["stage"])
    assert seq == ["letter", "intake", "materials", "enumeration", "enumeration", "surface",
                   "chain", "chain", "release", "report", "closed"]


def test_settings_write_the_engagement_file_letter_and_policy(env, monkeypatch):
    c, root = env
    _new(c)
    from client_ui import cf_access
    synced = []
    monkeypatch.setattr(cf_access, "ensure_emails", lambda emails: synced.append(list(emails)) or {"synced": True, "added": list(emails), "reason": "added"})
    mail.sent.clear()
    r = c.post("/p/api/engagements/e1/settings" + _as(PRACTICE), json={
        "claim_sources": ["README.md", " docs/llms.txt ", ""], "client_emails": [CLIENT, "second@example.test"],
        "target": "target", "letter": "# Our letter\n\nSigned.\n"})
    assert r.status_code == 200, r.text
    e = next(x for x in r.json()["engagements"] if x["name"] == "e1")
    assert e["claim_sources"] == ["README.md", "docs/llms.txt"]
    assert e["client_emails"] == [CLIENT, "second@example.test"]
    assert e["settings"]["letter"].startswith("# Our letter") and e["settings"]["letter_is_template"] is False
    assert r.json()["policy"]["added"] == ["second@example.test"]
    assert synced == [["second@example.test"]]                  # only the new address
    assert mail.sent[-1]["to"] == ["second@example.test"]
    assert "Our letter" in c.get("/e/e1/api/status" + _as(CLIENT)).json()["letter"]
    # an empty letter removes the engagement's own and shows the template again
    r = c.post("/p/api/engagements/e1/settings" + _as(PRACTICE), json={"letter": ""})
    assert r.json()["policy"] is None
    assert "Terms **here**" in c.get("/e/e1/api/status" + _as(CLIENT)).json()["letter"]
    assert st.claim_sources(root / "e1") == ["README.md", "docs/llms.txt"]     # untouched by a letter-only save
    assert c.post("/p/api/engagements/e1/settings" + _as(CLIENT), json={"letter": "x"}).status_code == 403


def test_a_conversation_that_cannot_start_says_so(tmp_path, monkeypatch):
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    monkeypatch.setenv("MAIL_DRY_RUN", "1"); monkeypatch.setenv("PRACTICE_EMAILS", PRACTICE)
    mail.sent.clear()
    def broken(key):
        raise RuntimeError("api_key env var 'FIREWORKS_API_KEY' is not set")
    c = TestClient(site.make_site_app(Access([PRACTICE], no_access=True), None, build=broken))
    st.new_engagement(tmp_path / "e1", client_emails=[CLIENT])
    r = c.get("/e/e1/intake/api/document" + _as(CLIENT))
    assert r.status_code == 503 and "FIREWORKS_API_KEY" in r.json()["detail"]
    with c.websocket_connect("/e/e1/intake/ws" + _as(CLIENT)) as ws:
        m = ws.receive_json()
        assert m["type"] == "error" and "could not be started" in m["text"]
    assert mail.sent[-1]["to"] == [PRACTICE] and "failed" in mail.sent[-1]["subject"]


def test_client_accepting_the_letter_notifies_the_practice(env):
    c, root = env
    _new(c)
    mail.sent.clear()
    c.post("/e/e1/api/letter/accept" + _as(PRACTICE))          # the practice's own click: no notice
    assert mail.sent == []
    c.post("/e/e1/api/letter/accept" + _as(CLIENT))
    assert mail.sent[-1]["to"] == [PRACTICE] and "accepted the letter" in mail.sent[-1]["subject"]
