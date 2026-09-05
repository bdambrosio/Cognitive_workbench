"""The Access policy sync: unconfigured does nothing; configured, it adds
only the addresses the first Allow policy lacks, through a fake transport."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from client_ui import cf_access                                 # noqa: E402


def test_unconfigured_is_a_no_op(monkeypatch):
    for k in ("CF_API_TOKEN_FILE", "CF_ACCOUNT_ID", "CF_ACCESS_APP_ID"):
        monkeypatch.delenv(k, raising=False)
    r = cf_access.ensure_emails(["a@x.test"])
    assert r["synced"] is False and r["added"] == []


def test_adds_only_missing_emails(monkeypatch, tmp_path):
    tok = tmp_path / "t"; tok.write_text("secret\n")
    monkeypatch.setenv("CF_API_TOKEN_FILE", str(tok))
    monkeypatch.setenv("CF_ACCOUNT_ID", "acct1")
    monkeypatch.setenv("CF_ACCESS_APP_ID", "app1")
    calls = []
    policy = {"id": "pol1", "name": "practice and clients", "decision": "allow", "precedence": 1,
              "include": [{"email": {"email": "Bruce@Example.test"}}]}

    def fake(method, url, token, body=None):
        calls.append((method, url, token, body))
        if method == "GET":
            return {"success": True, "result": [
                {"id": "deny", "decision": "deny", "precedence": 0, "include": []}, policy]}
        return {"success": True, "result": body}
    monkeypatch.setattr(cf_access, "transport", fake)
    r = cf_access.ensure_emails(["bruce@example.test", "New@Client.test", " ", "new@client.test"])
    assert r == {"synced": True, "added": ["new@client.test"], "reason": "added"}
    put = calls[-1]
    assert put[0] == "PUT" and put[1].endswith("/accounts/acct1/access/apps/app1/policies/pol1")
    assert put[2] == "secret"
    assert put[3]["include"] == [{"email": {"email": "Bruce@Example.test"}},
                                 {"email": {"email": "new@client.test"}}]
    assert put[3]["decision"] == "allow" and "id" not in put[3]
    # nothing new: no PUT
    calls.clear()
    r = cf_access.ensure_emails(["bruce@example.test"])
    assert r["reason"] == "already allowed" and [c[0] for c in calls] == ["GET"]


def test_transport_failure_is_reported_not_raised(monkeypatch, tmp_path):
    tok = tmp_path / "t"; tok.write_text("secret")
    monkeypatch.setenv("CF_API_TOKEN_FILE", str(tok))
    monkeypatch.setenv("CF_ACCOUNT_ID", "a"); monkeypatch.setenv("CF_ACCESS_APP_ID", "b")

    def boom(*a, **k):
        raise OSError("network down")
    monkeypatch.setattr(cf_access, "transport", boom)
    r = cf_access.ensure_emails(["a@x.test"])
    assert r["synced"] is False and "network down" in r["reason"]
