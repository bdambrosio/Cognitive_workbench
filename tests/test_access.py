"""Identity and roles: a Cloudflare Access token verified against a key and
an audience, the no-access mode, the practice and client roles."""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workflowsv2 import engagement_state as st                  # noqa: E402
from client_ui import access as acc                             # noqa: E402

jwt = pytest.importorskip("jwt")
pytest.importorskip("cryptography")


@pytest.fixture(scope="module")
def keys():
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    priv = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    pem_priv = priv.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                  serialization.NoEncryption())
    pem_pub = priv.public_key().public_bytes(serialization.Encoding.PEM,
                                             serialization.PublicFormat.SubjectPublicKeyInfo)
    return pem_priv, pem_pub


def _token(pem_priv, aud="aud1", email="Client@Example.test", exp=None):
    return jwt.encode({"email": email, "aud": aud, "exp": exp or int(time.time()) + 60},
                      pem_priv, algorithm="RS256")


def test_decode_checks_signature_and_audience(keys):
    priv, pub = keys
    assert acc.decode(_token(priv), pub, "aud1")["email"] == "Client@Example.test"
    with pytest.raises(Exception):
        acc.decode(_token(priv, aud="other"), pub, "aud1")
    with pytest.raises(Exception):
        acc.decode(_token(priv, exp=int(time.time()) - 5), pub, "aud1")
    with pytest.raises(Exception):
        acc.decode(_token(priv) + "x", pub, "aud1")


def test_access_email_from_header_or_cookie_and_roles(keys, tmp_path, monkeypatch):
    priv, pub = keys
    a = acc.Access(["Bruce@Example.test"], verifier=lambda t: acc.decode(t, pub, "aud1"))
    tok = _token(priv)
    assert a.email({acc.HEADER: tok}, {}, {}) == "client@example.test"
    assert a.email({}, {acc.COOKIE: tok}, {}) == "client@example.test"
    assert a.email({}, {}, {"as": "x@y"}) is None                       # ?as= means nothing with access on
    assert a.email({acc.HEADER: tok + "bad"}, {}, {}) is None
    monkeypatch.setattr(st, "ENGAGEMENTS", tmp_path)
    e1 = st.new_engagement(tmp_path / "e1", client_emails=["client@example.test"])
    st.new_engagement(tmp_path / "e2", client_emails=["other@example.test"])
    assert a.role("bruce@example.test", e1) == "practice"
    assert a.role("client@example.test", e1) == "client"
    assert a.role("other@example.test", e1) is None
    assert a.role(None, e1) is None
    assert a.engagements_for("bruce@example.test") == ["e1", "e2"]
    assert a.engagements_for("client@example.test") == ["e1"]
    assert a.engagements_for("nobody@example.test") == []


def test_no_access_mode_and_missing_config():
    a = acc.Access(["p@x"], no_access=True)
    assert a.email({}, {}, {"as": " P@X "}) == "p@x"
    assert a.email({}, {acc.LOCAL_COOKIE: "c@x"}, {}) == "c@x"
    assert a.email({}, {}, {}) is None
    with pytest.raises(SystemExit):
        acc.Access(["p@x"])                                            # no team, no aud, access on
    assert acc.certs_url("tuuyi") == "https://tuuyi.cloudflareaccess.com/cdn-cgi/access/certs"
