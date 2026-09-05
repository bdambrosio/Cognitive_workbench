"""Who is asking: the email Cloudflare Access verified, and what it may see.

Cloudflare Access sits in front of client.tuuyi.com and, once a visitor has
passed its one-time-code login, sends every request through with a signed
JWT in the `Cf-Access-Jwt-Assertion` header (and the `CF_Authorization`
cookie, which is what a websocket upgrade carries). The token is verified
here against the team's public keys and the application's audience tag, so
a request that reached this process some other way carries no identity.

    CF_ACCESS_TEAM     the Zero Trust team name (<team>.cloudflareaccess.com)
    CF_ACCESS_AUD      the Access application's audience tag
    PRACTICE_EMAILS    comma-separated; these emails are the practice

Roles: `practice` for an email in PRACTICE_EMAILS; `client` for an email in
the engagement's `client_emails`; None otherwise. A practice email may open
every engagement.

NO ACCESS, FOR LOCAL WORK. With `no_access=True` the identity is the `as`
query parameter or the `site_as` cookie, and the site must be bound to the
loopback address. Tests run that way.
"""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional

logger = logging.getLogger("client_ui.access")

HEADER = "cf-access-jwt-assertion"
COOKIE = "CF_Authorization"
LOCAL_COOKIE = "site_as"


def certs_url(team: str) -> str:
    return f"https://{team}.cloudflareaccess.com/cdn-cgi/access/certs"


def decode(token: str, key: Any, aud: str) -> Dict[str, Any]:
    """The claims of a token signed with `key` for audience `aud`; raises
    on any failure. `key` is what PyJWT accepts (a JWK's signing key, or a
    PEM public key in tests)."""
    import jwt
    return jwt.decode(token, key, algorithms=["RS256"], audience=aud)


def cloudflare_verifier(team: str, aud: str) -> Callable[[str], Dict[str, Any]]:
    """A verifier that fetches (and caches) the team's signing keys."""
    import jwt
    client = jwt.PyJWKClient(certs_url(team), cache_keys=True)

    def verify(token: str) -> Dict[str, Any]:
        key = client.get_signing_key_from_jwt(token).key
        return decode(token, key, aud)
    return verify


class Access:
    def __init__(self, practice_emails: List[str], no_access: bool = False,
                 verifier: Optional[Callable[[str], Dict[str, Any]]] = None,
                 team: Optional[str] = None, aud: Optional[str] = None) -> None:
        self.practice = {e.strip().lower() for e in practice_emails if e.strip()}
        self.no_access = no_access
        if no_access:
            self.verifier = None
        elif verifier is not None:
            self.verifier = verifier
        else:
            if not (team and aud):
                raise SystemExit("CF_ACCESS_TEAM and CF_ACCESS_AUD are required unless "
                                 "--no-access is given")
            self.verifier = cloudflare_verifier(team, aud)

    @classmethod
    def from_env(cls, no_access: bool = False) -> "Access":
        emails = [e for e in os.environ.get("PRACTICE_EMAILS", "").split(",")]
        return cls(emails, no_access=no_access,
                   team=os.environ.get("CF_ACCESS_TEAM"),
                   aud=os.environ.get("CF_ACCESS_AUD"))

    def email(self, headers: Mapping[str, str], cookies: Mapping[str, str],
              query: Mapping[str, str]) -> Optional[str]:
        """The verified email of this request, or None."""
        if self.no_access:
            e = query.get("as") or cookies.get(LOCAL_COOKIE)
            return e.strip().lower() if e else None
        token = headers.get(HEADER) or cookies.get(COOKIE)
        if not token:
            return None
        try:
            claims = self.verifier(token)                      # type: ignore[misc]
        except Exception as e:                                 # noqa: BLE001
            logger.warning("access token rejected: %s", e)
            return None
        e = claims.get("email")
        return e.strip().lower() if isinstance(e, str) and e.strip() else None

    def role(self, email: Optional[str], eng_dir: Optional[Path]) -> Optional[str]:
        if not email:
            return None
        if email in self.practice:
            return "practice"
        if eng_dir is not None:
            from workflowsv2 import engagement_state as state
            if email in state.client_emails(eng_dir):
                return "client"
        return None

    def engagements_for(self, email: Optional[str]) -> List[str]:
        """The engagements this email may open: all of them for the
        practice, those naming it for a client."""
        from workflowsv2 import engagement_state as state
        if not email:
            return []
        names = state.engagements()
        if email in self.practice:
            return names
        return [n for n in names if email in state.client_emails(state.ENGAGEMENTS / n)]
