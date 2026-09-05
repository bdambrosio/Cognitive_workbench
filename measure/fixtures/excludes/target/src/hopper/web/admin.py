"""The admin console: create links, see traffic."""
import time

from hopper.auth.passwords import verify
from hopper.store.codes import new_code


def handler(db, cfg):
    def _admin(request):
        if not verify(db, request.form.get("name"), request.form.get("password")):
            return 401, "who are you"
        if request.method == "POST":
            code = new_code(db, cfg)
            expiry_days = cfg["links"]["expiry_days"]
            expires_at = time.time() + expiry_days * 86400 if expiry_days > 0 else None
            db.execute("INSERT INTO links (code, url, created_at, expires_at) VALUES (?, ?, ?, ?)",
                       (code, request.form["url"], time.time(), expires_at))
            db.commit()
            return 201, code
        rows = db.execute("SELECT code, peer, client_string, served_at FROM peers ORDER BY served_at DESC LIMIT 200").fetchall()
        return 200, render_traffic(rows)
    return _admin


def render_traffic(rows):
    return "\n".join(f"{code} {peer} {client_string} {served_at}" for code, peer, client_string, served_at in rows)
