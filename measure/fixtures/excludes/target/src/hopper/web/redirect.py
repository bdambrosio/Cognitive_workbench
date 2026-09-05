"""GET /<code>: look the code up and send the visitor on."""
import time


def handler(db, cfg):
    def _redirect(request, code):
        row = db.execute("SELECT url, expires_at FROM links WHERE code = ?", (code,)).fetchone()
        if row is None:
            return 404, "no such link"
        url, expires_at = row
        if expires_at is not None and expires_at < time.time():
            return 410, "link expired"
        db.execute("UPDATE links SET hits = hits + 1 WHERE code = ?", (code,))
        db.commit()
        return 302, url
    return _redirect
