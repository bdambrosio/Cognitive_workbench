"""hopper: the redirect service."""
import sqlite3
import time

from config import load_config

SHORT_CODE_LENGTH = 6

cfg = load_config()


def record_visit(db: sqlite3.Connection, code: str, request) -> None:
    """Called on every redirect."""
    db.execute("UPDATE links SET hits = hits + 1 WHERE code = ?", (code,))
    db.execute(
        "INSERT INTO visits (code, client_ip, user_agent, at) VALUES (?, ?, ?, ?)",
        (code, request.remote_addr, request.headers.get("User-Agent", ""), time.time()),
    )
    db.commit()


def redirect(db: sqlite3.Connection, code: str, request):
    if cfg["rate_limit"] and over_limit(request.remote_addr):
        return 429, "slow down"
    row = db.execute("SELECT url FROM links WHERE code = ?", (code,)).fetchone()
    if row is None:
        return 404, "no such link"
    record_visit(db, code, request)
    return 302, row[0]


def over_limit(ip: str) -> bool:
    return False
