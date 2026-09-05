"""SQLite connection and migrations."""
import sqlite3
from pathlib import Path

MIGRATIONS = Path(__file__).resolve().parents[3] / "migrations"


def connect(path: str) -> sqlite3.Connection:
    db = sqlite3.connect(path)
    for sql in sorted(MIGRATIONS.glob("*.sql")):
        try:
            db.executescript(sql.read_text())
        except sqlite3.OperationalError:
            pass  # already applied
    return db
