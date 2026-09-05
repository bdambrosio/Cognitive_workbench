"""Short codes: random, from the configured alphabet, of the configured length."""
import secrets


def new_code(db, cfg) -> str:
    length = cfg["codes"]["length"]
    alphabet = cfg["codes"]["alphabet"]
    while True:
        code = "".join(secrets.choice(alphabet) for _ in range(length))
        if db.execute("SELECT 1 FROM links WHERE code = ?", (code,)).fetchone() is None:
            return code
