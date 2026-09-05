"""Admin passwords: salted SHA-256 digests in the admins table."""
import hashlib
import hmac
import secrets


def digest(password: str, salt: str) -> str:
    return hashlib.sha256((salt + password).encode()).hexdigest()


def create(db, name: str, password: str) -> None:
    salt = secrets.token_hex(8)
    db.execute("INSERT INTO admins (name, digest, salt) VALUES (?, ?, ?)", (name, digest(password, salt), salt))
    db.commit()


def verify(db, name, password) -> bool:
    if not name or not password:
        return False
    row = db.execute("SELECT digest, salt FROM admins WHERE name = ?", (name,)).fetchone()
    if row is None:
        return False
    return hmac.compare_digest(row[0], digest(password, row[1]))
