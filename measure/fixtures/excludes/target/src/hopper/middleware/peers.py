"""After every response, note who was served."""
import time


def record_peer(db):
    def _after(request, response):
        code = request.path.lstrip("/")
        if response.status == 302 and code:
            db.execute(
                "INSERT INTO peers (code, peer, client_string, served_at) VALUES (?, ?, ?, ?)",
                (code, request.remote_addr, request.headers.get("User-Agent", ""), time.time()),
            )
            db.commit()
        return response
    return _after
