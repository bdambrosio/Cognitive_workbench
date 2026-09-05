"""The HTTP application: routes and middleware wiring."""
from hopper import settings
from hopper.middleware.peers import record_peer
from hopper.middleware.throttle import Throttle
from hopper.store.db import connect
from hopper.web import admin, redirect


def create_app():
    cfg = settings.load()
    db = connect(cfg["store"]["db_path"])
    app = Router()
    app.after_request(record_peer(db))
    throttle = Throttle(cfg["limits"]["requests_per_minute"])
    app.before_request(throttle.check)
    app.route("/<code>", redirect.handler(db, cfg))
    app.route("/admin", admin.handler(db, cfg))
    return app


class Router:
    def __init__(self):
        self._before, self._after, self._routes = [], [], {}

    def before_request(self, fn):
        self._before.append(fn)

    def after_request(self, fn):
        self._after.append(fn)

    def route(self, pattern, fn):
        self._routes[pattern] = fn
