"""Run the bridge:  .venv/bin/python -m bridge   (from factorio/)"""
import logging
import os

import uvicorn

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

from bridge.app import app  # noqa: E402  (logging configured first)

uvicorn.run(app, host="127.0.0.1", port=int(os.environ.get("BRIDGE_PORT", "3004")), log_level="info")
