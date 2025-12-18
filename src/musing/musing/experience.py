import json
import logging
import os
from datetime import datetime
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class ExperienceLogger:
    """Append-only JSONL logger for musing events."""

    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)

    def log_event(self, event: Dict[str, Any]):
        try:
            event["timestamp"] = datetime.utcnow().isoformat() + "Z"
            with open(self.path, "a", encoding="utf-8") as f:
                json.dump(event, f, ensure_ascii=False)
                f.write("\n")
        except Exception as exc:  # pragma: no cover - best-effort logging
            logger.debug("Failed to log musing event: %s", exc)
