"""
Shared session store for ScienceWorld tools.
"""

from typing import Any, Dict, Optional

_SESSIONS: Dict[str, Any] = {}


def set_session(session_id: str, env: Any) -> None:
    _SESSIONS[session_id] = env


def get_session(session_id: str) -> Optional[Any]:
    return _SESSIONS.get(session_id)


def delete_session(session_id: str) -> None:
    if session_id in _SESSIONS:
        del _SESSIONS[session_id]

