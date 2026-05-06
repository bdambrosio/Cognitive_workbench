#!/usr/bin/env python3
"""
Telegram bridge — relay between a Telegram bot and a chat-mode character.

Standalone subprocess (like resource_browser.py). Holds a long-poll
connection to api.telegram.org, with no inbound port and no webhook.

Wire topology (no chat_loop changes — the existing pub/sub is correct):
  inbound:  Telegram message → publish to cognitive/{character}/sense_data
            using the same envelope shape the CLI uses.
  outbound: subscribe to cognitive/{character}/action; for type=='say'
            forward the text to the configured Telegram chat.

Authorization is a hard allowlist of Telegram numeric user IDs
(from.id). Non-allowlisted senders are silently dropped — the bridge
does not respond, so a passing stranger who finds the bot's username
gets no signal that the bot is active. /start from an allowlisted
sender gets a one-line acknowledgement; everything else passes
straight through to the character.

Required environment:
  TELEGRAM_BOT_TOKEN         — bot token from @BotFather
  TELEGRAM_ALLOWED_CHAT_IDS  — comma-separated numeric user IDs

Failure modes are loud at startup: missing token, empty allowlist,
or unparseable allowlist all SystemExit before opening Zenoh.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional, Set

import requests
import zenoh


_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
_LOG_DIR.mkdir(parents=True, exist_ok=True)
_file_handler = logging.FileHandler(_LOG_DIR / "telegram_bridge.log", mode="a")
_file_handler.setLevel(logging.INFO)
_console_handler = logging.StreamHandler()
_console_handler.setLevel(logging.WARNING)
_fmt = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
_file_handler.setFormatter(_fmt)
_console_handler.setFormatter(_fmt)
logging.basicConfig(
    level=logging.INFO,
    handlers=[_file_handler, _console_handler],
    force=True,
)
logger = logging.getLogger("telegram_bridge")


API_BASE = "https://api.telegram.org"
TG_MAX_MESSAGE_CHARS = 4096   # Telegram hard limit
SEND_CHUNK = 4000             # leave room for any client-added prefixes
LONG_POLL_SECONDS = 25        # server-side wait; client timeout is +buffer
HTTP_TIMEOUT = LONG_POLL_SECONDS + 10


class TelegramBridge:
    def __init__(
        self,
        character: str,
        token: str,
        allowlist: Set[int],
        default_chat_id: int,
    ):
        self.character = character
        self.token = token
        self.allowlist = allowlist
        self.default_chat_id = default_chat_id
        self.last_update_id = 0
        self._stop = False

        self.session = zenoh.open(zenoh.Config())
        self.sense_pub = self.session.declare_publisher(
            f"cognitive/{character}/sense_data"
        )
        self.action_sub = self.session.declare_subscriber(
            f"cognitive/{character}/action",
            self._on_action,
        )

    # ---- shutdown ----------------------------------------------------

    def stop(self) -> None:
        self._stop = True

    # ---- outbound: zenoh action → Telegram ---------------------------

    def _on_action(self, sample) -> None:
        try:
            payload = json.loads(bytes(sample.payload).decode("utf-8"))
        except Exception as e:
            logger.warning(f"action decode failed: {e}")
            return
        if payload.get("type") != "say":
            return
        text = (payload.get("text") or "").strip()
        if not text:
            return
        self._send(self.default_chat_id, text)

    def _send(self, chat_id: int, text: str) -> None:
        url = f"{API_BASE}/bot{self.token}/sendMessage"
        # Chunk on Telegram's 4096-char hard limit. We split at SEND_CHUNK
        # to leave headroom; long agent replies arrive as multiple
        # consecutive messages rather than one truncated message.
        chunks = [text[i:i + SEND_CHUNK]
                  for i in range(0, len(text), SEND_CHUNK)] or [""]
        for chunk in chunks:
            try:
                resp = requests.post(
                    url,
                    json={
                        "chat_id": chat_id,
                        "text": chunk,
                        "disable_web_page_preview": True,
                    },
                    timeout=30,
                )
                if not resp.ok:
                    logger.warning(
                        f"sendMessage to {chat_id} failed: "
                        f"{resp.status_code} {resp.text[:300]}"
                    )
            except Exception as e:
                logger.warning(f"sendMessage to {chat_id} raised: {e}")

    # ---- inbound: Telegram → zenoh sense_data ------------------------

    def _publish_user_text(self, source: str, text: str) -> None:
        envelope = {
            "mode": "text",
            "content": json.dumps({
                "source": source,
                "text": text,
                "close": False,
            }),
        }
        try:
            self.sense_pub.put(json.dumps(envelope).encode("utf-8"))
        except Exception as e:
            logger.warning(f"sense_data put failed: {e}")

    def _handle_message(self, msg: dict) -> None:
        chat = msg.get("chat") or {}
        sender = msg.get("from") or {}
        sender_id = sender.get("id")
        chat_type = chat.get("type")
        text = (msg.get("text") or "").strip()

        # Hard authorization gate. Group/channel chats are refused
        # categorically — the bridge only operates 1:1 with allowlisted
        # users. Silent drop avoids advertising the bot's activity.
        if chat_type != "private":
            logger.info(
                f"drop non-private chat (type={chat_type}, sender={sender_id})"
            )
            return
        if sender_id not in self.allowlist:
            logger.info(f"drop unauthorized sender {sender_id}")
            return
        if not text:
            return

        # Bridge-handled commands. /start is the conventional first
        # message; replying with a one-liner saves the character from
        # processing an empty introduction turn.
        if text == "/start":
            self._send(
                sender_id,
                f"Connected to {self.character}. Type to chat; "
                "messages here are forwarded as user input.",
            )
            return

        sender_label = (sender.get("username")
                        or sender.get("first_name")
                        or str(sender_id))
        logger.info(
            f"sense_data from telegram:{sender_id} ({sender_label}): "
            f"{text[:200]!r}"
        )
        self._publish_user_text(
            source=f"User@telegram:{sender_label}",
            text=text,
        )

    # ---- main loop ---------------------------------------------------

    def run(self) -> None:
        url = f"{API_BASE}/bot{self.token}/getUpdates"
        logger.info(
            f"[{self.character}] telegram bridge ready "
            f"(allowlist={sorted(self.allowlist)}, "
            f"default_chat_id={self.default_chat_id})"
        )
        backoff = 1.0
        while not self._stop:
            try:
                resp = requests.get(
                    url,
                    params={
                        "timeout": LONG_POLL_SECONDS,
                        "offset": self.last_update_id + 1,
                    },
                    timeout=HTTP_TIMEOUT,
                )
                if not resp.ok:
                    logger.warning(
                        f"getUpdates failed: {resp.status_code} "
                        f"{resp.text[:300]}"
                    )
                    time.sleep(min(backoff, 30.0))
                    backoff = min(backoff * 2, 30.0)
                    continue
                data = resp.json()
                if not data.get("ok"):
                    logger.warning(f"getUpdates not ok: {data}")
                    time.sleep(min(backoff, 30.0))
                    backoff = min(backoff * 2, 30.0)
                    continue
                backoff = 1.0
                for upd in data.get("result", []):
                    self.last_update_id = max(
                        self.last_update_id, int(upd.get("update_id", 0))
                    )
                    msg = upd.get("message")
                    if msg:
                        self._handle_message(msg)
            except requests.exceptions.RequestException as e:
                logger.warning(f"poll error: {e}")
                time.sleep(min(backoff, 30.0))
                backoff = min(backoff * 2, 30.0)
            except Exception as e:
                logger.error(f"unexpected error: {e}")
                time.sleep(min(backoff, 30.0))
                backoff = min(backoff * 2, 30.0)


def parse_allowlist(spec: str) -> Set[int]:
    out: Set[int] = set()
    for piece in (spec or "").split(","):
        piece = piece.strip()
        if not piece:
            continue
        try:
            out.add(int(piece))
        except ValueError:
            raise SystemExit(
                f"telegram_bridge: invalid chat_id in allowlist: {piece!r}"
            )
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--character",
        default="Jill",
        help="character name (matches cognitive/<name>/* topics)",
    )
    ap.add_argument(
        "--default-chat-id",
        type=int,
        default=None,
        help="chat_id to send the character's outbound say events to "
             "(defaults to the first allowlist entry)",
    )
    args = ap.parse_args()

    token = os.getenv("TELEGRAM_BOT_TOKEN", "").strip()
    if not token:
        raise SystemExit(
            "telegram_bridge: TELEGRAM_BOT_TOKEN env var is not set"
        )

    allowlist = parse_allowlist(os.getenv("TELEGRAM_ALLOWED_CHAT_IDS", ""))
    if not allowlist:
        raise SystemExit(
            "telegram_bridge: TELEGRAM_ALLOWED_CHAT_IDS env var is empty — "
            "refusing to start a publicly-DM-able bridge"
        )

    default_chat_id = (
        args.default_chat_id
        if args.default_chat_id is not None
        else next(iter(sorted(allowlist)))
    )
    if default_chat_id not in allowlist:
        raise SystemExit(
            f"telegram_bridge: --default-chat-id {default_chat_id} "
            f"is not in TELEGRAM_ALLOWED_CHAT_IDS"
        )

    bridge = TelegramBridge(args.character, token, allowlist, default_chat_id)

    def _shutdown(_sig, _frame):
        logger.info("shutdown signal received")
        bridge.stop()
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        bridge.run()
    finally:
        logger.info("bridge exiting")
        try:
            bridge.session.close()
        except Exception as e:
            logger.warning(f"zenoh close raised: {e}")


if __name__ == "__main__":
    main()
