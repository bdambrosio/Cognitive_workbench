#!/usr/bin/env python3
"""
One-time curation: build the bundled sayings pool for the ramana-saying tool.

Reads the verbatim, reviewed Talks corpus and extracts single
question -> answer exchanges into a self-contained data/sayings.jsonl so the
runtime tool has no dependency on the (separate) /data/satsang repo.

Run by hand when refreshing the pool:
    python src/tools/ramana-saying/build_pool.py

Each output record: {"id", "talk", "question", "answer"}.
"""

import json
import re
from pathlib import Path

SOURCE = Path(
    "/data/satsang/ramana/Talks-with-Sri-Ramana-Maharshi-parsed-reviewed-merged.jsonl"
)
OUT = Path(__file__).parent / "data" / "sayings.jsonl"

# Keep answers substantial but quotable. Word bounds, not content filters.
MIN_ANSWER_WORDS = 8
MAX_ANSWER_WORDS = 130

_TALK_NUM = re.compile(r"talk_(\d+)")

# Drop answers that end mid-sentence (chunk-boundary truncation in the source):
# require terminal sentence punctuation, allowing a trailing quote/paren.
_ENDS_SENTENCE = re.compile(r"[.?!][\"')”’]?$")


def talk_label(record_id: str) -> str | None:
    """'talk_18_chunk2' -> 'Talk 18'."""
    m = _TALK_NUM.search(record_id or "")
    return f"Talk {m.group(1)}" if m else None


def main() -> None:
    if not SOURCE.exists():
        raise SystemExit(f"Source corpus not found: {SOURCE}")

    OUT.parent.mkdir(parents=True, exist_ok=True)

    seen: set[tuple[str, str]] = set()
    sayings: list[dict] = []

    for line in SOURCE.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        record = json.loads(line)
        talk = talk_label(record.get("id", ""))
        if not talk:
            continue
        messages = record.get("messages", [])
        # Walk adjacent human -> assistant pairs.
        for i in range(len(messages) - 1):
            if messages[i].get("role") != "human":
                continue
            if messages[i + 1].get("role") != "assistant":
                continue
            question = (messages[i].get("content") or "").strip()
            answer = (messages[i + 1].get("content") or "").strip()
            n_words = len(answer.split())
            if n_words < MIN_ANSWER_WORDS or n_words > MAX_ANSWER_WORDS:
                continue
            if not _ENDS_SENTENCE.search(answer):
                continue
            # Dedup across duplicate/chunked records by talk + answer prefix.
            key = (talk, answer[:60])
            if key in seen:
                continue
            seen.add(key)
            sayings.append(
                {
                    "id": f"{talk.replace(' ', '_').lower()}#{len(sayings)}",
                    "talk": talk,
                    "question": question,
                    "answer": answer,
                }
            )

    with open(OUT, "w") as f:
        for s in sayings:
            f.write(json.dumps(s, ensure_ascii=False) + "\n")

    print(f"Wrote {len(sayings)} sayings to {OUT}")


if __name__ == "__main__":
    main()
