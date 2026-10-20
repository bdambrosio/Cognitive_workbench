import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from utils.json_utils import repair_json_string  # noqa: E402


def test_a_string_value_starting_with_a_signed_number_parses_unchanged():
    # Valid JSON; the arithmetic repair used to drop the value's opening
    # quote and return None (claim attribution, grader output, 2026-10-20).
    assert repair_json_string('{"pct": "-0.29%"}') == {"pct": "-0.29%"}
