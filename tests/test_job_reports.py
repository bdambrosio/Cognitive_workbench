"""Reports from background jobs (agreed with Jill, 2026-10-03): a job writes
a report into the inbox; the sensor delivers each once, also after a
restart, and says once when an expected report has not arrived."""
import importlib.util
import sys
from datetime import datetime, timedelta
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from utils.job_report import write_report  # noqa: E402


def _load():
    spec = importlib.util.spec_from_file_location(
        "sensor_job_reports", SRC / "sensors/job-reports/sensor.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ctx(inbox, expected=()):
    return {'character_name': 'Tester',
            'parameters': {'inbox': str(inbox), 'expected': list(expected)}}


def test_each_report_is_one_turn_delivered_once_even_across_a_restart(tmp_path):
    t0 = datetime(2026, 10, 4, 8, 30)
    write_report(tmp_path, "outreach-daily", "ok", t0, "Ready to contact now: 4.",
                 finished=t0 + timedelta(minutes=31))
    write_report(tmp_path, "site-check", "failed", t0, "The site did not answer.",
                 finished=t0 + timedelta(minutes=40))
    first = _load().run(_ctx(tmp_path))          # a fresh process each time: no memory
    second = _load().run(_ctx(tmp_path))
    third = _load().run(_ctx(tmp_path))
    assert "job: outreach-daily" in first['content'] and "duration: 31 min" in first['content']
    assert "Ready to contact now: 4." in first['content'] and "site-check" not in first['content']
    assert "status: failed" in second['content'] and "The site did not answer." in second['content']
    assert third['status'] == 'nothing'
    assert len(list((tmp_path / 'seen').glob('*.txt'))) == 2


def test_a_report_still_being_written_is_not_delivered(tmp_path):
    (tmp_path / '.tmp').mkdir(parents=True)
    (tmp_path / '.tmp' / 'outreach-daily-2026-10-04T090100.txt').write_text("job: outr")
    assert _load().run(_ctx(tmp_path))['status'] == 'nothing'


def test_a_long_report_is_cut_and_says_where_the_whole_is(tmp_path):
    t0 = datetime(2026, 10, 4, 8, 30)
    write_report(tmp_path, "chain", "ok", t0, "x" * 10000, finished=t0)
    out = _load().run(_ctx(tmp_path))
    assert out['metadata']['truncated'] and "The report is cut here" in out['content']
    assert len(out['content']) < 4000


def test_a_missing_expected_report_is_said_once_and_not_when_it_arrived(tmp_path, monkeypatch):
    sensor = _load()
    expected = [{'job': 'outreach-daily', 'by': '10:00'}]

    class _Clock(datetime):
        at = datetime(2026, 10, 4, 9, 55)

        @classmethod
        def now(cls, tz=None):
            return cls.at
    monkeypatch.setattr(sensor, 'datetime', _Clock)
    assert sensor.run(_ctx(tmp_path, expected))['status'] == 'nothing'      # not due yet
    _Clock.at = datetime(2026, 10, 4, 10, 5)
    warned = sensor.run(_ctx(tmp_path, expected))
    assert warned['metadata'] == {'missing': 'outreach-daily'} and "due by 10:00" in warned['content']
    _Clock.at = datetime(2026, 10, 4, 10, 10)
    restarted = _load()                                                     # once, across a restart
    monkeypatch.setattr(restarted, 'datetime', _Clock)
    assert restarted.run(_ctx(tmp_path, expected))['status'] == 'nothing'
    # The next day the report arrives in time: delivered, and no warning after.
    write_report(tmp_path, "outreach-daily", "ok", datetime(2026, 10, 5, 8, 30), "done",
                 finished=datetime(2026, 10, 5, 9, 2))
    _Clock.at = datetime(2026, 10, 5, 10, 5)
    assert "job: outreach-daily" in sensor.run(_ctx(tmp_path, expected))['content']
    assert sensor.run(_ctx(tmp_path, expected))['status'] == 'nothing'


def test_a_status_other_than_ok_or_failed_is_refused(tmp_path):
    with pytest.raises(ValueError):
        write_report(tmp_path, "x", "partial", datetime.now(), "body")
