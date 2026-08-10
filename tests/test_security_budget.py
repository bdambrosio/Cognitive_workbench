"""Cost controls on the `security` subagent.

Both cases here come from one live incident (2026-08-10): a "what's on my
LAN" question put Sentinel into a per-host nmap sweep that blocked its
inbox loop for 20+ minutes, so every message to it — Jill's and the
user's — queued behind an in-flight turn with no reply.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import chat.security as sec


def test_scan_services_does_not_run_nse_scripts(monkeypatch):
    """`--script=safe` cost minutes per host and `-oG` discarded every
    line of its output (grepable output has no script field), so the parse
    below it never saw any. -F bounds the scan to the top-100 ports."""
    seen = {}

    class _Proc:
        returncode = 0
        stdout = "Host: 192.168.68.1 ()\tPorts: 22/open/tcp//ssh//OpenSSH 9.6/\n"
        stderr = ""

    def _fake_run(cmd, **kw):
        seen['cmd'] = cmd
        seen['timeout'] = kw.get('timeout')
        return _Proc()

    monkeypatch.setattr(sec.shutil, 'which', lambda _n: '/usr/bin/nmap')
    monkeypatch.setattr(sec.subprocess, 'run', _fake_run)
    out = sec._tool_scan_services('192.168.68.1')
    assert out.startswith('OK:')
    assert '--script=safe' not in seen['cmd']
    assert '-F' in seen['cmd'] and '-sV' in seen['cmd']
    assert seen['cmd'][-1] == '192.168.68.1'


def test_call_budget_refuses_probes_not_the_answer(tmp_path, monkeypatch):
    """Past the wall-clock budget, the next probe is refused and the model
    is told to answer with what it has and name the gap. The per-probe
    timeouts bound one probe; only this bounds the run."""
    calls = {'n': 0}
    probes = {'n': 0}

    class _Backend:
        def chat(self, messages, **kw):
            calls['n'] += 1
            if calls['n'] == 1:
                return '{"tool": "discover", "cidr": "192.168.68.0/24"}'
            if calls['n'] == 2:
                return '{"tool": "scan_services", "host": "192.168.68.1"}'
            return '{"tool": "respond", "text": "partial: only host discovery ran"}'

    def _fake_discover(cidr):
        probes['n'] += 1
        # First probe eats the entire budget.
        monkeypatch.setattr(sec.time, 'monotonic', lambda: 10_000.0)
        return 'OK: Host: 192.168.68.1 () Status: Up'

    def _boom(*a, **k):
        probes['n'] += 1
        raise AssertionError('a probe ran after the budget was spent')

    monkeypatch.setattr(sec.time, 'monotonic', lambda: 0.0)
    monkeypatch.setattr(sec, '_tool_discover', _fake_discover)
    monkeypatch.setattr(sec, '_tool_scan_services', _boom)

    answer = sec.security('what is on my LAN?', _Backend(), tmp_path)
    assert probes['n'] == 1              # the second probe never ran
    assert 'partial' in answer
    trace = '\n'.join(p.read_text() for p in tmp_path.glob('*'))
    assert 'used its whole time budget' in trace
