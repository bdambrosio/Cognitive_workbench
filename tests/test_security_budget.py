"""Cost controls on the `security` subagent.

Both cases here come from one live incident (2026-08-10): a "what's on my
LAN" question put Sentinel into a per-host nmap sweep that blocked its
inbox loop for 20+ minutes, so every message to it — Jill's and the
user's — queued behind an in-flight turn with no reply.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import chat.subagents.security as sec


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
    # Cost is driven by unresponsive hosts, not port count: -F alone still
    # ran past 130s on a filtered gateway. -Pn drops a redundant discovery
    # pass (discover already proved the host up), --max-retries trims the
    # retransmit ladder, and --host-timeout is what lets nmap RETURN its
    # partial results instead of being killed with everything discarded.
    assert '-Pn' in seen['cmd']
    assert '--host-timeout' in seen['cmd']
    assert seen['cmd'][seen['cmd'].index('--max-retries') + 1] == '1'
    # The inner cap must fire before the outer one, or the data is lost.
    assert sec._NMAP_HOST_TIMEOUT < seen['timeout'] == sec._NMAP_TIMEOUT_SERVICES


def test_host_timeout_with_no_ports_is_empty_not_ok(monkeypatch):
    """--host-timeout expiring is a clean exit carrying only a Status line.
    Rendered as OK it is a scan that learned nothing reported as success."""
    class _Proc:
        returncode = 0
        stdout = ("Host: 192.168.68.1 (_gateway)\tStatus: Up\n"
                  "Host: 192.168.68.1 (_gateway)\tStatus: Timeout\n")
        stderr = ""

    monkeypatch.setattr(sec.shutil, 'which', lambda _n: '/usr/bin/nmap')
    monkeypatch.setattr(sec.subprocess, 'run', lambda cmd, **kw: _Proc())
    out = sec._tool_scan_services('192.168.68.1')
    assert out.startswith('EMPTY:')
    assert 'do not retry it unchanged' in out


def test_a_shared_deadline_is_not_reset_by_a_second_call(tmp_path, monkeypatch):
    """The per-call cap bounds nothing on its own: the parent ReAct loop
    can call security() again and get a fresh budget (it did, twice in one
    turn). A caller passing one deadline for the whole turn must see the
    second call inherit what is left — here, nothing."""
    probes = {'n': 0}

    class _Backend:
        def chat(self, messages, **kw):
            if 'BUDGET' in messages[-1]['content'] or 'time budget' in messages[-1]['content']:
                return '{"tool": "respond", "text": "out of budget"}'
            return '{"tool": "discover", "cidr": "192.168.68.0/24"}'

    def _probe(_cidr):
        probes['n'] += 1
        return 'OK: Host: 192.168.68.1 () Status: Up'

    monkeypatch.setattr(sec, '_tool_discover', _probe)
    monkeypatch.setattr(sec.time, 'monotonic', lambda: 0.0)
    shared = sec.call_deadline()          # 0 + _CALL_BUDGET
    # Turn's clock is now spent; a fresh call must not restart it.
    monkeypatch.setattr(sec.time, 'monotonic', lambda: sec._CALL_BUDGET + 1.0)
    answer = sec.security('what is on my LAN?', _Backend(), tmp_path,
                          deadline=shared)
    assert probes['n'] == 0
    assert 'budget' in answer.lower()


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
